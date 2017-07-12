#include "reconstruction.h"
#include "cgal/aabbtree.h"
#include "common.h"
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wunused-variable" //Doesn't work on gcc < 6.0
#include <cinolib/smoothing.h>
#pragma GCC diagnostic pop
#endif //__GNUC__

#include <cinolib_interface/mesh_conversions.h>

std::map< const Dcel::Vertex*,int > Reconstruction::getMappingId(const Dcel& smoothedSurface, const HeightfieldsList& he) {
    std::map< const Dcel::Vertex*,int > mapping;
    CGALInterface::AABBTree tree(smoothedSurface);
    //int referenced = 0;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const EigenMesh m = he.getHeightfield(i);
        for (unsigned int j = 0; j < m.getNumberVertices(); j++){
            const Dcel::Vertex* v = tree.getNearestDcelVertex(m.getVertex(j));
            assert(v != nullptr);
            if (v->getCoordinate().dist(m.getVertex(j)) == 0){
                mapping[v] = i;
                //referenced++;
            }
        }
    }

    //std::cerr << "Unreferenced vertices: " << smoothedSurface.getNumberVertices() - referenced << "\n";

    return mapping;
}

std::vector< std::pair<int,int> > Reconstruction::getMapping(const Dcel& smoothedSurface, const HeightfieldsList& he) {
    std::vector< std::pair<int,int> > mapping;
    mapping.resize(smoothedSurface.getNumberVertices(), std::pair<int,int>(-1,-1));
    CGALInterface::AABBTree tree(smoothedSurface);
    int referenced = 0;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const EigenMesh m = he.getHeightfield(i);
        for (unsigned int j = 0; j < m.getNumberVertices(); j++){
            const Dcel::Vertex* v = tree.getNearestDcelVertex(m.getVertex(j));
            assert(v != nullptr);
            if (v->getCoordinate().dist(m.getVertex(j)) == 0){
                Vec3 target = he.getTarget(i);
                for (int k  = 0; k < 6; k++) {
                    if (target == XYZ[k]){ // it happens just one time for every target
                        std::pair<int,int> p;
                        p.first = i; //heightfield
                        p.second = k; //vertex
                        mapping[v->getId()] = p;
                        referenced++;
                    }
                }
            }
        }
    }

    std::cerr << "Unreferenced vertices: " << smoothedSurface.getNumberVertices() - referenced << "\n";

    return mapping;
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool Reconstruction::validate_move(const cinolib::Trimesh & m, const int vid, const int hf, const int dir, const cinolib::vec3d & vid_new_pos, const BoxList &boxList)
{
    cinolib::vec3d vertex = m.vertex(vid);
    if (hf < 0 || ! boxList[hf].isEpsilonIntern(Pointd(vertex.x(), vertex.y(), vertex.z()), -0.5))
        return false;
    /*if (hf >= 0) {
        for (int i = 0; i < hf; i++){
            if (boxList.getBox(i).isEpsilonIntern(Pointd(vertex.x(), vertex.y(), vertex.z())))
                return false;
        }
    }*/
    for(int tid : m.adj_vtx2tri(vid))
    {
        cinolib::vec3d tri[3];
        for(int offset=0; offset< 3; ++offset)
        {
            int nbr = m.triangle_vertex_id(tid, offset);
            tri[offset] = (vid == nbr) ? vid_new_pos : m.vertex(nbr);
        }
        cinolib::vec3d n = triangle_normal(tri[0], tri[1], tri[2]);

        switch (dir)
        {
            case -1: return false; break;
            case 0 :
                if (n.dot(cinolib::vec3d( 1, 0, 0)) < FLIP_ANGLE)
                    return false;
                break;
            case 1 :
                if (n.dot(cinolib::vec3d( 0, 1, 0)) < FLIP_ANGLE)
                    return false;
                break;
            case 2 :
                if (n.dot(cinolib::vec3d( 0, 0, 1)) < FLIP_ANGLE)
                    return false;
                break;
            case 3 :
                if (n.dot(cinolib::vec3d(-1, 0, 0)) < FLIP_ANGLE)
                    return false;
                break;
            case 4 :
                if (n.dot(cinolib::vec3d( 0,-1, 0)) < FLIP_ANGLE)
                    return false;
                break;
            case 5 :
                if (n.dot(cinolib::vec3d( 0, 0,-1)) < FLIP_ANGLE)
                    return false;
                break;
            default: assert(false);
        }
    }
    return true;

}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void Reconstruction::differential_coordinates(const cinolib::Trimesh & m, std::vector<cinolib::vec3d> & diff_coords)
{
    assert(diff_coords.empty());
    diff_coords.resize(m.num_vertices());

    #pragma omp parallel for
    for(int vid=0; vid<m.num_vertices(); ++vid)
    {
        double w    = 1.0 / double(m.vertex_valence(vid));
        cinolib::vec3d  curr = m.vertex(vid);
        cinolib::vec3d  delta(0,0,0);
        for(int nbr : m.adj_vtx2vtx(vid))
        {
            delta += w * (curr - m.vertex(nbr));
        }
        diff_coords[vid] = delta;
    }
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

void Reconstruction::restore_high_frequencies_gauss_seidel(cinolib::Trimesh          & m_smooth,
                                           const cinolib::Trimesh          & m_detail,
                                           const std::vector< std::pair<int, int> > & hf_directions,
                                           const BoxList &boxList,
                                           const int n_iters)
{
    std::vector<cinolib::vec3d> diff_coords;
    differential_coordinates(m_detail, diff_coords);

    for(int i=0; i<n_iters; ++i)
    {
        std::cerr << "iter " << i << std::endl;//<< "; nv: " << m_smooth.num_vertices() <<std::endl;

        #pragma omp parallel for
        for(int vid=0; vid<m_smooth.num_vertices(); ++vid)
        {
            //std::cerr << vid << "\n";
            cinolib::vec3d  gauss_iter(0,0,0);
            double w = 1.0 / double(m_smooth.vertex_valence(vid));
            for(int nbr : m_smooth.adj_vtx2vtx(vid))
            {
                gauss_iter += w * m_smooth.vertex(nbr);
            }

            cinolib::vec3d new_pos = diff_coords.at(vid) + gauss_iter;

            // do binary search until the new pos does not violate the hf condition...
            int count = 0;
            while(!validate_move(m_smooth, vid, hf_directions.at(vid).first, hf_directions.at(vid).second, new_pos, boxList) && ++count<5)
            {
                new_pos = 0.5 * (new_pos + m_smooth.vertex(vid));
            }

            if (count < 5) m_smooth.set_vertex(vid, new_pos);
        }
    }
}

Dcel Reconstruction::taubinSmoothing(const SimpleEigenMesh& m, int n_iters, double lambda, const double mu) {
    cinolib::Trimesh trimesh;
    MeshConversions::eigenMeshToTrimesh(trimesh, m);
    cinolib::smooth_taubin(trimesh, cinolib::COTANGENT, std::set<int>(), n_iters, lambda, mu);
    //trimesh.save("smoothed.obj");
    Dcel d(trimesh);
    return d;
}

Dcel Reconstruction::taubinSmoothing(const Dcel& d, int n_iters, double lambda, const double mu) {
    cinolib::Trimesh trimesh;
    MeshConversions::dcelToTrimesh(trimesh, d);
    cinolib::smooth_taubin(trimesh, cinolib::COTANGENT, std::set<int>(), n_iters, lambda, mu);
    Dcel d1(trimesh);
    return d1;
}

void Reconstruction::reconstruction(Dcel& smoothedSurface, const std::vector<std::pair<int, int>>& mapping, const EigenMesh& originalSurface, const BoxList &bl) {
    SimpleEigenMesh tmp(smoothedSurface);
    cinolib::logger.disable();
    cinolib::Trimesh smoothedTrimesh;
    cinolib::Trimesh originalTrimesh;
    MeshConversions::eigenMeshToTrimesh(smoothedTrimesh, tmp);
    MeshConversions::eigenMeshToTrimesh(originalTrimesh, originalSurface);

    //restoring
    restore_high_frequencies_gauss_seidel(smoothedTrimesh, originalTrimesh, mapping, bl, 400);

    smoothedSurface = SimpleEigenMesh(smoothedTrimesh);
}
