#include "reconstruction.h"
#include "cgal/aabbtree.h"
#include "common.h"

std::map< const Dcel::Vertex*,int > Reconstruction::getMappingId(const Dcel& smoothedSurface, const HeightfieldsList& he) {
    std::map< const Dcel::Vertex*,int > mapping;
    CGALInterface::AABBTree tree(smoothedSurface);
    //int referenced = 0;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const IGLInterface::IGLMesh m = he.getHeightfield(i);
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
        const IGLInterface::IGLMesh m = he.getHeightfield(i);
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
    if (hf < 0 || ! boxList.getBox(hf).isEpsilonIntern(Pointd(vertex.x(), vertex.y(), vertex.z()), -0.5))
        return false;
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
                if (n.dot(cinolib::vec3d( 1, 0, 0)) < 0)
                    return false;
                break;
            case 1 :
                if (n.dot(cinolib::vec3d( 0, 1, 0)) < 0)
                    return false;
                break;
            case 2 :
                if (n.dot(cinolib::vec3d( 0, 0, 1)) < 0)
                    return false;
                break;
            case 3 :
                if (n.dot(cinolib::vec3d(-1, 0, 0)) < 0)
                    return false;
                break;
            case 4 :
                if (n.dot(cinolib::vec3d( 0,-1, 0)) < 0)
                    return false;
                break;
            case 5 :
                if (n.dot(cinolib::vec3d( 0, 0,-1)) < 0)
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
    for(int vid=0; vid<m.num_vertices(); ++vid)
    {
        double w    = 1.0 / double(m.vertex_valence(vid));
        cinolib::vec3d  curr = m.vertex(vid);
        cinolib::vec3d  delta(0,0,0);
        for(int nbr : m.adj_vtx2vtx(vid))
        {
            delta += w * (curr - m.vertex(nbr));
        }
        diff_coords.push_back(delta);
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
        //std::cout << "iter " << i << std::endl;

        for(int vid=0; vid<m_smooth.num_vertices(); ++vid)
        {
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

void Reconstruction::iglMeshToTrimesh(cinolib::Trimesh& m, const IGLInterface::SimpleIGLMesh& simpleIGLMesh) {
    unsigned int nVertices=simpleIGLMesh.getNumberVertices();
    unsigned int nFaces=simpleIGLMesh.getNumberFaces();

    std::vector<double> coords;
    std::vector<unsigned int> tris;

    coords.resize(nVertices*3);
    tris.resize(nFaces*3);

    for(unsigned int i=0;i<nVertices;++i) {
        unsigned int j=i*3;
        coords[j]=simpleIGLMesh.getVertex(i).x();
        coords[j+1]=simpleIGLMesh.getVertex(i).y();
        coords[j+2]=simpleIGLMesh.getVertex(i).z();

    }
    for(unsigned int i=0;i<nFaces;++i) {
        unsigned int j=i*3;
        tris[j]=simpleIGLMesh.getFace(i).x();
        tris[j+1]=simpleIGLMesh.getFace(i).y();
        tris[j+2]=simpleIGLMesh.getFace(i).z();
    }
    m = cinolib::Trimesh(coords, tris);
}

void Reconstruction::trimeshToIglMesh(IGLInterface::SimpleIGLMesh& simpleIGLMesh, const cinolib::Trimesh& m) {
    simpleIGLMesh.clear();
    simpleIGLMesh.resizeVertices(m.num_vertices());
    simpleIGLMesh.resizeFaces(m.num_triangles());
    for (int i = 0; i <m.num_vertices(); i++){
        cinolib::vec3d v = m.vertex(i);
        simpleIGLMesh.setVertex(i, v.x(), v.y(), v.z());
    }
    for (int i = 0; i < m.num_triangles(); i++){
        simpleIGLMesh.setFace(i, m.triangle_vertex_id(i,0), m.triangle_vertex_id(i,1), m.triangle_vertex_id(i,2));
    }

}

void Reconstruction::reconstruction(Dcel& smoothedSurface, const std::vector<std::pair<int, int>>& mapping, const IGLInterface::IGLMesh& originalSurface, const BoxList &bl) {
    IGLInterface::SimpleIGLMesh tmp(smoothedSurface);
    cinolib::logger.disable();
    cinolib::Trimesh smoothedTrimesh;
    cinolib::Trimesh originalTrimesh;
    iglMeshToTrimesh(smoothedTrimesh, tmp);
    iglMeshToTrimesh(originalTrimesh, originalSurface);

    //restoring
    restore_high_frequencies_gauss_seidel(smoothedTrimesh, originalTrimesh, mapping, bl, 50);

    trimeshToIglMesh(tmp, smoothedTrimesh);
    smoothedSurface = tmp;
}
