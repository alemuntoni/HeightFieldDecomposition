#include "reconstruction.h"
#include "cgal/aabbtree.h"
#include "common.h"

std::vector<int> Reconstruction::getMapping(const Dcel& smoothedSurface, const HeightfieldsList& he) {
    std::vector<Vec3> tmpmapping(smoothedSurface.getNumberVertices(), Vec3());
    CGALInterface::AABBTree tree(smoothedSurface);
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const IGLInterface::IGLMesh m = he.getHeightfield(i);
        for (unsigned int j = 0; j < m.getNumberVertices(); j++){
            const Dcel::Vertex* v = tree.getNearestDcelVertex(m.getVertex(j));
            assert(v != nullptr);
            if (v->getCoordinate().dist(m.getVertex(j)) == 0){
                assert(tmpmapping[v->getId()] == Vec3());
                tmpmapping[v->getId()] = he.getTarget(i);
            }
        }
    }
    std::vector<int> mapping;
    mapping.reserve(tmpmapping.size());
    for (Vec3 target : tmpmapping){
        assert(target != Vec3());
        for (int i  = 0; i < 3; i++) {
            if (target == XYZ[i] || target == XYZ[i+3]) // it happens just one time for every target
                mapping.push_back(i);
        }
    }

    return mapping;
}

void Reconstruction::saveMappingOnFile(const std::vector<int>& mapping, const std::string& filename) {
    std::ofstream myfile;
    myfile.open (filename);
    myfile << mapping.size() << "\n";
    for (int target : mapping){
        myfile << target << "\n";
    }
    myfile.close();
}

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

bool Reconstruction::validate_move(const cinolib::Trimesh & m, const int vid, const int dir, const cinolib::vec3d & vid_new_pos)
{
    std::vector<cinolib::vec3d> one_ring_tris;
    for(int tid : m.adj_vtx2tri(vid))
    {
        for(int offset=0; offset<3; ++offset)
        {
            cinolib::vec3d v = m.triangle_vertex(tid, offset);
            v[dir] = 0.0; // project along the HF direction
            one_ring_tris.push_back(v);
        }
    }

    cinolib::vec3d query = vid_new_pos;
    query[dir] = 0.0; // project along the HF direction

    // check whether the projection falls within the one ring
    for(int i=0; i<(int)one_ring_tris.size()/3;++i)
    {
        bool in = triangle_point_is_inside(one_ring_tris.at(3*i+0),
                                           one_ring_tris.at(3*i+1),
                                           one_ring_tris.at(3*i+2),
                                           query);
        if (in) return true;
    }
    return false;
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
                                           const std::vector<int> & hf_directions,
                                           const int                n_iters)
{
    std::vector<cinolib::vec3d> diff_coords;
    differential_coordinates(m_detail, diff_coords);

    for(int i=0; i<n_iters; ++i)
    {
        std::cout << "iter " << i << std::endl;

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
            while(!validate_move(m_smooth, vid, hf_directions.at(vid), new_pos) && ++count<5)
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

void Reconstruction::reconstruction(Dcel& smoothedSurface, const std::vector<int>& mapping, const IGLInterface::IGLMesh& originalSurface) {
    IGLInterface::SimpleIGLMesh tmp(smoothedSurface);
    cinolib::Trimesh smoothedTrimesh;
    cinolib::Trimesh originalTrimesh;
    iglMeshToTrimesh(smoothedTrimesh, tmp);
    iglMeshToTrimesh(originalTrimesh, originalSurface);

    //restoring
    restore_high_frequencies_gauss_seidel(smoothedTrimesh, originalTrimesh, mapping, 50);

    trimeshToIglMesh(tmp, smoothedTrimesh);
    smoothedSurface = tmp;
}
