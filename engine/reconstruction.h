#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "heightfieldslist.h"
#include "boxlist.h"

#include <iostream>
#include <fstream>
#include <cinolib/meshes/trimesh/trimesh.h>
#include <cinolib/scalar_field.h>


namespace Reconstruction {
    std::vector<std::pair<int, int> > getMapping(const Dcel &smoothedSurface, const HeightfieldsList &he);


    bool validate_move(const cinolib::Trimesh & m, const int vid, const int hf, const int dir, const cinolib::vec3d & vid_new_pos, const BoxList& boxList);
    void differential_coordinates(const cinolib::Trimesh & m, std::vector<cinolib::vec3d> & diff_coords);
    void restore_high_frequencies_gauss_seidel(cinolib::Trimesh& m_smooth, const cinolib::Trimesh& m_detail, const std::vector<std::pair<int, int> >& hf_directions, const BoxList& boxList, const int n_iters);

    void iglMeshToTrimesh(cinolib::Trimesh &m, const IGLInterface::SimpleIGLMesh &simpleIGLMesh);

    void trimeshToIglMesh(IGLInterface::SimpleIGLMesh &simpleIGLMesh, const cinolib::Trimesh &m);

    void reconstruction(Dcel &smoothedSurface, const std::vector<std::pair<int, int> >& mapping, const IGLInterface::IGLMesh &originalSurface, const BoxList& bl);

}

#endif // RECONSTRUCTION_H
