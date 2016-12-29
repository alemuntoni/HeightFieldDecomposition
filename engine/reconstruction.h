#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "heightfieldslist.h"

#include <iostream>
#include <fstream>
#include <cinolib/meshes/trimesh/trimesh.h>
#include <cinolib/scalar_field.h>

namespace Reconstruction {
    std::vector<int> getMapping(const Dcel &smoothedSurface, const HeightfieldsList &he);
    void saveMappingOnFile(const std::vector<int>& mapping, const std::string &filename);


    bool validate_move(const cinolib::Trimesh & m, const int vid, const int dir, const cinolib::vec3d & vid_new_pos);
    void differential_coordinates(const cinolib::Trimesh & m, std::vector<cinolib::vec3d> & diff_coords);
    void restore_high_frequencies_gauss_seidel(cinolib::Trimesh& m_smooth, const cinolib::Trimesh& m_detail, const std::vector<int> & hf_directions, const int n_iters);

    void iglMeshToTrimesh(cinolib::Trimesh &m, const IGLInterface::SimpleIGLMesh &simpleIGLMesh);

    void trimeshToIglMesh(IGLInterface::SimpleIGLMesh &simpleIGLMesh, const cinolib::Trimesh &m);

    void reconstruction(Dcel &smoothedSurface, const std::vector<int>& mapping, const IGLInterface::IGLMesh &originalSurface);

}

#endif // RECONSTRUCTION_H
