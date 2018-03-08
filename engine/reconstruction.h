#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "heightfieldslist.h"
#include "boxlist.h"

#include <iostream>
#include <fstream>
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-variable" //Doesn't work on gcc < 6.0
#include <cinolib/meshes/trimesh.h>
#pragma GCC diagnostic pop
#endif //__GNUC__
#include <cinolib/scalar_field.h>


namespace Reconstruction {
    std::map<const cg3::Dcel::Vertex*, int > getMappingId(const cg3::Dcel &smoothedSurface, const HeightfieldsList &he);

    std::vector<std::pair<int, int> > getMapping(const cg3::Dcel &smoothedSurface, const HeightfieldsList &he);


    bool validate_move(const cinolib::Trimesh<> & m, const int vid, const int hf, const int dir, const cinolib::vec3d & vid_new_pos, const BoxList& boxList, bool internToHF);
    void differential_coordinates(const cinolib::Trimesh<> & m, std::vector<cinolib::vec3d> & diff_coords);
    void restore_high_frequencies_gauss_seidel(cinolib::Trimesh<>& m_smooth, const cinolib::Trimesh<>& m_detail, const std::vector<std::pair<int, int> >& hf_directions, const BoxList& boxList, const int n_iters, bool internToHF);

    cg3::Dcel taubinSmoothing(const cg3::SimpleEigenMesh &m, int n_iters = 10, double lambda = 0.89, const double mu = -0.9);

    cg3::Dcel taubinSmoothing(const cg3::Dcel &d, int n_iters = 10, double lambda = 0.89, const double mu = -0.9);

    void reconstruction(cg3::Dcel &smoothedSurface, const std::vector<std::pair<int, int> >& mapping, const cg3::EigenMesh& originalSurface, const BoxList& bl, bool internToHF = false);
}

#endif // RECONSTRUCTION_H
