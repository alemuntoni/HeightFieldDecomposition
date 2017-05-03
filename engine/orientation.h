#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <common/utils.h>
#include <cinolib/meshes/trimesh/trimesh.h>
#include <cinolib/sphere_coverage.h>
#include <cinolib_interface/mesh_conversions.h>
#include <dcel/dcel.h>

namespace Orientation {
    void define_rotation(const cinolib::vec3d & Z_axis, cinolib::vec3d & rot_axis, double & angle);

    Eigen::Matrix3d optimalOrientation(const Dcel& inputMesh, unsigned int n_dirs = 1000);
}

#endif // ORIENTATION_H
