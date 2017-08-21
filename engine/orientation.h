#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <cg3/utilities/utils.h>
#include <cinolib/meshes/trimesh/trimesh.h>
#include <cinolib/sphere_coverage.h>
#include <cg3/cinolib/mesh_conversions.h>
#include <cg3/meshes/dcel/dcel.h>

namespace Orientation {
    void define_rotation(const cinolib::vec3d & Z_axis, cinolib::vec3d & rot_axis, double & angle);

    Eigen::Matrix3d optimalOrientation(const cg3::Dcel& inputMesh, unsigned int n_dirs = 10000);
}

#endif // ORIENTATION_H
