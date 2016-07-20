#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "../common/serialize.h"
#include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>

#include <igl/decimate.h>

#include "iglmesh.h"

#include <Eigen/Core>
#include <iostream>

namespace IGLInterface {
    bool generateGridAndDistanceField(const std::string &s);
}


#endif // FUNCTIONS_H
