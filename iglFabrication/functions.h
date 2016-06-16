#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "serialize.h"
#include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <Eigen/Core>
#include <iostream>


bool generateGridAndDistanceField(const std::string &s);

#endif // FUNCTIONS_H
