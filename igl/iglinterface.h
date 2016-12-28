#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "../common/serialize.h"
#include "../common/arrays.h"
#include "../common/point2d.h"

#include "iglmesh.h"

#include <Eigen/Core>
#include <iostream>

namespace IGLInterface {
    template <typename T>
    void generateGridAndDistanceField(Array3D<Pointd>& grid, Array3D<T> &distanceField, const SimpleIGLMesh &m, double gridUnit = 2, bool integer= true);

    IGLInterface::SimpleIGLMesh makeBox(const BoundingBox &bb, double minimumEdge = -1);

    IGLInterface::SimpleIGLMesh makeBox(const Pointd &min, const Pointd &max, double minimumEdge = -1);

    bool isABox(const SimpleIGLMesh& simpleIGLMesh);
}

inline IGLInterface::SimpleIGLMesh IGLInterface::makeBox(const Pointd &min, const Pointd &max, double minimumEdge){
    return makeBox(BoundingBox(min, max), minimumEdge);
}

#endif // FUNCTIONS_H
