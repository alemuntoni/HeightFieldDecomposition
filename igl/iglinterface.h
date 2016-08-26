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

    //static std::vector< std::vector<Point2D> > dummy_holes2D;
    //std::vector<std::array<Point2D, 3> > triangulate(const std::vector<Point2D>& polygon, const std::vector<std::vector<Point2D> >& holes = dummy_holes2D, double maximumArea, double minimumAngle = 0.2);
}


#endif // FUNCTIONS_H
