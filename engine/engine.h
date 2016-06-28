#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>

#include "lib/dcel/dcel.h"
#include "lib/grid/grid.h"
#include "lib/common/timer.h"
#include "energy.h"
#include "boxlist.h"
#include "common.h"

#define ORIENTATIONS 4

namespace Engine {

    Eigen::Matrix3d scaleAndRotateDcel(Dcel& d, int resolution = 50, int rot = 0);

    void generateGrid(Grid &g, const Dcel &d, double kernelDistance = 6, const Vec3& target = Vec3(), bool heightfields = false);

    Vec3 getClosestTarget(const Vec3 &n);

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g);

    void largeScaleFabrication(const Dcel &input, int resolution = 50, double kernelDistance = 6, bool heightfields = false);

}

#endif // ENGINE_H
