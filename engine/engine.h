#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>

#include "lib/dcel/dcel.h"
#include "lib/grid/grid.h"
#include "lib/common/timer.h"
#include "energy.h"
#include "boxlist.h"
#include "common.h"

namespace Engine {

    void generateGrid(Grid &g, Dcel &d, int resolution, const Vec3& target, double kernelDistance = 6, bool heightfields = false);

    Vec3 getClosestTarget(const Vec3 &n);

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g);

}

#endif // ENGINE_H
