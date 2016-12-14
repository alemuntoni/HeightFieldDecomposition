#ifndef SPLITTING_H
#define SPLITTING_H

#include "lib/grid/irregulargrid.h"
#include "igl/iglmesh.h"
#include "heightfieldslist.h"
#include "boxlist.h"
#include "cgal/aabbtree.h"

namespace Splitting {

    //Naive splitting
    bool boxesIntersect(const Box3D &b1, const Box3D &b2);

    bool isDangerousIntersection(const Box3D &b1, const Box3D &b2, const CGALInterface::AABBTree& tree, bool checkMeshes = false);

    double getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3);

    void splitBox(const Box3D &b1, Box3D& b2, Box3D& b3);

    double minimumSplit(const Box3D &b1, const Box3D &b2);

    Array2D<int> getOrdering(BoxList& bl, const Dcel &d);
}

#endif // SPLITTING_H
