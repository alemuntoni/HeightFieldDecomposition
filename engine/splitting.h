#ifndef SPLITTING_H
#define SPLITTING_H

#include "lib/grid/irregulargrid.h"
#include "heightfieldslist.h"
#include "boxlist.h"
#include "cgal/aabbtree.h"
#include "lib/graph/graph.h"


namespace Splitting {

    //Naive splitting
    bool boxesIntersect(const Box3D &b1, const Box3D &b2);

    bool isDangerousIntersection(const Box3D &b1, const Box3D &b2, const CGALInterface::AABBTree& tree, bool checkMeshes = false);

    double getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3);

    void splitBox(const Box3D &b1, Box3D& b2, Box3D& b3, double subd = -1);

    double minimumSplit(const Box3D &b1, const Box3D &b2);

    std::set<unsigned int> getTrianglesCovered(const Box3D& b, const CGALInterface::AABBTree &aabb);

    Graph getGraph(const BoxList& bl, const Dcel& d, const CGALInterface::AABBTree &tree);

    std::pair<unsigned int, unsigned int> getArcToRemove(const std::vector<std::vector<unsigned int> > &loops, const BoxList& bl);

    void chooseBestSplit(Box3D &b1, Box3D &b2, const BoxList &bl, const CGALInterface::AABBTree& tree, const std::set<unsigned int>& boxToEliminate, std::vector<std::set<unsigned int> >& trianglesCovered);

    Array2D<int> getOrdering(BoxList& bl, const Dcel &d);
}

#endif // SPLITTING_H
