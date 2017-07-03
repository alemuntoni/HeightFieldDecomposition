#ifndef SPLITTING_H
#define SPLITTING_H

#include "lib/grid/irregulargrid.h"
#include "heightfieldslist.h"
#include "boxlist.h"
#include "cgal/aabbtree.h"
#include "lib/graph/directedgraph.h"
#include <common/comparators.h>

#define SPLIT_DEBUG

namespace Splitting {

    //Naive splitting
    bool boxesIntersect(const Box3D &b1, const Box3D &b2);

    bool boxesIntersectNS(const Box3D &b1, const Box3D &b2);

    bool meshCollide(const SimpleEigenMesh &b1, const SimpleEigenMesh &b2);

    bool isDangerousIntersection(const Box3D &b1, const Box3D &b2, const CGALInterface::AABBTree& tree, bool checkMeshes = false);

    double getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3);

    void splitBox(const Box3D &b1, Box3D& b2, Box3D& b3, double subd = -1);

    double minimumSplit(const Box3D &b1, const Box3D &b2);

    std::set<unsigned int> getTrianglesCovered(const Box3D& b, const CGALInterface::AABBTree &aabb, bool completely = true);

    DirectedGraph getGraph(const BoxList& bl, const CGALInterface::AABBTree &tree);

    std::pair<unsigned int, unsigned int> getArcToRemove(const std::vector<std::vector<unsigned int> > &loops, const BoxList& bl, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs);

    void chooseBestSplit(Box3D &b1, Box3D &b2, const BoxList &bl, const CGALInterface::AABBTree& tree, const std::set<unsigned int>& boxToEliminate);

    bool checkDeleteBox(const Box3D &b, const std::set<unsigned int>& boxesToEliminate,  const BoxList &bl);

    void splitB2(const Box3D& b1, Box3D& b2, BoxList& bl, DirectedGraph& g, const Dcel& d, const CGALInterface::AABBTree& tree, std::set<unsigned int> &boxesToEliminate, std::map<unsigned int, unsigned int> &mappingNewToOld, int& numberOfSplits, int& deletedBoxes, std::set<std::pair<unsigned int, unsigned int>, cmpUnorderedStdPair<unsigned int> >& impossibleArcs);

    Array2D<int> getOrdering(BoxList& bl, const Dcel &d, std::map<unsigned int, unsigned int>& mappingNewToOld, std::list<unsigned int>& priorityBoxes, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs);
}

#endif // SPLITTING_H
