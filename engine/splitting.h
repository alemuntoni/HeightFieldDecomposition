#ifndef SPLITTING_H
#define SPLITTING_H

#include "heightfieldslist.h"
#include "boxlist.h"
#include "cg3/cgal/aabbtree.h"
#include "lib/graph/directedgraph.h"
#include <cg3/utilities/comparators.h>

#define SPLIT_DEBUG

namespace Splitting {

    //Naive splitting
    bool boxesIntersect(const Box3D &b1, const Box3D &b2);

    bool boxesIntersectNS(const Box3D &b1, const Box3D &b2);

    bool meshCollide(const cg3::SimpleEigenMesh &b1, const cg3::SimpleEigenMesh &b2);

    bool isDangerousIntersection(const Box3D &b1, const Box3D &b2, const cg3::cgal::AABBTree& tree, bool checkMeshes = false);

    bool getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3);

    void splitBox(const Box3D &b1, Box3D& b2, Box3D& b3, double subd = -1);

    int getMinTrianglesCoveredIfBoxesSplitted(const Box3D &b1, const Box3D &b2, const cg3::cgal::AABBTree& tree);

    std::set<unsigned int> getTrianglesCovered(const Box3D& b, const cg3::cgal::AABBTree &aabb, bool completely = true);

    DirectedGraph getGraph(const BoxList& bl, const cg3::cgal::AABBTree &tree);

    std::pair<unsigned int, unsigned int> getArcToRemove(const std::vector<std::vector<unsigned int> > &loops, const BoxList& bl, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs, const cg3::cgal::AABBTree& tree);

    void chooseBestSplit(Box3D &b1, Box3D &b2, const BoxList &bl, const cg3::cgal::AABBTree& tree, const std::set<unsigned int>& boxToEliminate);

    bool checkDeleteBox(const Box3D &b, const std::set<unsigned int>& boxesToEliminate,  const BoxList &bl);

    void splitB2(const Box3D& b1, Box3D& b2, BoxList& bl, DirectedGraph& g, const cg3::cgal::AABBTree& tree, std::set<unsigned int> &boxesToEliminate, std::map<unsigned int, unsigned int> &mappingNewToOld, int& numberOfSplits, int& deletedBoxes, std::set<std::pair<unsigned int, unsigned int>, cg3::cmpUnorderedStdPair<unsigned int> >& impossibleArcs);

    cg3::Array2D<int> getOrdering(BoxList& bl, const cg3::Dcel &d, std::map<unsigned int, unsigned int>& mappingNewToOld, std::list<unsigned int>& priorityBoxes, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs);
}

#endif // SPLITTING_H
