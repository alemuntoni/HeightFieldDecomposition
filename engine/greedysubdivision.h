#ifndef GREEDYSUBDIVISION_H
#define GREEDYSUBDIVISION_H

#include <lib/graph/bipartitegraph.h>
#include <engine/boxlist.h>
#include <dcel/dcel.h>
#include <engine/box.h>

namespace GreedySubdivision {
    BipartiteGraph<const Dcel::Face*, Box3D> getBipartiteGraph(const BoxList& bl, const Dcel &d);

    std::set<Box3D> getMandatoryBoxesToCompute(const BipartiteGraph<const Dcel::Face*, Box3D>& bigraph);
}

#endif // GREEDYSUBDIVISION_H
