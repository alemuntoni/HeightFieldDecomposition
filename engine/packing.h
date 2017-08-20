#ifndef PACKING_H
#define PACKING_H

#include "heightfieldslist.h"
#include "cg3/utilities/utils.h"

namespace Packing {

    void rotateAllPieces(HeightfieldsList &he);

    int getMaximum(const HeightfieldsList &he, const cg3::BoundingBox &block, double &factor);

    void scaleAll(HeightfieldsList &he, double factor);

    std::vector<std::vector<std::pair<int, cg3::Pointd> > > pack(const HeightfieldsList &he, const cg3::BoundingBox& packSize, int distance = -1);

    std::vector< std::vector<cg3::EigenMesh> > getPacks(std::vector<std::vector<std::pair<int, cg3::Pointd> > > &packing, const HeightfieldsList &he);
}

#endif // PACKING_H
