#ifndef PACKING_H
#define PACKING_H

#include "heightfieldslist.h"
#include "common/utils.h"

namespace Packing {

    void rotateAllPieces(HeightfieldsList &he);

    int getMaximum(const HeightfieldsList &he, const BoundingBox &block, double &factor);

    void scaleAll(HeightfieldsList &he, double factor);

    std::vector<std::vector<std::pair<int, Pointd> > > pack(const HeightfieldsList &he, const BoundingBox& packSize, int distance = -1);

    std::vector< std::vector<EigenMesh> > getPacks(std::vector<std::vector<std::pair<int, Pointd> > > &packing, const HeightfieldsList &he);
}

#endif // PACKING_H
