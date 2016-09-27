#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "lib/grid/irregulargrid.h"

namespace Reconstruction {
    Pointi getGrowthStep(const Vec3& target);

    void getAdjacents(Pointi &s1, Pointi &s2, Pointi &s3, Pointi &s4, const Pointi &base, const Vec3 &target);

    Pointi getBase(const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void growTarget(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void recursiveGrowth(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBaseBox, const Vec3 &target);

    void growPiece(IrregularGrid& g, const Pointi &startingBox, const Vec3 &target);

}

#endif // RECONSTRUCTION_H
