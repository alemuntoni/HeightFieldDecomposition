#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "lib/grid/irregulargrid.h"
#include "igl/iglmesh.h"
#include "heightfieldslist.h"

namespace Reconstruction {
    Pointi getGrowthStep(const Vec3& target);

    void getAdjacents(Pointi &s1, Pointi &s2, Pointi &s3, Pointi &s4, const Pointi &base, const Vec3 &target);

    bool isBounded(const Pointi &box, const IrregularGrid& g);

    Pointi getBase(const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void growTarget(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void recursiveGrowth(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBaseBox, const Vec3 &target);

    std::set<Pointi> growPiece(const IrregularGrid& g, const Pointi &startingBox, const Vec3 &target);

    void setDefinitivePiece(IrregularGrid& g, const std::set<Pointi>& piece, const Vec3 &target);

    IGLInterface::IGLMesh getSurfaceOfPiece(const std::set<Pointi> &boxes, const IrregularGrid& g);

    std::vector<IGLInterface::IGLMesh> getPieces(IrregularGrid &g, std::vector<Vec3>& targets);

    void booleanOperations(HeightfieldsList &heightfields, IGLInterface::SimpleIGLMesh &baseComplex, HeightfieldsList &polycubes);
}

#endif // RECONSTRUCTION_H
