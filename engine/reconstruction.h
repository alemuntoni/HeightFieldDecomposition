#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "lib/grid/irregulargrid.h"
#include "igl/iglmesh.h"
#include "heightfieldslist.h"
#include "boxlist.h"

namespace Reconstruction {
    //Creation Irregular Grid
    void compactSet(std::set<double> &set, double epsilon = 1e-6);

    void createIrregularGrid(IrregularGrid &grid, const BoxList& solutions, const Dcel& d, double epsilon = 0.01);

    //Naive Reconstruction
    Pointi getGrowthStep(const Vec3& target);

    void getAdjacents(Pointi &s1, Pointi &s2, Pointi &s3, Pointi &s4, const Pointi &base, const Vec3 &target);

    bool isBounded(const Pointi &box, const IrregularGrid& g);

    Pointi getBase(const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void growTarget(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBox, const Vec3 &target);

    void recursiveGrowth(std::set<Pointi>& conqueredBoxes, const IrregularGrid &g, const Pointi &startingBaseBox, const Vec3 &target);

    std::set<Pointi> growPiece(const IrregularGrid& g, const Pointi &startingBox, const Vec3 &target);

    void setDefinitivePiece(IrregularGrid& g, const std::set<Pointi>& piece, const Vec3 &target);

    //BruteForce
    std::set<Pointi> findConnectedComponent(IrregularGrid &g, const Pointi& startingBox, const Vec3& target);


    //Getting pieces from Irregular Grid
    IGLInterface::IGLMesh getSurfaceOfPiece(const std::set<Pointi> &boxes, const IrregularGrid& g);

    std::vector<IGLInterface::IGLMesh> getPieces(IrregularGrid &g, std::vector<Vec3>& targets);

    //Booleans
    void booleanOperations(HeightfieldsList &heightfields, IGLInterface::SimpleIGLMesh &baseComplex, HeightfieldsList &polycubes);
}

#endif // RECONSTRUCTION_H
