#include "reconstruction.h"

#include "common.h"

Pointi Reconstruction::getGrowthStep(const Vec3& target) {
    Pointi step;
    if (target == XYZ[0])
        step.setX(1);
    else if (target == XYZ[1])
        step.setY(1);
    else if (target == XYZ[2])
        step.setZ(1);
    else if (target == XYZ[3])
        step.setX(-1);
    else if (target == XYZ[4])
        step.setY(-1);
    else if (target == XYZ[5])
        step.setZ(-1);
    else assert(0);
    return step;
}

void Reconstruction::getAdjacents(Pointi& s1, Pointi& s2, Pointi& s3, Pointi& s4, const Pointi& base, const Vec3& target) {
    if (target == XYZ[0] || target == XYZ[3]){
        s1 = base + Pointi(0,1,0);
        s2 = base + Pointi(0,0,1);
        s3 = base + Pointi(0,-1,0);
        s4 = base + Pointi(0,0,-1);
    }
    else if (target == XYZ[1] || target == XYZ[4]){
        s1 = base + Pointi(1,0,0);
        s2 = base + Pointi(0,0,1);
        s3 = base + Pointi(-1,0,0);
        s4 = base + Pointi(0,0,-1);
    }
    else if (target == XYZ[2] || target == XYZ[5]){
        s1 = base + Pointi(0,1,0);
        s2 = base + Pointi(1,0,0);
        s3 = base + Pointi(0,-1,0);
        s4 = base + Pointi(-1,0,0);
    }
    else assert(0);
}

Pointi Reconstruction::getBase(const IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    assert(g.boxHasPossibleTarget(startingBox.x(), startingBox.y(), startingBox.z(), target) && !(g.isDefinitiveTarget(startingBox.x(), startingBox.y(), startingBox.z())));
    Pointi step = getGrowthStep(target);

    Pointi base = startingBox;
    do {
        base -= step;
    } while (g.boxHasPossibleTarget(base.x(), base.y(), base.z(), target) && !(g.isDefinitiveTarget(base.x(), base.y(), base.z())));
    base += step;

    return base;
}

void Reconstruction::growTarget(std::set<Pointi>& conqueredBoxes, const IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    assert(g.boxHasPossibleTarget(startingBox.x(), startingBox.y(), startingBox.z(), target) && !(g.isDefinitiveTarget(startingBox.x(), startingBox.y(), startingBox.z())));
    Pointi step = getGrowthStep(target);
    Pointi base = startingBox;
    do {
        conqueredBoxes.insert(base);
        base += step;
    } while (g.boxHasPossibleTarget(base.x(), base.y(), base.z(), target) && !(g.isDefinitiveTarget(base.x(), base.y(), base.z())));
}

void Reconstruction::recursiveGrowth(std::set<Pointi>& conqueredBoxes, const IrregularGrid& g, const Pointi& startingBaseBox, const Vec3& target) {
    growTarget(conqueredBoxes, g, startingBaseBox, target);
    Pointi s1, s2, s3, s4;
    getAdjacents(s1, s2, s3, s4, startingBaseBox, target);
    if (g.boxHasPossibleTarget(s1.x(), s1.y(), s1.z(), target) && conqueredBoxes.find(s1) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s1.x(), s1.y(), s1.z())))
        recursiveGrowth(conqueredBoxes, g, s1, target);
    if (g.boxHasPossibleTarget(s2.x(), s2.y(), s2.z(), target) && conqueredBoxes.find(s2) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s2.x(), s2.y(), s2.z())))
        recursiveGrowth(conqueredBoxes, g, s2, target);
    if (g.boxHasPossibleTarget(s3.x(), s3.y(), s3.z(), target) && conqueredBoxes.find(s3) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s3.x(), s3.y(), s3.z())))
        recursiveGrowth(conqueredBoxes, g, s3, target);
    if (g.boxHasPossibleTarget(s4.x(), s4.y(), s4.z(), target) && conqueredBoxes.find(s4) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s4.x(), s4.y(), s4.z())))
        recursiveGrowth(conqueredBoxes, g, s4, target);
}

void Reconstruction::growPiece(IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    Pointi base = getBase(g, startingBox, target);
    std::set<Pointi> conqueredBoxes;
    recursiveGrowth(conqueredBoxes, g, base, target);
    for (Pointi box : conqueredBoxes){
        //setting definitive targets in g
        g.setDefinitiveTarget(box.x(), box.y(), box.z(), target);
    }
}
