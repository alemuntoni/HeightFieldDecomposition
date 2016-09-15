#ifndef IRREGULARGRID_H
#define IRREGULARGRID_H

#include "common/arrays.h"
#include "common/point.h"

class IrregularGrid {
    public:
        IrregularGrid();
        IrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ);
        void addPoint(unsigned int i, unsigned int j, unsigned int k, const Pointd& p);
        unsigned int getIndexI(double x) const;
        unsigned int getIndexJ(double y) const;
        unsigned int getIndexK(double z) const;
        void reset(unsigned int resX, unsigned int resY, unsigned int resZ);
        void setTarget(unsigned int i, unsigned int j, unsigned int k, const Pointd& target);
        Pointd getPoint(unsigned int i, unsigned int j, unsigned int k);
        int getNumberTargets(unsigned int i, unsigned int j, unsigned int k);
        unsigned int getResolutionX();
        unsigned int getResolutionY();
        unsigned int getResolutionZ();

    protected:
        unsigned int resX, resY, resZ;
        Array3D<Pointd> points;
        std::map<double, unsigned int> mapX, mapY, mapZ;
        Array3D<std::set<Pointd> > targets;
};

inline void IrregularGrid::addPoint(unsigned int i, unsigned int j, unsigned int k, const Pointd& p) {
    assert(i < resX);
    assert(j < resY);
    assert(k < resZ);
    points(i,j,k) = p;
    mapX[p.x()] = i;
    mapX[p.y()] = j;
    mapX[p.z()] = k;
}

inline unsigned int IrregularGrid::getIndexI(double x) const {
    assert(mapX.find(x) != mapX.end());
    return mapX.at(x);
}

inline unsigned int IrregularGrid::getIndexJ(double y) const{
    assert(mapY.find(y) != mapY.end());
    return mapY.at(y);
}

inline unsigned int IrregularGrid::getIndexK(double z) const{
    assert(mapZ.find(z) != mapZ.end());
    return mapZ.at(z);
}

inline void IrregularGrid::reset(unsigned int resX, unsigned int resY, unsigned int resZ) {
    mapX.clear();
    mapY.clear();
    mapZ.clear();
    points.resize(resX, resY, resZ);
    targets.resize(resX-1, resY-1, resZ-1);
    this->resX = resX;
    this->resY = resY;
    this->resZ = resZ;
}

inline void IrregularGrid::setTarget(unsigned int i, unsigned int j, unsigned int k, const Pointd& target) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    targets(i,j,k).insert(target);
}

inline Pointd IrregularGrid::getPoint(unsigned int i, unsigned int j, unsigned int k) {
    assert(i < resX);
    assert(j < resY);
    assert(k < resZ);
    return points(i,j,k);
}

inline int IrregularGrid::getNumberTargets(unsigned int i, unsigned int j, unsigned int k) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    return targets(i,j,k).size();
}

inline unsigned int IrregularGrid::getResolutionX() {
    return resX;
}

inline unsigned int IrregularGrid::getResolutionY() {
    return resY;
}

inline unsigned int IrregularGrid::getResolutionZ() {
    return resZ;
}



#endif // IRREGULARGRID_H
