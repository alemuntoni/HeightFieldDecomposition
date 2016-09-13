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

    protected:
        unsigned int resX, resY, resZ;
        Array3D<Pointd> points;
        std::map<double, unsigned int> mapX, mapY, mapZ;
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
    this->resX = resX;
    this->resY = resY;
    this->resZ = resZ;
}

#endif // IRREGULARGRID_H
