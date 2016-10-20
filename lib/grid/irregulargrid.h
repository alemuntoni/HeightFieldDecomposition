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
        void addPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3& target);
        Pointd getPoint(unsigned int i, unsigned int j, unsigned int k) const;
        int getNumberPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const;
        std::vector<Vec3> getPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const;
        double getVolumeOfBox(unsigned int i, unsigned int j, unsigned int k) const;
        unsigned int getResolutionX() const;
        unsigned int getResolutionY() const;
        unsigned int getResolutionZ() const;
        bool boxHasPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3& target) const;
        bool isDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k) const;
        void setDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3& definitveTarget);

    protected:
        unsigned int resX, resY, resZ;
        Array3D<Pointd> points;
        std::map<double, unsigned int> mapX, mapY, mapZ;
        Array3D<std::set<Vec3> > possibleTargets;
        Array3D<Vec3> definitiveTargets;
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
    possibleTargets.resize(resX-1, resY-1, resZ-1);
    definitiveTargets.resize(resX-1, resY-1, resZ-1, Vec3());
    this->resX = resX;
    this->resY = resY;
    this->resZ = resZ;
}

inline void IrregularGrid::addPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3& target) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    possibleTargets(i,j,k).insert(target);
}

inline Pointd IrregularGrid::getPoint(unsigned int i, unsigned int j, unsigned int k) const{
    assert(i < resX);
    assert(j < resY);
    assert(k < resZ);
    return points(i,j,k);
}

inline int IrregularGrid::getNumberPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const{
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    return possibleTargets(i,j,k).size();
}

inline std::vector<Vec3> IrregularGrid::getPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    std::vector<Vec3> tmp;
    std::set<Vec3> tmp2 = possibleTargets(i,j,k);
    std::copy(tmp2.begin(), tmp2.end(), std::back_inserter(tmp));
    return tmp;
}

inline double IrregularGrid::getVolumeOfBox(unsigned int i, unsigned int j, unsigned int k) const {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    return (points(i+1,j,k).x() - points(i,j,k).x()) * (points(i,j+1,k).y() - points(i,j,k).y()) * (points(i,j,k+1).z() - points(i,j,k).z());
}

inline unsigned int IrregularGrid::getResolutionX() const{
    return resX;
}

inline unsigned int IrregularGrid::getResolutionY() const{
    return resY;
}

inline unsigned int IrregularGrid::getResolutionZ() const{
    return resZ;
}

inline bool IrregularGrid::boxHasPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3& target) const {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    const std::set<Vec3>& pt =  possibleTargets(i,j,k);

    return (pt.find(target) != pt.end());
}

inline bool IrregularGrid::isDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k) const{
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);

    return possibleTargets(i,j,k).size() == 0 || definitiveTargets(i,j,k) != Vec3();
}

inline void IrregularGrid::setDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k, const Vec3 &definitveTarget) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);

    possibleTargets(i,j,k).clear();
    definitiveTargets(i,j,k) = definitveTarget;
}



#endif // IRREGULARGRID_H
