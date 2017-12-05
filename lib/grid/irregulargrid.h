#ifndef IRREGULARGRID_H
#define IRREGULARGRID_H

#include "cg3/data_structures/arrays/arrays.h"
#include "cg3/geometry/point.h"

class IrregularGrid {
    public:
        IrregularGrid();
        IrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ);
        void addPoint(unsigned int i, unsigned int j, unsigned int k, const cg3::Pointd& p);
        unsigned int getIndexI(double x) const;
        unsigned int getIndexJ(double y) const;
        unsigned int getIndexK(double z) const;
        void reset(unsigned int resX, unsigned int resY, unsigned int resZ);
        void addPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3& target);
        cg3::Pointd getPoint(unsigned int i, unsigned int j, unsigned int k) const;
        int getNumberPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const;
        std::vector<cg3::Vec3> getPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const;
        double getVolumeOfBox(unsigned int i, unsigned int j, unsigned int k) const;
        unsigned int getResolutionX() const;
        unsigned int getResolutionY() const;
        unsigned int getResolutionZ() const;
        bool boxHasPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3& target) const;
        bool isDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k) const;
        void setDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3& definitveTarget);
        int& flag(unsigned int i, unsigned int j, unsigned int k);
        int getFlag(unsigned int i, unsigned int j, unsigned int k) const;

    protected:
        unsigned int resX, resY, resZ;
        cg3::Array3D<cg3::Pointd> points;
        std::map<double, unsigned int> mapX, mapY, mapZ;
        cg3::Array3D<std::set<cg3::Vec3> > possibleTargets;
        cg3::Array3D<cg3::Vec3> definitiveTargets;
        cg3::Array3D<int> flags;
};

inline IrregularGrid::IrregularGrid(){
}

inline IrregularGrid::IrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ): resX(resX), resY(resY), resZ(resZ) {
    mapX.clear();
    mapY.clear();
    mapZ.clear();
    points.resize(resX, resY, resZ);
    possibleTargets.resize(resX-1, resY-1, resZ-1);
    definitiveTargets.resize(resX-1, resY-1, resZ-1);
    definitiveTargets.fill(cg3::Vec3());
    flags.resize(resX-1, resY-1, resZ-1);
}

inline void IrregularGrid::addPoint(unsigned int i, unsigned int j, unsigned int k, const cg3::Pointd& p) {
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
    definitiveTargets.resize(resX-1, resY-1, resZ-1);
    flags.resize(resX-1, resY-1, resZ-1);
    this->resX = resX;
    this->resY = resY;
    this->resZ = resZ;
}

inline void IrregularGrid::addPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3& target) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    possibleTargets(i,j,k).insert(target);
}

inline cg3::Pointd IrregularGrid::getPoint(unsigned int i, unsigned int j, unsigned int k) const{
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

inline std::vector<cg3::Vec3> IrregularGrid::getPossibleTargets(unsigned int i, unsigned int j, unsigned int k) const {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    std::vector<cg3::Vec3> tmp;
    std::set<cg3::Vec3> tmp2 = possibleTargets(i,j,k);
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

inline bool IrregularGrid::boxHasPossibleTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3& target) const {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    const std::set<cg3::Vec3>& pt =  possibleTargets(i,j,k);

    return (pt.find(target) != pt.end());
}

inline bool IrregularGrid::isDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k) const{
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);

    return definitiveTargets(i,j,k) != cg3::Vec3();
}

inline void IrregularGrid::setDefinitiveTarget(unsigned int i, unsigned int j, unsigned int k, const cg3::Vec3 &definitveTarget) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);

    //possibleTargets(i,j,k).clear();
    definitiveTargets(i,j,k) = definitveTarget;
}

inline int& IrregularGrid::flag(unsigned int i, unsigned int j, unsigned int k) {
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    return flags(i,j,k);
}

inline int IrregularGrid::getFlag(unsigned int i, unsigned int j, unsigned int k) const{
    assert(i < resX-1);
    assert(j < resY-1);
    assert(k < resZ-1);
    return flags(i,j,k);
}



#endif // IRREGULARGRID_H
