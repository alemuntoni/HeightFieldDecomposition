#include "grid.h"

Grid::Grid() {
}

Grid::Grid(const Pointi& resolution, const Array3D<Pointd>& gridCoordinates, const Array3D<gridreal>& signedDistances, const Pointd& gMin, const Pointd& gMax) :
    signedDistances(signedDistances), target(0,0,0) {
    unit = gridCoordinates(1,0,0).x() - gridCoordinates(0,0,0).x();
    bb.setMin(gMin);
    bb.setMax(gMax);
    resX = resolution.x();
    resY = resolution.y();
    resZ = resolution.z();
    weights = Array3D<gridreal>(resX,resY,resZ, BORDER_PAY);
    for (unsigned int i = 2; i < resX-2; i++){
        for (unsigned int j = 2; j < resY-2; j++){
            for (unsigned int k = 2; k < resZ-2; ++k){
                weights(i,j,k) = STD_PAY;
            }
        }
    }
    coeffs = Array4D<gridreal>((resX-1),(resY-1),(resZ-1),64, 0);
}

/**
 * @brief Grid::calculateWeights
 *
 * Calcola i pesi per gli heightfieltds rispetto alla normale target
 * @param d
 */
void Grid::calculateBorderWeights(const Dcel& d, bool tolerance, std::set<const Dcel::Face*>& savedFaces) {
    CGALInterface::AABBTree aabb(d);
    double unit = getUnit();
    std::vector<Pointi> flipped;
    std::vector<Pointi> notFlipped;
    for (unsigned int i = 0; i < resX-1; i++){
        for (unsigned int j = 0; j < resY-1; j++){
            for (unsigned int k = 0; k < resZ-1; k++){
                Pointd bbmin = getPoint(i,j,k);
                Pointd bbmax(bbmin.x()+unit, bbmin.y()+unit, bbmin.z()+unit);
                BoundingBox bb(bbmin, bbmax);
                std::list<const Dcel::Face*> l;
                aabb.getIntersectedDcelFaces(l, bb);
                if (l.size() != 0){
                    bool b = true;
                    for (std::list<const Dcel::Face*>::iterator it = l.begin(); it != l.end(); ++it) {
                        const Dcel::Face* f = *it;
                        Pointi p(i,j,k);
                        if (f->getNormal().dot(target) < FLIP_ANGLE && f->getFlag() != 1 && savedFaces.find(f) == savedFaces.end()){
                             flipped.push_back(p);
                             b = false;
                        }
                        ///
                        else {
                            notFlipped.push_back(p);
                        }
                        ///
                    }
                    if (b){
                        setWeightOnCube(i,j,k, MIN_PAY);
                    }
                }
            }
        }
    }
    if (tolerance){
        for (unsigned int i = 0; i < flipped.size(); ++i){
            setWeightOnCube(flipped[i].x(),flipped[i].y(),flipped[i].z(), MAX_PAY);
        }
        ///
        for (unsigned int i = 0; i < notFlipped.size(); ++i){
            setWeightOnCube(notFlipped[i].x(),notFlipped[i].y(),notFlipped[i].z(), MIN_PAY);
        }
        ///
    }
    else {
        ///
        for (unsigned int i = 0; i < notFlipped.size(); ++i){
            setWeightOnCube(notFlipped[i].x(),notFlipped[i].y(),notFlipped[i].z(), MIN_PAY);
        }
        ///
        for (unsigned int i = 0; i < flipped.size(); ++i){
            setWeightOnCube(flipped[i].x(),flipped[i].y(),flipped[i].z(), MAX_PAY);
        }
    }
}

/**
 * @brief Grid::freezeKernel
 *
 * Assegna massimo peso ai punti nel kernel
 * @param value should be a number between 0 and 1
 */
void Grid::calculateWeightsAndFreezeKernel(const Dcel& d, double value, bool tolerance, std::set<const Dcel::Face*>& savedFaces) {
    assert(value >= 0 && value <= 1);
    // grid border and rest
    weights.setConstant(BORDER_PAY);
    for (unsigned int i = 2; i < resX-2; i++){
        for (unsigned int j = 2; j < resY-2; j++){
            for (unsigned int k = 2; k < resZ-2; ++k){
                weights(i,j,k) = STD_PAY;
            }
        }
    }

    //mesh border
    calculateBorderWeights(d, tolerance, savedFaces);

    double minValue = signedDistances.getMin();
    value = 1 - value;
    value *= minValue;
    value = std::abs(value);

    //kernel
    for (unsigned int i = 0; i < getResX(); ++i){
        for (unsigned int j = 0; j < getResY(); ++j){
            for (unsigned int k = 0; k < getResZ(); ++k){
                if (getSignedDistance(i,j,k) < -value){
                    weights(i,j,k) = MAX_PAY;
                }
            }
        }
    }
    TricubicInterpolator::getCoefficients(coeffs, weights);
}

void Grid::calculateFullBoxValues(double (*integralTricubicInterpolation)(const gridreal *&, double, double, double, double, double, double)) {
    fullBoxValues = Array3D<gridreal>(getResX()-1, getResY()-1, getResZ()-1);
    #pragma omp parallel for
    for (unsigned int i = 0; i < fullBoxValues.getSizeX(); ++i){
        for (unsigned int j = 0; j < fullBoxValues.getSizeY(); ++j){
            for (unsigned int k = 0; k < fullBoxValues.getSizeZ(); ++k){
                const gridreal * coeffs;
                getCoefficients(coeffs, i, j, k);
                fullBoxValues(i,j,k) = integralTricubicInterpolation(coeffs, 0,0,0,1,1,1);
            }
        }
    }
}

double Grid::getValue(const Pointd& p) const {
    if (! bb.isStrictlyIntern(p)) return BORDER_PAY;
    unsigned int xi = getIndexOfCoordinateX(p.x()), yi = getIndexOfCoordinateY(p.y()), zi = getIndexOfCoordinateZ(p.z());
    Pointd n = getPoint(xi, yi, zi);
    if (n == p)
        return weights(xi,yi,zi);
    else{
        n = (p - n) / getUnit(); // n ora è un punto nell'intervallo 0 - 1
        const gridreal* coef = coeffs(xi,yi,zi);
        return TricubicInterpolator::getValue(n, coef);
    }
}

void Grid::getMinAndMax(double& min, double& max) {
    min = MIN_PAY; max = MAX_PAY;
    for (double xi = bb.getMinX(); xi <= bb.getMaxX(); xi+=0.5){
        for (double yi = bb.getMinY(); yi <= bb.getMaxY(); yi+=0.5){
            for (double zi = bb.getMinZ(); zi <= bb.getMaxZ(); zi+=0.5){
                double w = getValue(Pointd(xi,yi,zi));
                if (w < min){
                    min = w;
                }
                if (w > max){
                    max = w;
                }
            }
        }
    }
}

void Grid::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(unit, binaryFile);
    bb.serialize(binaryFile);
    Serializer::serialize(resX, binaryFile);
    Serializer::serialize(resY, binaryFile);
    Serializer::serialize(resZ, binaryFile);
    signedDistances.serialize(binaryFile);
    weights.serialize(binaryFile);
    coeffs.serialize(binaryFile);
    fullBoxValues.serialize(binaryFile);
    target.serialize(binaryFile);

}

bool Grid::deserialize(std::ifstream& binaryFile) {
    Grid tmp;
    if (Serializer::deserialize(tmp.unit, binaryFile) &&
            tmp.bb.deserialize(binaryFile) &&
            Serializer::deserialize(tmp.resX, binaryFile) &&
            Serializer::deserialize(tmp.resY, binaryFile) &&
            Serializer::deserialize(tmp.resZ, binaryFile) &&
            tmp.signedDistances.deserialize(binaryFile) &&
            tmp.weights.deserialize(binaryFile) &&
            tmp.coeffs.deserialize(binaryFile) &&
            tmp.fullBoxValues.deserialize(binaryFile) &&
            tmp.target.deserialize(binaryFile)){
        *this = std::move(tmp);
        return true;
    }
    else
        return false;
}


