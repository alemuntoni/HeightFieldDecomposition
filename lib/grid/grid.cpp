#include "grid.h"

Grid::Grid() {
}

Grid::Grid(const Eigen::RowVector3i& resolution, const Eigen::MatrixXd& gridCoordinates, const Eigen::VectorXd& signedDistances, const Eigen::RowVector3i& gMin, const Eigen::RowVector3i& gMax) :
    gridCoordinates(gridCoordinates), signedDistances(signedDistances), target(0,0,0){
    bb.setMin(Pointd(gMin(0), gMin(1), gMin(2)));
    bb.setMax(Pointd(gMax(0), gMax(1), gMax(2)));
    resX = resolution(0);
    resY = resolution(1);
    resZ = resolution(2);
    weights = Array3D<double>(resX,resY,resZ, BORDER_PAY);
    for (unsigned int i = 2; i < resX-2; i++){
        for (unsigned int j = 2; j < resY-2; j++){
            for (unsigned int k = 2; k < resZ-2; ++k){
                weights(i,j,k) = STD_PAY;
            }
        }
    }
    coeffs = Array4D<double>((resX-1),(resY-1),(resZ-1),64, 0);
}

/**
 * @brief Grid::calculateWeights
 *
 * Calcola i pesi per gli heightfieltds rispetto alla normale target
 * @param d
 */
void Grid::calculateBorderWeights(const Dcel& d, bool heightfields) {
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        Pointd p1 = f->getOuterHalfEdge()->getFromVertex()->getCoordinate();
        Pointd p2 = f->getOuterHalfEdge()->getToVertex()->getCoordinate();
        Pointd p3 = f->getOuterHalfEdge()->getNext()->getToVertex()->getCoordinate();

        setNeighboroudWeigth(p1, MIN_PAY);
        setNeighboroudWeigth(p2, MIN_PAY);
        setNeighboroudWeigth(p3, MIN_PAY);

    }
    if (heightfields) {
        for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
            const Dcel::Face* f = *fit;
            if (f->getNormal().dot(target) < 0){
                Pointd p1 = f->getOuterHalfEdge()->getFromVertex()->getCoordinate();
                Pointd p2 = f->getOuterHalfEdge()->getToVertex()->getCoordinate();
                Pointd p3 = f->getOuterHalfEdge()->getNext()->getToVertex()->getCoordinate();

                setNeighboroudWeigth(p1, MAX_PAY);
                setNeighboroudWeigth(p2, MAX_PAY);
                setNeighboroudWeigth(p3, MAX_PAY);
            }

        }
    }
}

/**
 * @brief Grid::freezeKernel
 *
 * Assegna massimo peso ai punti nel kernel
 * @param value
 */
void Grid::calculateWeightsAndFreezeKernel(const Dcel& d, double value, bool heightfields) {
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
    calculateBorderWeights(d, heightfields);

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

void Grid::calculateFullBoxValues(double (*integralTricubicInterpolation)(const double *&, double, double, double, double, double, double)) {
    fullBoxValues = Array3D<double>(getResX()-1, getResY()-1, getResZ()-1);
    for (unsigned int i = 0; i < fullBoxValues.getSizeX(); ++i){
        for (unsigned int j = 0; j < fullBoxValues.getSizeY(); ++j){
            for (unsigned int k = 0; k < fullBoxValues.getSizeZ(); ++k){
                const double * coeffs;
                getCoefficients(coeffs, i, j, k);
                fullBoxValues(i,j,k) = integralTricubicInterpolation(coeffs, 0,0,0,1,1,1);
            }
        }
    }
}

double Grid::getValue(const Pointd& p) const {
    if (! bb.isStrictlyIntern(p)) return BORDER_PAY;
    unsigned int xi = getIndexOfCoordinateX(p.x()), yi = getIndexOfCoordinateY(p.y()), zi = getIndexOfCoordinateZ(p.z());
    unsigned int gridIndex = getIndex(xi,yi,zi);
    Pointd n(gridCoordinates(gridIndex,0), gridCoordinates(gridIndex,1), gridCoordinates(gridIndex,2));
    if (n == p)
        return weights(xi,yi,zi);
    else{
        n = (p - n) / getUnit(); // n ora è un punto nell'intervallo 0 - 1
        const double* coef = coeffs(xi,yi,zi);
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
    bb.serialize(binaryFile);
    Serializer::serialize(resX, binaryFile);
    Serializer::serialize(resY, binaryFile);
    Serializer::serialize(resZ, binaryFile);
    Serializer::serialize(gridCoordinates, binaryFile);
    Serializer::serialize(signedDistances, binaryFile);
    weights.serialize(binaryFile);
    coeffs.serialize(binaryFile);
    fullBoxValues.serialize(binaryFile);
    target.serialize(binaryFile);

}

void Grid::deserialize(std::ifstream& binaryFile) {
    bb.deserialize(binaryFile);
    Serializer::deserialize(resX, binaryFile);
    Serializer::deserialize(resY, binaryFile);
    Serializer::deserialize(resZ, binaryFile);
    Serializer::deserialize(gridCoordinates, binaryFile);
    Serializer::deserialize(signedDistances, binaryFile);
    weights.deserialize(binaryFile);
    coeffs.deserialize(binaryFile);
    fullBoxValues.deserialize(binaryFile);
    target.deserialize(binaryFile);
}

void Grid::setNeighboroudWeigth(const Pointd& p, double w) {
    unsigned int i = getIndexOfCoordinateX(p.x());
    unsigned int j = getIndexOfCoordinateY(p.y());
    unsigned int k = getIndexOfCoordinateZ(p.z());
    double dx = gridCoordinates(getIndex(i,j,k),0);
    double dy = gridCoordinates(getIndex(i,j,k),1);
    double dz = gridCoordinates(getIndex(i,j,k),2);

    // point in a grid point
    if (dx == p.x() &&
            dy == p.y() &&
            dz == p.z()){
        weights(i-1,j-1,k-1) = w;
        weights(i-1,j-1,k  ) = w;
        weights(i-1,j-1,k+1) = w;
        weights(i-1,j  ,k-1) = w;
        weights(i-1,j  ,k  ) = w;
        weights(i-1,j  ,k+1) = w;
        weights(i-1,j+1,k-1) = w;
        weights(i-1,j+1,k  ) = w;
        weights(i-1,j+1,k+1) = w;
        weights(i  ,j-1,k-1) = w;
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j-1,k+1) = w;
        weights(i  ,j  ,k-1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k-1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j-1,k-1) = w;
        weights(i+1,j-1,k  ) = w;
        weights(i+1,j-1,k+1) = w;
        weights(i+1,j  ,k-1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k-1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;

    }

    //point between edge xy
    if (dx == p.x() &&
            dy == p.y() &&
            dz != p.z()){
        weights(i-1,j-1,k  ) = w;
        weights(i-1,j  ,k  ) = w;
        weights(i-1,j+1,k  ) = w;
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i+1,j-1,k  ) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i-1,j-1,k+1) = w;
        weights(i-1,j  ,k+1) = w;
        weights(i-1,j+1,k+1) = w;
        weights(i  ,j-1,k+1) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j-1,k+1) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k+1) = w;
    }

    //point between edge xz
    if (dx == p.x() &&
            dy != p.y() &&
            dz == p.z()){
        weights(i-1,j  ,k-1) = w;
        weights(i-1,j  ,k  ) = w;
        weights(i-1,j  ,k+1) = w;
        weights(i  ,j  ,k-1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i+1,j  ,k-1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i-1,j+1,k-1) = w;
        weights(i-1,j+1,k  ) = w;
        weights(i-1,j+1,k+1) = w;
        weights(i  ,j+1,k-1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j+1,k-1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;

    }

    //point between edge yz
    if (dx != p.x() &&
            dy == p.y() &&
            dz == p.z()){
        weights(i  ,j-1,k-1) = w;
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j-1,k+1) = w;
        weights(i  ,j  ,k-1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k-1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j-1,k-1) = w;
        weights(i+1,j-1,k  ) = w;
        weights(i+1,j-1,k+1) = w;
        weights(i+1,j  ,k-1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k-1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;
    }

    //point between faces x
    if (dx == p.x() &&
            dy != p.y() &&
            dz != p.z()){
        weights(i-1,j  ,k  ) = w;
        weights(i-1,j  ,k+1) = w;
        weights(i-1,j+1,k  ) = w;
        weights(i-1,j+1,k+1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;
    }
    //point between faces y
    if (dx != p.x() &&
            dy == p.y() &&
            dz != p.z()){
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j-1,k+1) = w;
        weights(i+1,j-1,k  ) = w;
        weights(i+1,j-1,k+1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,  j,k+1) = w;
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j-1,k+1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;
    }
    //point between faces z
    if (dx != p.x() &&
            dy != p.y() &&
            dz == p.z()){
        weights(i  ,j  ,k-1) = w;
        weights(i  ,j+1,k-1) = w;
        weights(i+1,j  ,k-1) = w;
        weights(i+1,j+1,k-1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k+1) = w;
    }

    //point inside a cube
    if (dx != p.x() &&
            dy != p.y() &&
            dz != p.z()){
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i  ,j+1,k+1) = w;
        weights(i+1,j  ,k  ) = w;
        weights(i+1,j  ,k+1) = w;
        weights(i+1,j+1,k  ) = w;
        weights(i+1,j+1,k+1) = w;
    }


    /*// point in a grid point
    if (dx == p.x() &&
        dy == p.y() &&
        dz == p.z()){
        weights(i-1,j  ,k  ) = w;
        weights(i  ,j-1,k  ) = w;
        weights(i  ,j  ,k-1) = w;
        weights(i  ,j  ,k  ) = w;
        weights(i  ,j  ,k+1) = w;
        weights(i  ,j+1,k  ) = w;
        weights(i+1,j  ,k  ) = w;
    }

    //point between edge xy
    if (dx == p.x() &&
        dy == p.y() &&
        dz != p.z()){
        weights(i-1,j,k) = w;
        weights(i,j-1,k) = w;
        weights(i,j,k) = w;
        weights(i,j+1,k) = w;
        weights(i+1,j,k) = w;
        weights(i-1,j,k+1) = w;
        weights(i,j-1,k+1) = w;
        weights(i,j,k+1) = w;
        weights(i,j+1,k+1) = w;
        weights(i+1,j,k+1) = w;
    }

    //point between edge xz
    if (dx == p.x() &&
        dy != p.y() &&
        dz == p.z()){
        weights(i-1,j,k) = w;
        weights(i,j,k-1) = w;
        weights(i,j,k) = w;
        weights(i,j,k+1) = w;
        weights(i+1,j,k) = w;
        weights(i-1,j+1,k) = w;
        weights(i,j+1,k-1) = w;
        weights(i,j+1,k) = w;
        weights(i,j+1,k+1) = w;
        weights(i+1,j+1,k) = w;
    }

    //point between edge xz
    if (dx != p.x() &&
        dy == p.y() &&
        dz == p.z()){
        weights(i,j-1,k) = w;
        weights(i,j,k-1) = w;
        weights(i,j,k) = w;
        weights(i,j,k+1) = w;
        weights(i,j+1,k) = w;
        weights(i+1,j-1,k) = w;
        weights(i+1,j,k-1) = w;
        weights(i+1,j,k) = w;
        weights(i+1,j,k+1) = w;
        weights(i+1,j+1,k) = w;
    }

    //point between faces x
    if (dx == p.x() &&
        dy != p.y() &&
        dz != p.z()){
        weights(i, j, k) = w;
        weights(i, j, k+1) = w;
        weights(i, j+1, k) = w;
        weights(i, j+1, k+1) = w;
    }
    //point between faces y
    if (dx != p.x() &&
        dy == p.y() &&
        dz != p.z()){
        weights(i, j, k) = w;
        weights(i, j, k+1) = w;
        weights(i+1, j, k) = w;
        weights(i+1, j, k+1) = w;
    }
    //point between faces z
    if (dx != p.x() &&
        dy != p.y() &&
        dz == p.z()){
        weights(i, j, k) = w;
        weights(i, j+1, k) = w;
        weights(i+1, j, k) = w;
        weights(i+1, j+1, k) = w;
    }

    //point inside a cube
    if (dx != p.x() &&
        dy != p.y() &&
        dz != p.z()){
        weights(i   ,j   ,k   ) = w;
        weights(i   ,j   ,k+1 ) = w;
        weights(i   ,j+1 ,k   ) = w;
        weights(i   ,j+1 ,k+1 ) = w;
        weights(i+1 ,j   ,k   ) = w;
        weights(i+1 ,j   ,k+1 ) = w;
        weights(i+1 ,j+1 ,k   ) = w;
        weights(i+1 ,j+1 ,k+1 ) = w;
    }*/
}


