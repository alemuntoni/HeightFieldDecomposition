#include "grid.h"

Grid::Grid() {

}

void Grid::serialize(std::ofstream& binaryFile) const {
    bb.serialize(binaryFile);
    Serializer::serialize(res(0), binaryFile);
    Serializer::serialize(res(1), binaryFile);
    Serializer::serialize(res(2), binaryFile);
    Serializer::serialize(gridCoordinates, binaryFile);
    Serializer::serialize(signedDistances, binaryFile);

}

void Grid::deserialize(std::ifstream& binaryFile) {
    bb.deserialize(binaryFile);
    Serializer::deserialize(res(0), binaryFile);
    Serializer::deserialize(res(1), binaryFile);
    Serializer::deserialize(res(2), binaryFile);
    Serializer::deserialize(gridCoordinates, binaryFile);
    Serializer::deserialize(signedDistances, binaryFile);
}

Pointd Grid::getPoint(int i, int j, int k) const {
    int ind = getIndex(i,j,k);
    return Pointd(gridCoordinates(ind,0), gridCoordinates(ind,1), gridCoordinates(ind,2));
}

int Grid::getIndex(int i, int j, int k) const {
    return i+res(0)*(j + res(1)*k);
}
