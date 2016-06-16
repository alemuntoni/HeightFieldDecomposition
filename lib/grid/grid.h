#ifndef GRID_H
#define GRID_H

#include "lib/common/bounding_box.h"
#include <Eigen/Core>


class Grid : public SerializableObject{
    public:
        Grid();

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

    protected:
        Pointd getPoint(int i, int j, int k) const;
        int getIndex(int i, int j, int k) const;
        BoundingBox bb;
        Eigen::RowVector3i res;
        Eigen::MatrixXd gridCoordinates;
        Eigen::VectorXd signedDistances;
        Eigen::VectorXd weights;

};

#endif // GRID_H
