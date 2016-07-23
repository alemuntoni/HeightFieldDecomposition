#include "iglinterface.h"

namespace IGLInterface {
    /**
     * @brief generateGridAndDistanceField
     *
     * The bounding box  must be sufficiently larger to build a Grid with a step gridUnit*gridUnit*gridUnit (default = 2).
     * If integer is true, a grid in only integer values will be created (gridUnit will be casted to int).
     *
     * The Grid generated will have a border of two points guaranteed to be outside the mesh.
     *
     * Every point of the grid will be stored on 3D Array grid, and the distance field will be stored on the parallel 3D array distanceField.
     */
    void generateGridAndDistanceField(Array3D<Pointd> &grid, Array3D<double> &distanceField, const SimpleIGLMesh &m, double gridUnit, bool integer) {
        assert(gridUnit > 0);
        // Bounding Box
        Eigen::RowVector3d Vmin, Vmax;
        m.getBoundingBox(Vmin, Vmax);

        // create grid GV
        Eigen::RowVector3d border((int)gridUnit*5, (int)gridUnit*5, (int)gridUnit*5);
        Eigen::RowVector3d nGmin;
        Eigen::RowVector3d nGmax;
        if (integer) {
            Eigen::RowVector3i Gmini = (Vmin).cast<int>() - border.cast<int>();
            Eigen::RowVector3i Gmaxi = (Vmax).cast<int>() + border.cast<int>();
            nGmin = Gmini.cast<double>();
            nGmax = Gmaxi.cast<double>();
            gridUnit = (int)gridUnit;
            assert(gridUnit > 0);
        }
        else {
            nGmin = Vmin - border;
            nGmax = Vmax + border; //bounding box of the Grid
        }
        Eigen::RowVector3i res = (nGmax.cast<int>() - nGmin.cast<int>())/2; res(0)+=1; res(1)+=1; res(2)+=1;
        Eigen::MatrixXd GV(res(0)*res(1)*res(2),3);

        grid.resize(res(0), res(1), res(2));
        distanceField.resize(res(0), res(1), res(2));

        int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
        for (int i = 0; i < res(0); ++i){
            yi = nGmin(1);
            for (int j = 0; j < res(1); ++j){
                zi = nGmin(2);
                for (int k = 0; k < res(2); ++k){
                    GV.row(k+res(2)*(j + res(1)*i)) = Eigen::RowVector3i(xi,yi,zi).cast<double>();
                    zi+=gridUnit;
                }
                yi+=gridUnit;
            }
            xi += gridUnit;
        }

        // compute values
        Eigen::VectorXd S = m.getSignedDistance(GV);

        for (int i = 0; i < res(0); i++){
            for (int j = 0; j < res(1); j++){
                for (int k = 0; k < res(2); k++){
                    grid(i,j,k) = Pointd(GV.row(k+res(2)*(j + res(1)*i)));
                    distanceField(i,j,k) = S(k+res(2)*(j + res(1)*i));
                }
            }
        }
    }
}
