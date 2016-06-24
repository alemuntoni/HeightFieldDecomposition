#ifndef DRAWABLEGRID_H
#define DRAWABLEGRID_H

#include "lib/common/drawable_object.h"
#include "grid.h"
#include "GUI/objects/sphere.h"


class DrawableGrid: public Grid, public DrawableObject
{
    public:
        DrawableGrid();
        DrawableGrid(const Eigen::RowVector3i& resolution, const Eigen::MatrixXd& gridCoordinates, const Eigen::VectorXd& signedDistances, const Eigen::RowVector3i& gMin, const Eigen::RowVector3i& gMax);
        virtual ~DrawableGrid();

        double getKernelDistance() const;
        void setKernelDistance(double value);

        void setDrawKernel();
        void setDrawBorders();

        void setSlice(int value);
        void setSliceValue(int value);

        double getHsvFactor(double w) const;

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

        void addCube(const BoundingBox &bb);
        void deleteCubes();

    private:

        void drawLine(const Pointd& a, const Pointd& b) const;
        void drawCube(const BoundingBox& b) const;

        enum {
            DRAW_KERNEL, DRAW_BORDERS
        };
        enum {
            NO_SLICE = 0, X_SLICE = 1, Y_SLICE = 2, Z_SLICE = 3
        };

        bool visible;
        double kernelDistance;
        int drawMode;
        int slice;
        int sliceValue;
        std::vector<BoundingBox> cubes;
};

#endif // DRAWABLEGRID_H
