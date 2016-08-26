#ifndef DRAWABLEGRID_H
#define DRAWABLEGRID_H

#include "viewer/interfaces//drawable_object.h"
#include "grid.h"
#include "viewer/objects/objects.h"

class DrawableGrid: public Grid, public DrawableObject
{
    public:
        DrawableGrid();
        DrawableGrid(const Grid &g);
        DrawableGrid(const Pointi& resolution, const Array3D<Pointd>& gridCoordinates, const Array3D<gridreal>& signedDistances, const Pointd& gMin, const Pointd& gMax);
        virtual ~DrawableGrid();

        double getKernelDistance() const;
        void setKernelDistance(double value);

        void setDrawKernel();
        void setDrawBorders();

        void setSlice(int value);
        void setSliceValue(int value);

        double getHsvHFactor(double w) const;
        double getHsvVFactor(double w) const;

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

        void addCube(const BoundingBox &bb);
        void deleteCubes();

        void setStepDrawGrid(double value);

    private:

        void drawLine(const Pointd& a, const Pointd& b) const;
        void drawCube(const BoundingBox& b) const;

        enum {
            DRAW_KERNEL, DRAW_WEIGHTS
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
        double stepDrawGrid;
};

#endif // DRAWABLEGRID_H
