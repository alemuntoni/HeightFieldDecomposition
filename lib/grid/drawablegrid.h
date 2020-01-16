#ifndef DRAWABLEGRID_H
#define DRAWABLEGRID_H

#include "cg3/viewer/interfaces//drawable_object.h"
#include "grid.h"
#include "cg3/viewer/opengl_objects/opengl_objects3.h"

class DrawableGrid: public Grid, public cg3::DrawableObject
{
    public:
        DrawableGrid();
        DrawableGrid(const Grid &g);
		DrawableGrid(const cg3::Point3i& resolution, const cg3::Array3D<cg3::Point3d>& gridCoordinates, const cg3::Array3D<gridreal>& signedDistances, const cg3::Point3d& gMin, const cg3::Point3d& gMax);
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
		cg3::Point3d sceneCenter() const;
        double sceneRadius() const;

		void addCube(const cg3::BoundingBox3 &bb);
        void deleteCubes();

        void setStepDrawGrid(double value);

        void updateMinSignedDistance();

    private:

		void drawLine(const cg3::Point3d& a, const cg3::Point3d& b) const;
		void drawCube(const cg3::BoundingBox3& b) const;

        enum {
            DRAW_KERNEL, DRAW_WEIGHTS
        };
        enum {
            NO_SLICE = 0, X_SLICE = 1, Y_SLICE = 2, Z_SLICE = 3
        };

        double kernelDistance;
        int drawMode;
        int slice;
        int sliceValue;
		std::vector<cg3::BoundingBox3> cubes;
        double stepDrawGrid;
        double minSignedDistance;
};

#endif // DRAWABLEGRID_H
