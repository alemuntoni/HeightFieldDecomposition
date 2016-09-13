#ifndef DRAWABLEIRREGULARGRID_H
#define DRAWABLEIRREGULARGRID_H

#include "irregulargrid.h"
#include "viewer/interfaces/drawable_object.h"

class DrawableIrregularGrid : public IrregularGrid, public DrawableObject{
    public:
        DrawableIrregularGrid();
        DrawableIrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ);

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    private:
        bool visible;
};

#endif // DRAWABLEIRREGULARGRID_H
