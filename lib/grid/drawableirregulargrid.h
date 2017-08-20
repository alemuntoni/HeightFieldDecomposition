#ifndef DRAWABLEIRREGULARGRID_H
#define DRAWABLEIRREGULARGRID_H

#include "irregulargrid.h"
#include "cg3/viewer/interfaces/drawable_object.h"

class DrawableIrregularGrid : public IrregularGrid, public cg3::DrawableObject{
    public:
        DrawableIrregularGrid();
        DrawableIrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ);

        // DrawableObject interface
        void draw() const;
        cg3::Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    private:
        bool visible;
};

#endif // DRAWABLEIRREGULARGRID_H
