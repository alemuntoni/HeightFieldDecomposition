#ifndef DRAWABLEGRID_H
#define DRAWABLEGRID_H

#include "lib/common/drawable_object.h"
#include "grid.h"


class DrawableGrid: public Grid, public DrawableObject
{
    public:
        DrawableGrid();

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool);
};

#endif // DRAWABLEGRID_H
