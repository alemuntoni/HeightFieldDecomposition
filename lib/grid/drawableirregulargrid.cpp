#include "drawableirregulargrid.h"
#include "cg3/viewer/renderable_objects/renderable_objects.h"

using namespace cg3;

DrawableIrregularGrid::DrawableIrregularGrid() : visible(true){
}

DrawableIrregularGrid::DrawableIrregularGrid(unsigned int resX, unsigned int resY, unsigned int resZ) : IrregularGrid(resX, resY, resZ), visible(true) {
}

void DrawableIrregularGrid::draw() const {
    if (visible) {
        for (unsigned int i = 0; i < resX-1; i++) {
            for (unsigned int j = 0; j < resY-1; j++) {
                for (unsigned int k = 0; k < resZ-1; k++) {
                    viewer::drawBox(points(i,j,k),points(i+1,j,k),points(i+1,j,k+1),points(i,j,k+1),points(i,j+1,k),points(i+1,j+1,k),points(i+1,j+1,k+1),points(i,j+1,k+1),QColor(0,0,0));
                }
            }
        }
    }
}

Pointd DrawableIrregularGrid::sceneCenter() const {
    return Pointd();
}

double DrawableIrregularGrid::sceneRadius() const {
    return -1;
}

bool DrawableIrregularGrid::isVisible() const {
    return visible;
}

void DrawableIrregularGrid::setVisible(bool b) {
    visible = b;
}
