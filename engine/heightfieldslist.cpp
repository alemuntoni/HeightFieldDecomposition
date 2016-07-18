#include "heightfieldslist.h"
#include "common.h"

HeightfieldsList::HeightfieldsList() : visible(true), nVisible(-1) {
}

void HeightfieldsList::draw() const {
    if (visible){
        if (nVisible < 0){
            for (unsigned int i = 0; i < heightfields.size(); ++i){
                heightfields[i].draw();
            }
        }
        else {
            heightfields[nVisible].draw();
        }
    }
}

Pointd HeightfieldsList::sceneCenter() const{
    return std::move(Pointd());
}

double HeightfieldsList::sceneRadius() const {
    return -1;
}

bool HeightfieldsList::isVisible() const {
    return visible;
}

void HeightfieldsList::setVisible(bool b) {
    visible = b;
}

void HeightfieldsList::setVisibleHeightfield(int i) {
    assert (i < heightfields.size());
    nVisible = i;
}

void HeightfieldsList::resize(int n) {
    heightfields.resize(n);
    targets.resize(n);
}

void HeightfieldsList::addHeightfield(const DrawableIGLMesh& m, const Vec3& target, int i) {
    if (i < 0){
        heightfields.push_back(m);
        targets.push_back(target);
        QColor c = colorOfNormal(target);
        heightfields[heightfields.size()-1].setColor(c.redF(), c.greenF(), c.blueF());
    }
    else {
        heightfields[i] = m;
        targets[i] = target;
        QColor c = colorOfNormal(target);
        heightfields[i].setColor(c.redF(), c.greenF(), c.blueF());
    }
}

unsigned int HeightfieldsList::getNumHeightfields() const {
    return heightfields.size();
}
