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
    assert (i < (int)heightfields.size());
    nVisible = i;
}

void HeightfieldsList::resize(int n) {
    heightfields.resize(n);
    targets.resize(n);
}

unsigned int HeightfieldsList::getNumberVerticesHeightfield(int i) const {
    assert (i < (int)heightfields.size());
    return heightfields[i].getNumberVertices();
}

Pointd HeightfieldsList::getVertexOfHeightfield(int he, int v) const {
    assert (he < (int)heightfields.size());
    return heightfields[he].getVertex(v);
}

void HeightfieldsList::setWireframe(bool b) {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setWireframe(b);
    }
}

void HeightfieldsList::setPointShading() {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setPointShading();
    }
}

void HeightfieldsList::setFlatShading() {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setFlatShading();
    }
}

void HeightfieldsList::setSmoothShading() {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setSmoothShading();
    }
}

void HeightfieldsList::addHeightfield(const IGLInterface::DrawableIGLMesh& m, const Vec3& target, int i) {
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

void HeightfieldsList::removeHeightfield(unsigned int i) {
    assert (i < heightfields.size());
    heightfields.erase(heightfields.begin()+i);
    targets.erase(targets.begin()+i);
}

const IGLInterface::IGLMesh& HeightfieldsList::getHeightfield(unsigned int i) const {
    assert (i < heightfields.size());
    return heightfields[i];
}

/*IGLInterface::IGLMesh HeightfieldsList::getHeightfield(unsigned int i) const {
    assert (i < heightfields.size());
    return heightfields[i];
}*/

void HeightfieldsList::setHeightfield(const IGLInterface::IGLMesh& he, unsigned int i) {
    assert (i < heightfields.size());
    heightfields[i] = he;
}

void HeightfieldsList::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(heightfields, binaryFile);
    Serializer::serialize(targets, binaryFile);
}

void HeightfieldsList::deserialize(std::ifstream& binaryFile) {
    Serializer::deserialize(heightfields, binaryFile);
    Serializer::deserialize(targets, binaryFile);
    nVisible = -1;
}
