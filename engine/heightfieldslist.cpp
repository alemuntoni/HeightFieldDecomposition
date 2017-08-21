#include "heightfieldslist.h"
#include "common.h"

using namespace cg3;

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
    BoundingBox bb(Pointd(-1,-1,-1), Pointd(1,1,1));
    for (unsigned int i = 0; i < heightfields.size(); i++){
        bb.min() = bb.min().min(heightfields[i].getBoundingBox().min());
        bb.max() = bb.max().max(heightfields[i].getBoundingBox().max());
    }
    return bb.center();
}

double HeightfieldsList::sceneRadius() const {
    BoundingBox bb(Pointd(-1,-1,-1), Pointd(1,1,1));
    for (unsigned int i = 0; i < heightfields.size(); i++){
        bb.min() = bb.min().min(heightfields[i].getBoundingBox().min());
        bb.max() = bb.max().max(heightfields[i].getBoundingBox().max());
    }
    return bb.diag() / 2;
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

Vec3 HeightfieldsList::getTarget(int i) const {
    assert(i < (int)heightfields.size());
    return targets[i];
}

void HeightfieldsList::setWireframe(bool b) {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setWireframe(b);
    }
}

void HeightfieldsList::setPointShading() {
    for (unsigned int i = 0; i < heightfields.size(); ++i){
        heightfields[i].setPointsShading();
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

void HeightfieldsList::checkHeightfields() const {
    for (unsigned int i = 0; i < heightfields.size(); i++){
        const DrawableEigenMesh& m = heightfields[i];
        for (unsigned int f = 0; f < m.getNumberFaces(); f++){
            if (m.getFaceNormal(f).dot(targets[i]) < FLIP_ANGLE-CG3_EPSILON && m.getFaceNormal(f).dot(targets[i]) > -1 + CG3_EPSILON){
                std::cerr << "Hieghtfield: " << i << "; Triangle: " << f << "; Flip: " << m.getFaceNormal(f).dot(targets[i]) << "\n";
            }
        }
    }
}

void HeightfieldsList::rotate(const Eigen::MatrixXd m) {
    for (unsigned int i = 0; i < heightfields.size(); i++){
        heightfields[i].rotate(m);
        heightfields[i].updateVerticesNormals();
        targets[i].rotate(m);
        targets[i].normalize();
    }
}

void HeightfieldsList::addHeightfield(const DrawableEigenMesh& m, const Vec3& target, int i, bool updateColor) {
    if (i < 0){
        heightfields.push_back(m);
        targets.push_back(target);
        if (updateColor){
            Color c = colorOfNormal(target);
            heightfields[heightfields.size()-1].setFaceColor(c.redF(), c.greenF(), c.blueF());
            for (unsigned int i = 0; i < m.getNumberFaces(); i++){
                if (m.getFaceNormal(i).dot(target) < FLIP_ANGLE-CG3_EPSILON)
                    heightfields[heightfields.size()-1].setFaceColor(0,0,0,i);
            }
        }
    }
    else {
        heightfields[i] = m;
        targets[i] = target;
        if (updateColor) {
            Color c = colorOfNormal(target);
            heightfields[i].setFaceColor(c.redF(), c.greenF(), c.blueF());
            for (unsigned int j = 0; j < m.getNumberFaces(); j++){
                if (m.getFaceNormal(j).dot(target) < FLIP_ANGLE-CG3_EPSILON)
                    heightfields[i].setFaceColor(0,0,0,j);
            }
        }
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

const cg3::EigenMesh& HeightfieldsList::getHeightfield(unsigned int i) const {
    assert (i < heightfields.size());
    return heightfields[i];
}

cg3::EigenMesh& HeightfieldsList::getHeightfield(unsigned int i) {
    assert (i < heightfields.size());
    return heightfields[i];
}

void HeightfieldsList::setHeightfield(const cg3::EigenMesh& m, unsigned int i, bool updateColor) {
    assert (i < heightfields.size());
    heightfields[i] = m;
    if (updateColor){
        Color c = colorOfNormal(targets[i]);
        heightfields[i].setFaceColor(c.redF(), c.greenF(), c.blueF());
        for (unsigned int j = 0; j < m.getNumberFaces(); j++){
            if (m.getFaceNormal(j).dot(targets[i]) < FLIP_ANGLE-CG3_EPSILON)
                heightfields[i].setFaceColor(0,0,0,j);
        }
    }
}

void HeightfieldsList::insertHeightfield(const cg3::EigenMesh& m, const Vec3& target, unsigned int i, bool updateColor) {
    assert (i < heightfields.size()+1);
    heightfields.insert(heightfields.begin() + i, m);
    targets.insert(targets.begin() + i, target);
    if (updateColor){
        Color c = colorOfNormal(target);
        heightfields[i].setFaceColor(c.redF(), c.greenF(), c.blueF());
        for (unsigned int j = 0; j < m.getNumberFaces(); j++){
            if (m.getFaceNormal(j).dot(target) < FLIP_ANGLE-CG3_EPSILON)
                heightfields[i].setFaceColor(0,0,0,j);
        }
    }
}

void HeightfieldsList::explode(const Pointd& bc, double dist) {
    for (unsigned int i = 0; i < heightfields.size(); ++i) {
        Pointd translation;
        Vec3 v = heightfields[i].getBarycenter() - bc;
        v.normalize();
        translation = v * dist;
        heightfields[i].translate(translation);
    }
}

void HeightfieldsList::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(heightfields, binaryFile);
    Serializer::serialize(targets, binaryFile);
}

bool HeightfieldsList::deserialize(std::ifstream& binaryFile) {
    std::vector<DrawableEigenMesh> tmp;
    if (Serializer::deserialize(tmp, binaryFile) &&
            Serializer::deserialize(targets, binaryFile)){
        heightfields = std::move(tmp);
        nVisible = -1;
        return true;
    }
    else
        return false;

}
