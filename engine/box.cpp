#include "box.h"

Box3D::Box3D(): visible(true) {
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const Pointd& c1, const Pointd& c2, const Pointd& c3, const QColor c) : BoundingBox(min, max), c1(c1), c2(c2), c3(c3), color(c), visible(true){
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const QColor c) :BoundingBox(min, max), color(c), visible(true){
}

void Box3D::setConstraint1(const Pointd& p) {
    c1 = p;
}

void Box3D::setConstraint2(const Pointd& p) {
    c2 = p;
}

void Box3D::setConstraint3(const Pointd& p) {
    c3 = p;
}

void Box3D::setRotationMatrix(const Eigen::Matrix3d& rot) {
    rotation = rot;
}

void Box3D::getRotatedExtremes(std::vector<Pointd>& v) const {
    v.resize(8);
    Pointd p = min;
    p.rotate(rotation);
    v[0] = p;
    p.set(max.x(), min.y(), min.z());
    p.rotate(rotation);
    v[1] = p;
    p.set(max.x(), min.y(), max.z());
    p.rotate(rotation);
    v[2] = p;
    p.set(min.x(), min.y(), max.z());
    p.rotate(rotation);
    v[3] = p;
    p.set(min.x(), max.y(), min.z());
    p.rotate(rotation);
    v[4] = p;
    p.set(max.x(), max.y(), min.z());
    p.rotate(rotation);
    v[5] = p;
    p = max;
    p.rotate(rotation);
    v[6] = p;
    p.set(min.x(), max.y(), max.z());
    p.rotate(rotation);
    v[7] = p;
}

void Box3D::getIGLMesh(IGLMesh& box) const {
    box.clear();
    std::vector<Pointd> extremes;
    getRotatedExtremes(extremes);
    box.resizeVertices(8);
    for (unsigned int i = 0; i < 8; i++){
        box.setVertex(i, extremes[i].x(), extremes[i].y(), extremes[i].z());
    }
    box.resizeFaces(12);
    box.setFace(0, 0,1,2);
    box.setFace(1, 0,2,3);
    box.setFace(2, 2,1,5);
    box.setFace(3, 2,5,6);
    box.setFace(4, 5,1,0);
    box.setFace(5, 5,0,4);
    box.setFace(6, 6,5,4);
    box.setFace(7, 6,4,7);
    box.setFace(8, 7,4,0);
    box.setFace(9, 7,0,3);
    box.setFace(10, 7,3,2);
    box.setFace(11, 7,2,6);
}

void Box3D::draw() const {
    if (visible){
        Pointd c1 = this->c1, c2 = this->c2, c3 = this->c3;
        c1.rotate(rotation);
        c2.rotate(rotation);
        c3.rotate(rotation);
        std::vector<Pointd> p;
        getRotatedExtremes(p);
        cylinder(p[0], p[1], 0.05, 0.05, color);
        cylinder(p[1], p[2], 0.05, 0.05, color);
        cylinder(p[2], p[3], 0.05, 0.05, color);
        cylinder(p[0], p[3], 0.05, 0.05, color);

        cylinder(p[4], p[5], 0.05, 0.05, color);
        cylinder(p[5], p[6], 0.05, 0.05, color);
        cylinder(p[6], p[7], 0.05, 0.05, color);
        cylinder(p[4], p[7], 0.05, 0.05, color);

        cylinder(p[0], p[4], 0.05, 0.05, color);
        cylinder(p[1], p[5], 0.05, 0.05, color);
        cylinder(p[2], p[6], 0.05, 0.05, color);
        cylinder(p[3], p[7], 0.05, 0.05, color);

        /*cylinder(min, Pointd(max.x(), min.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), max.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), Pointd(min.x(), max.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), min, 0.05, 0.05, color);

        cylinder(Pointd(min.x(), min.y(), max.z()), Pointd(max.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), max.z()), max, 0.05, 0.05, color);
        cylinder(max, Pointd(min.x(), max.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), max.z()), Pointd(min.x(), min.y(), max.z()), 0.05, 0.05, color);

        cylinder(min, Pointd(min.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), max, 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), Pointd(min.x(), max.y(), max.z()), 0.05, 0.05, color);*/

        sphere(c1, 0.15, QColor(255,0,255));
        sphere(c2, 0.15, QColor(255,0,255));
        sphere(c3, 0.15, QColor(255,0,255));
    }
}

Pointd Box3D::sceneCenter() const {
    return (max+min)/2;
}

double Box3D::sceneRadius() const {
    return -1;
    //return min.dist(max);
}

bool Box3D::isVisible() const {
    return visible;
}

void Box3D::setVisible(bool b) {
    visible = b;
}

void Box3D::serialize(std::ofstream& binaryFile) const {
    BoundingBox::serialize(binaryFile);
    //min.serialize(binaryFile);
    //max.serialize(binaryFile);
    c1.serialize(binaryFile);
    c2.serialize(binaryFile);
    c3.serialize(binaryFile);
    Serializer::serialize(color, binaryFile);
    target.serialize(binaryFile);
    Serializer::serialize(rotation, binaryFile);
}

void Box3D::deserialize(std::ifstream& binaryFile) {
    BoundingBox::deserialize(binaryFile);
    //min.deserialize(binaryFile);
    //max.deserialize(binaryFile);
    c1.deserialize(binaryFile);
    c2.deserialize(binaryFile);
    c3.deserialize(binaryFile);
    Serializer::deserialize(color, binaryFile);
    target.deserialize(binaryFile);
    Serializer::deserialize(rotation, binaryFile);
}

const Vec3& Box3D::getTarget() const {
    return target;
}

void Box3D::setTarget(const Vec3& value) {
    target = value;
}
