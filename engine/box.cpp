#include "box.h"

Box3D::Box3D(): visible(true) {
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const Pointd& c1, const Pointd& c2, const Pointd& c3, const QColor c) : min(min), max(max), c1(c1), c2(c2), c3(c3), color(c), visible(true){
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const QColor c) : min(min), max(max), color(c), visible(true){
}

Pointd Box3D::getConstraint1() const {
    return c1;
}

Pointd Box3D::getConstraint2() const {
    return c2;
}

Pointd Box3D::getConstraint3() const {
    return c3;
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

void Box3D::draw() const {
    if (visible){
        cylinder(min, Pointd(max.x(), min.y(), min.z()), 0.05, 0.05, color);
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
        cylinder(Pointd(min.x(), max.y(), min.z()), Pointd(min.x(), max.y(), max.z()), 0.05, 0.05, color);

        sphere(c1, 0.15, QColor(255,0,255));
        sphere(c2, 0.15, QColor(255,0,255));
        sphere(c3, 0.15, QColor(255,0,255));
    }
}

Pointd Box3D::sceneCenter() const {
    return (max+min)/2;
}

double Box3D::sceneRadius() const {
    return min.dist(max);
}

bool Box3D::isVisible() const {
    return visible;
}

void Box3D::setVisible(bool b) {
    visible = b;
}

void Box3D::serialize(std::ofstream& binaryFile) const {
    min.serialize(binaryFile);
    max.serialize(binaryFile);
    c1.serialize(binaryFile);
    c2.serialize(binaryFile);
    c3.serialize(binaryFile);
    Serializer::serialize(color, binaryFile);
    Serializer::serialize(rotation, binaryFile);
}

void Box3D::deserialize(std::ifstream& binaryFile) {
    min.deserialize(binaryFile);
    max.deserialize(binaryFile);
    c1.deserialize(binaryFile);
    c2.deserialize(binaryFile);
    c3.deserialize(binaryFile);
    Serializer::deserialize(color, binaryFile);
    Serializer::deserialize(rotation, binaryFile);
}
