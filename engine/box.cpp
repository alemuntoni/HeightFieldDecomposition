#include "box.h"

Box::Box(): visible(true) {
}

Box::Box(const Pointd& min, const Pointd& max, const Pointd& c1, const Pointd& c2, const Pointd& c3, const QColor c) : min(min), max(max), c1(c1), c2(c2), c3(c3), color(c), visible(true){
}

Box::Box(const Pointd& min, const Pointd& max, const QColor c) : min(min), max(max), color(c), visible(true){
}

void Box::draw() const {
    if (visible){
        cylinder(min, Pointd(max.x(), min.y(), min.z()), 0.3, 0.3, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), max.y(), min.z()), 0.3, 0.3, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), Pointd(min.x(), max.y(), min.z()), 0.3, 0.3, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), min, 0.3, 0.3, color);

        cylinder(Pointd(min.x(), min.y(), max.z()), Pointd(max.x(), min.y(), max.z()), 0.3, 0.3, color);
        cylinder(Pointd(max.x(), min.y(), max.z()), max, 0.3, 0.3, color);
        cylinder(max, Pointd(min.x(), max.y(), max.z()), 0.3, 0.3, color);
        cylinder(Pointd(min.x(), max.y(), max.z()), Pointd(min.x(), min.y(), max.z()), 0.3, 0.3, color);

        cylinder(min, Pointd(min.x(), min.y(), max.z()), 0.3, 0.3, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), min.y(), max.z()), 0.3, 0.3, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), max, 0.3, 0.3, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), Pointd(min.x(), max.y(), max.z()), 0.3, 0.3, color);
    }
}

Pointd Box::sceneCenter() const {
    return (max+min)/2;
}

double Box::sceneRadius() const {
    return min.dist(max);
}

bool Box::isVisible() const {
    return visible;
}

void Box::setVisible(bool b) {
    visible = b;
}
