#ifndef BOX_H
#define BOX_H

#include "common/drawable_object.h"
#include "common/bounding_box.h"
#include "viewer/objects/cylinder.h"
#include "viewer/objects/sphere.h"
#include "igl/iglmesh.h"

class Box3D : public BoundingBox, public DrawableObject{
    public:
        Box3D();
        Box3D(const Pointd &min, const Pointd &max, const Pointd &c1 = Pointd(), const Pointd &c2 = Pointd(), const Pointd &c3 = Pointd(), const QColor c = QColor(0,0,0));
        Box3D(const Pointd &min, const Pointd &max, const QColor c);

        void setColor(const QColor &c);
        QColor getColor() const;
        /*const Pointd& getMin() const;
        void setMin(const Pointd& value);
        const Pointd& getMax() const;
        void setMax(const Pointd& value);*/

        const Pointd& getConstraint1() const;
        const Pointd& getConstraint2() const;
        const Pointd& getConstraint3() const;
        void setConstraint1(const Pointd &p);
        void setConstraint2(const Pointd &p);
        void setConstraint3(const Pointd &p);
        void setRotationMatrix(const Eigen::Matrix3d &rot);
        const Eigen::Matrix3d& getRotationMatrix() const;

        double getVolume() const;
        void getRotatedExtremes(std::vector<Pointd> &v) const;

        void setW(double d);
        void setH(double d);
        void setD(double d);
        void moveX(double d);
        void moveY(double d);
        void moveZ(double d);
        void getIGLMesh(SimpleIGLMesh& box) const;

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

        const Vec3& getTarget() const;
        void setTarget(const Vec3& value);

    protected:
        //Pointd min, max;
        Pointd c1, c2, c3;
        QColor color;
        bool visible;
        Vec3 target;
        Eigen::Matrix3d rotation;
};

inline void Box3D::setColor(const QColor& c) {
    color = c;
}

inline QColor Box3D::getColor() const {
    return color;
}

/*inline const Pointd& Box3D::getMin() const {
    return min;
}

inline void Box3D::setMin(const Pointd& value) {
    min = value;
}

inline const Pointd& Box3D::getMax() const {
    return max;
}

inline void Box3D::setMax(const Pointd& value) {
    max = value;
}*/

inline const Pointd& Box3D::getConstraint1() const {
    return c1;
}

inline const Pointd& Box3D::getConstraint2() const {
    return c2;
}

inline const Pointd& Box3D::getConstraint3() const {
    return c3;
}

inline const Eigen::Matrix3d&Box3D::getRotationMatrix() const {
    return rotation;
}

inline double Box3D::getVolume() const {
    return (max.x()-min.x())*(max.y()-min.y())*(max.z()-min.z());
}

inline void Box3D::setW(double d) {
    max.setX(min.x()+d);
}

inline void Box3D::setH(double d) {
    max.setY(min.y()+d);
}

inline void Box3D::setD(double d) {
    max.setZ(min.z()+d);
}

inline void Box3D::moveX(double d) {
    min.setX(min.x() + d);
    max.setX(max.x() + d);
}

inline void Box3D::moveY(double d) {
    min.setY(min.y() + d);
    max.setY(max.y() + d);
}

inline void Box3D::moveZ(double d) {
    min.setZ(min.z() + d);
    max.setZ(max.z() + d);
}

#endif // BOX_H
