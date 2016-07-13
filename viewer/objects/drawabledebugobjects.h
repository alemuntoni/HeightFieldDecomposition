#ifndef DRAWABLEDEBUGOBJECTS_H
#define DRAWABLEDEBUGOBJECTS_H

#include "../../common/drawable_object.h"
#include "../../common/bounding_box.h"


class DrawableDebugObjects : public DrawableObject{
    public:
        DrawableDebugObjects();
        virtual ~DrawableDebugObjects();

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

        void addDebugSphere(const Pointd& center, double radius, const QColor &color, int precision = 4);
        void clearDebugSpheres();
        void addDebugCylinder(const Pointd& a, const Pointd& b, double radius, const QColor color);
        void clearDebugCylinders();

    protected:
        typedef struct {
                Pointd center;
                double radius;
                QColor color;
                int precision;
        } Sphere;
        typedef struct {
                Pointd a;
                Pointd b;
                double radius;
                QColor color;
        } Cylinder;
        std::vector<Sphere> debugSpheres;
        std::vector<Cylinder> debugCylinders;

        bool visible;
};

#endif // DRAWABLEDEBUGOBJECTS_H
