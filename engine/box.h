#ifndef BOX_H
#define BOX_H

#include "lib/common/drawable_object.h"
#include "GUI/objects/cylinder.h"

class Box : public DrawableObject{
    public:
        Box();
        Box(const Pointd &min, const Pointd &max, const Pointd &c1 = Pointd(), const Pointd &c2 = Pointd(), const Pointd &c3 = Pointd(), const QColor c = QColor(0,0,0));
        Box(const Pointd &min, const Pointd &max, const QColor c);

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    protected:
        Pointd min, max;
        Pointd c1, c2, c3;
        QColor color;
        bool visible;
};

#endif // BOX_H
