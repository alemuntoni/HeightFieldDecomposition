#ifndef BOXLIST_H
#define BOXLIST_H

#include "box.h"

class BoxList : public DrawableObject, public SerializableObject{
    public:
        BoxList();
        BoxList(bool cylinders);

        void addBox(const Box3D &b);
        void setVisibleBox(int i);
        void clearBoxes();
        unsigned int getNumberBoxes();
        Box3D getBox(unsigned int i);
        void setBox(unsigned int i, const Box3D &b);
        void setCylinders(bool b);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    private:
        std::vector<Box3D> boxes;
        bool visible;
        int n;
        bool cylinder;

        void drawLine(const Pointd& a, const Pointd& b) const;
        void drawCube(const Box3D& b) const;
};

#endif // BOXLIST_H
