#ifndef BOXLIST_H
#define BOXLIST_H

#include "box.h"

class BoxList : public DrawableObject, public SerializableObject{
    public:
        BoxList();

        void addBox(const Box3D &b);
        void setVisibleBox(int i);
        void clearBoxes();
        unsigned int getNumberBoxes();
        Box3D getBox(unsigned int i);

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
};

#endif // BOXLIST_H
