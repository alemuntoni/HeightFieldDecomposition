#ifndef BOXLIST_H
#define BOXLIST_H

#include "box.h"

class BoxList : public DrawableObject, public SerializableObject{
    public:
        BoxList();
        BoxList(bool cylinders);

        void addBox(const Box3D &b);

        void clearBoxes();
        unsigned int getNumberBoxes() const;
        Box3D getBox(unsigned int i) const;
        void setBox(unsigned int i, const Box3D &b);
        void insert(const BoxList &o);
        void removeBox(unsigned int i);
        void getSubBoxLists(std::vector<BoxList> &v, int nPerBoxList);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);



        //visualization

        void setVisibleBox(int i);
        void setCylinders(bool b);

        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    private:
        std::vector<Box3D> boxes;

        //visualization
        bool visible;
        int visibleBox;
        bool cylinder;

        void drawLine(const Pointd& a, const Pointd& b, const QColor& c) const;
        void drawCube(const Box3D& b, const QColor& c) const;
};

#endif // BOXLIST_H
