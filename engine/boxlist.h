#ifndef BOXLIST_H
#define BOXLIST_H

#include "box.h"
#include "common/arrays.h"
#include "cgal/aabbtree.h"

class BoxList : public DrawableObject, public SerializableObject{
    public:
        BoxList();
        BoxList(bool cylinders);

        void addBox(const Box3D &b, int i = -1);

        void clearBoxes();
        unsigned int getNumberBoxes() const;
        Box3D getBox(unsigned int i) const;
        void setBox(unsigned int i, const Box3D &b);
        void insert(const BoxList &o);
        void insert(const Box3D &b, int i = -1);
        void removeBox(unsigned int i);
        void getSubBoxLists(std::vector<BoxList> &v, int nPerBoxList);
        void setIds();
        void sort(const Array2D<int> &ordering);
        void sortByTrianglesCovered();
        void generatePieces(double minimumDistance = -1);
        void calculateTrianglesCovered(const CGALInterface::AABBTree &tree);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        bool deserialize(std::ifstream& binaryFile);



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
        #ifdef VIEWER_DEFINED
        void drawLine(const Pointd& a, const Pointd& b, const Color& c) const;
        void drawCube(const Box3D& b, const Color& c) const;
        #endif
};

#endif // BOXLIST_H
