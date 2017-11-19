#ifndef BOXLIST_H
#define BOXLIST_H

#include "box.h"
#include "cg3/data_structures/arrays.h"
#include "cg3/cgal/cgal_aabbtree.h"

class BoxList : public cg3::DrawableObject, public cg3::SerializableObjectOld{
    public:
        BoxList();
        BoxList(bool cylinders);

        void addBox(const Box3D &b, int i = -1);

        void clearBoxes();
        unsigned int getNumberBoxes() const;
        unsigned int size() const;
        Box3D& operator[](unsigned int i);
        const Box3D& operator[](unsigned int i) const;
        const Box3D& getBox(unsigned int i) const;
        const Box3D& find(unsigned int id) const;
        Box3D& find(unsigned int id);
        unsigned int getIndexOf(unsigned int id) const;
        void setBox(unsigned int i, const Box3D &b);
        void insert(const BoxList &o);
        void insert(const Box3D &b, int i = -1);
        void removeBox(unsigned int i);
        void getSubBoxLists(std::vector<BoxList> &v, int nPerBoxList);
        void setIds();
        void sort(const cg3::Array2D<int> &ordering);
        void sortByTrianglesCovered();
        void sortByHeight();
        void generatePieces(double minimumDistance = -1);
        void calculateTrianglesCovered(const cg3::cgal::AABBTree &tree);
        void changeBoxLimits(const cg3::BoundingBox &newLimits, unsigned int i);
        std::vector<Box3D>::const_iterator begin() const;
        std::vector<Box3D>::const_iterator end() const;
        std::vector<Box3D>::iterator begin();
        std::vector<Box3D>::iterator end();

        // SerializableObject interface
        void serializeOld(std::ofstream& binaryFile) const;
        bool deserializeOld(std::ifstream& binaryFile);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);



        //visualization

        void setVisibleBox(int i);
        void setCylinders(bool b);
        void visualizeEigenMeshBox(bool b);

        // DrawableObject interface
        void draw() const;
        cg3::Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

    private:
        std::vector<Box3D> boxes;

        //visualization
        bool visible;
        int visibleBox;
        bool cylinder;
        bool eigenMesh;
        #ifdef CG3_VIEWER_DEFINED
        void drawLine(const cg3::Pointd& a, const cg3::Pointd& b, const cg3::Color& c) const;
        void drawCube(const Box3D& b, const cg3::Color& c) const;
        #endif
};

#endif // BOXLIST_H
