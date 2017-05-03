#ifndef HEIGHTFIELDSLIST_H
#define HEIGHTFIELDSLIST_H

#include "eigenmesh/gui/drawableeigenmesh.h"
#include "viewer/interfaces/drawable_object.h"

class HeightfieldsList : public DrawableObject, public SerializableObject{
    public:
        HeightfieldsList();
        // DrawableObject interface
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);
        void setVisibleHeightfield(int i);
        void resize(int n);
        unsigned int getNumberVerticesHeightfield(int i) const;
        Pointd getVertexOfHeightfield(int he, int v) const;
        Vec3 getTarget(int i) const;
        void setWireframe(bool b);
        void setPointShading();
        void setFlatShading();
        void setSmoothShading();
        void checkHeightfields() const;
        void rotate(const Eigen::MatrixXd m);

        void addHeightfield(const DrawableEigenMesh &m, const Vec3 &target, int i = -1, bool updateColor = true);
        unsigned int getNumHeightfields() const;

        void removeHeightfield(unsigned int i);
        const EigenMesh& getHeightfield(unsigned int i) const;
        EigenMesh& getHeightfield(unsigned int i);
        void setHeightfield(const EigenMesh& m, unsigned int i, bool updateColor=false);
        void insertHeightfield(const EigenMesh& m, const Vec3 &target, unsigned int i, bool updateColor = true);
        void explode(const Pointd &bc, double dist);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        bool deserialize(std::ifstream& binaryFile);


    private:
        std::vector<DrawableEigenMesh> heightfields;
        std::vector<Vec3> targets;
        bool visible;
        int nVisible;
};

#endif // HEIGHTFIELDSLIST_H
