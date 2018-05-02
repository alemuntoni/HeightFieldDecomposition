#ifndef HEIGHTFIELDSLIST_H
#define HEIGHTFIELDSLIST_H

#include "cg3/viewer/drawable_objects/drawable_eigenmesh.h"
#include "cg3/viewer/interfaces/drawable_object.h"

class HeightfieldsList : public cg3::DrawableObject, cg3::SerializableObject{
    public:
        HeightfieldsList();
        // DrawableObject interface
        void draw() const;
        cg3::Pointd sceneCenter() const;
        double sceneRadius() const;
        void setVisibleHeightfield(int i);
        void resize(int n);
        unsigned int getNumberVerticesHeightfield(int i) const;
        cg3::Pointd getVertexOfHeightfield(int he, int v) const;
        cg3::Vec3 getTarget(int i) const;
        void setWireframe(bool b);
        void setPointShading();
        void setFlatShading();
        void setSmoothShading();
        void checkHeightfields() const;
        void rotate(const Eigen::MatrixXd m);

        void addHeightfield(const cg3::DrawableEigenMesh &m, const cg3::Vec3 &target, int i = -1, bool updateColor = true);
        unsigned int getNumHeightfields() const;

        void removeHeightfield(unsigned int i);
        const cg3::EigenMesh& getHeightfield(unsigned int i) const;
        cg3::EigenMesh& getHeightfield(unsigned int i);
        void setHeightfield(const cg3::EigenMesh& m, unsigned int i, bool updateColor=false);
        void insertHeightfield(const cg3::EigenMesh& m, const cg3::Vec3 &target, unsigned int i, bool updateColor = true);
        void explode(const cg3::Pointd &bc, double dist);

        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);


    private:
        std::vector<cg3::DrawableEigenMesh> heightfields;
        std::vector<cg3::Vec3> targets;
        int nVisible;
};

#endif // HEIGHTFIELDSLIST_H
