#ifndef HEIGHTFIELDSLIST_H
#define HEIGHTFIELDSLIST_H

#include "igl/drawableiglmesh.h"
#include "common/drawable_object.h"

class HeightfieldsList : public DrawableObject{
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

        void addHeightfield(const DrawableIGLMesh &m, const Vec3 &target, int i = -1);
        unsigned int getNumHeightfields() const;

    private:
        std::vector<DrawableIGLMesh> heightfields;
        std::vector<Vec3> targets;
        bool visible;
        int nVisible;
};

#endif // HEIGHTFIELDSLIST_H
