#include "drawableiglmesh.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace IGLInterface {
    DrawableIGLMesh::DrawableIGLMesh() {
    }

    DrawableIGLMesh::DrawableIGLMesh(const SimpleIGLMesh& m) : IGLMesh(m) {
    }

    DrawableIGLMesh::DrawableIGLMesh(const IGLMesh& m) : IGLMesh(m){
    }

    DrawableIGLMesh::~DrawableIGLMesh(){
    }

    void DrawableIGLMesh::draw() const {
        DrawableMesh::draw(V.rows(), F.rows(), V.data(), F.data(), NV.data(), CV.data(), NF.data(), CF.data());
    }

    Pointd DrawableIGLMesh::sceneCenter() const {
        return Pointd((BBmin(0) + BBmax(0))/2, (BBmin(1) + BBmax(1))/2, (BBmin(2) + BBmax(2))/2);
    }

    double DrawableIGLMesh::sceneRadius() const {
        return Pointd(BBmin(0), BBmin(1), BBmin(2)).dist(Pointd(BBmax(0), BBmax(1), BBmax(2)));
    }
}
