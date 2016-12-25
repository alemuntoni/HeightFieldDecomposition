#ifndef DRAWABLEIGLMESH_H
#define DRAWABLEIGLMESH_H

#include "../../viewer/interfaces/drawable_mesh.h"
#include "../iglmesh.h"
namespace IGLInterface {
    class DrawableIGLMesh : public IGLMesh, public DrawableMesh {
        public:
            DrawableIGLMesh();
            DrawableIGLMesh(const SimpleIGLMesh &m);
            DrawableIGLMesh(const IGLMesh &m);
            virtual ~DrawableIGLMesh();

            // DrawableObject interface
            void draw() const;
            Pointd sceneCenter() const;
            double sceneRadius() const;
    };
}

#endif // DRAWABLEIGLMESH_H
