#ifndef DRAWABLEMESH_H
#define DRAWABLEMESH_H

#ifdef COMMON_WITH_EIGEN
#include <Eigen/Core>
#endif

#include "drawable_object.h"

// From: https://blog.nobel-joergensen.com/2013/01/29/debugging-opengl-using-glgeterror/
void _check_gl_error(const char *file, int line);

///
/// Usage
/// [... some opengl calls]
/// glCheckError();
///
#define check_gl_error() _check_gl_error(__FILE__,__LINE__)

/**
 * @brief The DrawableMesh class
 * This is a non-instantiable class.
 * You can only inherit this class (protected constructors).
 */
class DrawableMesh : public DrawableObject{
    public:

        void init();

        // Implementation of the
        // DrawableObject interface
        virtual void draw() const = 0;
        virtual Pointd sceneCenter() const = 0;
        virtual double sceneRadius() const = 0;
        bool isVisible() const;

        // rendering options
        //
        void setWireframe(bool b);
        void setFlatShading();
        void setSmoothShading();
        void setPointsShading();
        void setWireframeColor(float r, float g, float b);
        void setWireframeWidth(float width);
        void setEnableVertexColor();
        void setEnableTriangleColor();
        void setVisible(bool b);

    protected:
        DrawableMesh();
        void draw(unsigned int nv, unsigned int nt, const double* pCoords, const int* pTriangles, const double* pVertexNormals, const float* pVertexColors, const double* pTriangleNormals, const float* pTriangleColors) const;
        void renderPass(unsigned int nv, unsigned int nt, const double* coords, const int* triangles, const double* vertexNormals, const float* vertexColors, const double* triangleNormals, const float* triangleColors) const;

        enum {
            DRAW_MESH        = 0b00000001,
            DRAW_POINTS      = 0b00000010,
            DRAW_FLAT        = 0b00000100,
            DRAW_SMOOTH      = 0b00001000,
            DRAW_WIREFRAME   = 0b00010000,
            DRAW_FACECOLOR   = 0b00100000,
            DRAW_VERTEXCOLOR = 0b01000000
        };

        int   drawMode;
        int   wireframeWidth;
        float wireframeColor[3];
};

#endif // DRAWABLEMESH_H
