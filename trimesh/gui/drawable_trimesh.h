/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#ifndef DRAWABLE_TRIMESH_H
#define DRAWABLE_TRIMESH_H

#include <vector>

#include "../../viewer/interfaces/drawable_object.h"
#include "../trimesh.h"

//using namespace std;

class DrawableTrimesh : public Trimesh<double> ,
                        public DrawableObject
{
    public:

        enum
        {
            DRAW_MESH        = 0x00000001,
            DRAW_POINTS      = 0x00000010,
            DRAW_FLAT        = 0x00000100,
            DRAW_SMOOTH      = 0x00001000,
            DRAW_WIREFRAME   = 0x00010000,
            DRAW_FACECOLOR   = 0x00100000,
            DRAW_VERTEXCOLOR = 0x01000000
        };

        DrawableTrimesh();
        DrawableTrimesh(const char *filename);

        void init();
        void clear();

        // Implementation of the
        // DrawableObject interface
        //
        void  draw()         const;
        Pointd sceneCenter() const;
        double sceneRadius() const;
        bool isVisible() const;
        void setVisible(bool b);

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
        void setVertexColor(float r, float g, float b);
        void setTriangleColor(float r, float g, float b);

        inline const float * vertexColor  (int vid) const { return &(vertexColors[vid*3]); }
        inline const float * triangleColor(int tid) const { return &(triangleColors[tid*3]); }

        inline void setVertexColor(int vid, float * color)
        {
            int vid_ptr = vid * 3;
            vertexColors[vid_ptr + 0] = color[0];
            vertexColors[vid_ptr + 1] = color[1];
            vertexColors[vid_ptr + 2] = color[2];
        }

        inline void setTriangleColor(int tid, float * color)
        {
            int tid_ptr = tid * 3;
            triangleColors[tid_ptr + 0] = color[0];
            triangleColors[tid_ptr + 1] = color[1];
            triangleColors[tid_ptr + 2] = color[2];
        }

    protected:

        void renderPass() const;

        int   drawMode;
        int   wireframeWidth;
        float wireframeColor[3];

        std::vector<float> vertexColors;
        std::vector<float> triangleColors;
};

#endif // DRAWABLE_TRIMESH_H
