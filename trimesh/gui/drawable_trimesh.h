/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#ifndef DRAWABLE_TRIMESH_H
#define DRAWABLE_TRIMESH_H

#include <vector>

#include "../../viewer/interfaces/drawable_mesh.h"
#include "../trimesh.h"

class DrawableTrimesh : public Trimesh<double> ,
                        public DrawableMesh
{
    public:

        DrawableTrimesh();
        DrawableTrimesh(const Trimesh<double> &t);
        DrawableTrimesh(const char *filename);

        void init();
        void clear();

        // Implementation of the
        // DrawableObject interface
        //
        void draw() const;
        Pointd sceneCenter() const;
        double sceneRadius() const;

        // rendering options
        //
        void setVertexColor(float r, float g, float b);
        void setTriangleColor(float r, float g, float b);

        inline const float * vertexColor  (int vid) const { return &(vertexColors[vid*3]); }
        inline const float * triangleColor(int tid) const { return &(triangleColors[tid*3]); }

        inline void setVertexColor(int vid, float * color) {
            int vid_ptr = vid * 3;
            vertexColors[vid_ptr + 0] = color[0];
            vertexColors[vid_ptr + 1] = color[1];
            vertexColors[vid_ptr + 2] = color[2];
        }

        inline void setTriangleColor(int tid, float * color) {
            int tid_ptr = tid * 3;
            triangleColors[tid_ptr + 0] = color[0];
            triangleColors[tid_ptr + 1] = color[1];
            triangleColors[tid_ptr + 2] = color[2];
        }

    protected:

        std::vector<float> vertexColors;
        std::vector<float> triangleColors;
};

#endif // DRAWABLE_TRIMESH_H
