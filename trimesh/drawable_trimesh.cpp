/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#include "drawable_trimesh.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

DrawableTrimesh::DrawableTrimesh() : Trimesh<double>()
{
    init();
}

DrawableTrimesh::DrawableTrimesh(const char *filename) : Trimesh<double>(filename)
{
    init();
}

void DrawableTrimesh::init()
{
    //type               = TRIMESH;
    drawMode          = DRAW_MESH | DRAW_SMOOTH | DRAW_VERTEXCOLOR;
    wireframeWidth    = 1;
    wireframeColor[0] = 0.1;
    wireframeColor[1] = 0.1;
    wireframeColor[2] = 0.1;

    setTriangleColor(0.1, 0.8, 0.1);
    setVertexColor(0.1, 0.8, 0.1);
}

void DrawableTrimesh::clear()
{
    Trimesh<double>::clear();
    vertexColors.clear();
    triangleColors.clear();
}

Pointd DrawableTrimesh::sceneCenter() const
{
    Pointd c = bbox.center();
    return Pointd(c.x(), c.y(), c.z());
}

double DrawableTrimesh::sceneRadius() const {
    return bbox.diag();
}

bool DrawableTrimesh::isVisible() const
{
    return (drawMode & DRAW_MESH);
}

void DrawableTrimesh::setVisible(bool b)
{
    if (b) drawMode |=  DRAW_MESH;
    else   drawMode &= ~DRAW_MESH;
}

void DrawableTrimesh::renderPass() const
{
    if (drawMode & DRAW_POINTS)
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, coords.data());

        glEnableClientState(GL_COLOR_ARRAY);
        glColorPointer (3, GL_FLOAT, 0, vertexColors.data());

        glDrawArrays(GL_POINTS, 0, numVertices());

        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    else if (drawMode & DRAW_SMOOTH || drawMode & DRAW_FLAT)
    {
        // Old fashioned, verbose and slow rendering.
        //
        if (drawMode & DRAW_FACECOLOR)
        {
            int n_tris = tris.size()/3;
            for(int tid=0; tid<n_tris; ++tid)
            {
                int tid_ptr  = 3 * tid;
                int vid0     = tris[tid_ptr + 0];
                int vid1     = tris[tid_ptr + 1];
                int vid2     = tris[tid_ptr + 2];
                int vid0_ptr = 3 * vid0;
                int vid1_ptr = 3 * vid1;
                int vid2_ptr = 3 * vid2;

                glBegin(GL_TRIANGLES);
                glColor3fv(&(triangleColors[tid_ptr]));
                glNormal3dv(&(vertexNormals[vid0_ptr]));
                glVertex3dv(&(coords[vid0_ptr]));
                glNormal3dv(&(vertexNormals[vid1_ptr]));
                glVertex3dv(&(coords[vid1_ptr]));
                glNormal3dv(&(vertexNormals[vid2_ptr]));
                glVertex3dv(&(coords[vid2_ptr]));
                glEnd();
            }
        }
        else
        {
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, 0, coords.data());

            glEnableClientState(GL_NORMAL_ARRAY);
            glNormalPointer(GL_DOUBLE, 0, vertexNormals.data());

            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(3, GL_FLOAT, 0, vertexColors.data());

            glDrawElements(GL_TRIANGLES, tris.size(), GL_UNSIGNED_INT, tris.data());

            glDisableClientState(GL_COLOR_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
        }
    }

    if (drawMode & DRAW_WIREFRAME)
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, coords.data());

        glLineWidth(wireframeWidth);
        glColor4fv(wireframeColor);

        glDrawElements(GL_TRIANGLES, tris.size(), GL_UNSIGNED_INT, tris.data());

        glDisableClientState(GL_VERTEX_ARRAY);
    }
}


void DrawableTrimesh::draw() const
{
    if (drawMode & DRAW_MESH)
    {
        if (drawMode & DRAW_WIREFRAME)
        {
            if (drawMode & DRAW_POINTS)
            {
                glDisable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDepthRange(0.0, 1.0);
                renderPass();
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            else

            if (drawMode & DRAW_FLAT)
            {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                glDepthRange(0.01, 1.0);
                renderPass();

                glDisable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDepthRange(0.0, 1.0);
                glDepthFunc(GL_LEQUAL);
                renderPass();
                glDepthFunc(GL_LESS);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            else

            if (drawMode & DRAW_SMOOTH)
            {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_SMOOTH);
                glDepthRange(0.01, 1.0);
                renderPass();

                glDisable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDepthRange(0.0, 1.0);
                glDepthFunc(GL_LEQUAL);
                renderPass();
                glDepthFunc(GL_LESS);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }
        }

        else

        {
            if (drawMode & DRAW_POINTS)
            {
                glDisable(GL_LIGHTING);
                renderPass();
            }

            else

            if (drawMode & DRAW_FLAT)
            {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                renderPass();
            }

            else

            if (drawMode & DRAW_SMOOTH)
            {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_SMOOTH);
                renderPass();
            }
        }
    }
}

void DrawableTrimesh::setWireframe(bool b)
{
    if (b) drawMode |=  DRAW_WIREFRAME;
    else   drawMode &= ~DRAW_WIREFRAME;
}

void DrawableTrimesh::setFlatShading()
{
    drawMode |=  DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
    drawMode &= ~DRAW_POINTS;
}

void DrawableTrimesh::setSmoothShading()
{
    drawMode |=  DRAW_SMOOTH;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_POINTS;
}

void DrawableTrimesh::setPointsShading()
{
    drawMode |=  DRAW_POINTS;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
}

void DrawableTrimesh::setEnableVertexColor()
{
    drawMode |=  DRAW_VERTEXCOLOR;
    drawMode &= ~DRAW_FACECOLOR;
}

void DrawableTrimesh::setEnableTriangleColor()
{
    drawMode |=  DRAW_FACECOLOR;
    drawMode &= ~DRAW_VERTEXCOLOR;
}

void DrawableTrimesh::setWireframeColor(float r, float g, float b)
{
    wireframeColor[0] = r;
    wireframeColor[1] = g;
    wireframeColor[2] = b;
}

void DrawableTrimesh::setWireframeWidth(float width)
{
    wireframeWidth = width;
}

void DrawableTrimesh::setVertexColor(float r, float g, float b)
{
    vertexColors.resize(numVertices()*3);
    for(int i=0; i<(int)vertexColors.size(); i+=3)
    {
        vertexColors[i + 0] = r;
        vertexColors[i + 1] = g;
        vertexColors[i + 2] = b;
    }
}

void DrawableTrimesh::setTriangleColor(float r, float g, float b)
{
    triangleColors.resize(numTriangles()*3);
    for(int i=0; i<(int)triangleColors.size(); i+=3)
    {
        triangleColors[i + 0] = r;
        triangleColors[i + 1] = g;
        triangleColors[i + 2] = b;
    }
}
