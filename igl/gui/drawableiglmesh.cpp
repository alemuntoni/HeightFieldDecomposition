#include "drawableiglmesh.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

DrawableIGLMesh::DrawableIGLMesh() {
    init();
}

DrawableIGLMesh::DrawableIGLMesh(const SimpleIGLMesh& m) : IGLMesh(m) {
    init();
}

DrawableIGLMesh::DrawableIGLMesh(const IGLMesh& m) : IGLMesh(m){
    init();
}

DrawableIGLMesh::~DrawableIGLMesh(){
}

void DrawableIGLMesh::init() {
    drawMode          = DRAW_MESH | DRAW_SMOOTH | DRAW_FACECOLOR;
}

void DrawableIGLMesh::draw() const {
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
            //check_gl_error();
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

Pointd DrawableIGLMesh::sceneCenter() const {
    return Pointd((BBmin(0) + BBmax(0))/2, (BBmin(1) + BBmax(1))/2, (BBmin(2) + BBmax(2))/2);
}

double DrawableIGLMesh::sceneRadius() const {
    return Pointd(BBmin(0), BBmin(1), BBmin(2)).dist(Pointd(BBmax(0), BBmax(1), BBmax(2)));
}

bool DrawableIGLMesh::isVisible() const {
    return (drawMode & DRAW_MESH);
}

void DrawableIGLMesh::setVisible(bool b) {
    if (b) drawMode |=  DRAW_MESH;
    else   drawMode &= ~DRAW_MESH;
}

void DrawableIGLMesh::setWireframe(bool b) {
    if (b) drawMode |=  DRAW_WIREFRAME;
    else   drawMode &= ~DRAW_WIREFRAME;
}

void DrawableIGLMesh::setFlatShading() {
    drawMode |=  DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
    drawMode &= ~DRAW_POINTS;
}

void DrawableIGLMesh::setSmoothShading() {
    drawMode |=  DRAW_SMOOTH;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_POINTS;
}

void DrawableIGLMesh::setPointShading() {
    drawMode |=  DRAW_POINTS;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
}

void DrawableIGLMesh::renderPass() const {
    if (drawMode & DRAW_POINTS) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, V.data());

        glDrawArrays(GL_POINTS, 0, V.rows());

        glDisableClientState(GL_VERTEX_ARRAY);
    }
    else if (drawMode & DRAW_SMOOTH || drawMode & DRAW_FLAT) {
        // Old fashioned, verbose and slow rendering.
        //
        if (drawMode & DRAW_FACECOLOR) {

            int n_tris = F.rows();
            for(int tid=0; tid<n_tris; ++tid) {
                int vid0     = F(tid, 0);
                int vid1     = F(tid, 1);
                int vid2     = F(tid, 2);
                glBegin(GL_TRIANGLES);
                glColor3dv((C.row(tid).data()));
                glNormal3dv((NF.row(tid).data()));
                glVertex3dv((V.row(vid0).data()));
                glNormal3dv((NF.row(tid).data()));
                glVertex3dv((V.row(vid1).data()));
                glNormal3dv((NF.row(tid).data()));
                glVertex3dv((V.row(vid2).data()));
                glEnd();
            }
        }
        else {

            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, 0, V.data());

            glEnableClientState(GL_NORMAL_ARRAY);
            glNormalPointer(GL_DOUBLE, 0, NV.data());

            glDrawElements(GL_TRIANGLES, F.rows()*3, GL_UNSIGNED_INT, F.data());

            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
        }
    }

    if (drawMode & DRAW_WIREFRAME) {
        float wirframecolor[3] = {0.1, 0.1, 0.1};
        int n_tris = F.rows();
        for(int tid=0; tid<n_tris; ++tid) {
            int vid0     = F(tid, 0);
            int vid1     = F(tid, 1);
            int vid2     = F(tid, 2);

            glBegin(GL_TRIANGLES);

            glColor3fv(wirframecolor);
            glNormal3dv((NF.row(tid).data()));
            glVertex3dv((V.row(vid0).data()));
            glNormal3dv((NF.row(tid).data()));
            glVertex3dv((V.row(vid1).data()));
            glNormal3dv((NF.row(tid).data()));
            glVertex3dv((V.row(vid2).data()));
            glEnd();
        }
    }

}
