#include "drawableiglmesh.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

DrawableIGLMesh::DrawableIGLMesh() {
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

void DrawableIGLMesh::update() {
    coords.clear();
    v_norm.clear();
    tris.clear();
    colors.clear();
    coords.resize(V.rows()*3);
    v_norm.resize(V.rows()*3);
    tris.resize(F.rows()*3);
    colors.resize(F.rows()*3);
    wireframe_colors.resize(F.rows()*3);
    for (unsigned int i = 0; i < V.rows(); i++){
        coords[i*3] = V(i,0);
        coords[i*3+1] = V(i,1);
        coords[i*3+2] = V(i,2);
        v_norm[i*3] = NV(i,0);
        v_norm[i*3+1] = NV(i,1);
        v_norm[i*3+2] = NV(i,2);
    }
    for (unsigned int i = 0; i < F.rows(); i++){
        tris[i*3] = F(i,0);
        tris[i*3+1] = F(i,1);
        tris[i*3+2] = F(i,2);
        colors[i*3] = C(i,0);
        colors[i*3+1] = C(i,1);
        colors[i*3+2] = C(i,2);
        wireframe_colors[i*3] = 0.1;
        wireframe_colors[i*3+1] = 0.1;
        wireframe_colors[i*3+2] = 0.1;
    }

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

void DrawableIGLMesh::setPointsShading() {
    drawMode |=  DRAW_POINTS;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
}

void DrawableIGLMesh::renderPass() const {
    if (drawMode & DRAW_POINTS) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, coords.data());

        glDrawArrays(GL_POINTS, 0, V.rows());

        glDisableClientState(GL_VERTEX_ARRAY);
    }
    else if (drawMode & DRAW_SMOOTH || drawMode & DRAW_FLAT) {
        // Old fashioned, verbose and slow rendering.
        //
        if (drawMode & DRAW_FACECOLOR) {

            int n_tris = tris.size()/3;
            for(int tid=0; tid<n_tris; ++tid) {
                int tid_ptr  = 3 * tid;
                int vid0     = tris[tid_ptr + 0];
                int vid1     = tris[tid_ptr + 1];
                int vid2     = tris[tid_ptr + 2];
                int vid0_ptr = 3 * vid0;
                int vid1_ptr = 3 * vid1;

                int vid2_ptr = 3 * vid2;

                glBegin(GL_TRIANGLES);
                glColor3fv(&(colors[tid_ptr]));
                glNormal3dv(&(v_norm[vid0_ptr]));
                glVertex3dv(&(coords[vid0_ptr]));
                glNormal3dv(&(v_norm[vid1_ptr]));
                glVertex3dv(&(coords[vid1_ptr]));
                glNormal3dv(&(v_norm[vid2_ptr]));
                glVertex3dv(&(coords[vid2_ptr]));
                glEnd();
            }
        }
        else {

            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_DOUBLE, 0, coords.data());

            glEnableClientState(GL_NORMAL_ARRAY);
            glNormalPointer(GL_DOUBLE, 0, v_norm.data());

            glDrawElements(GL_TRIANGLES, tris.size(), GL_UNSIGNED_INT, tris.data());

            glDisableClientState(GL_NORMAL_ARRAY);
            glDisableClientState(GL_VERTEX_ARRAY);
        }
    }

    if (drawMode & DRAW_WIREFRAME) {
        int n_tris = tris.size()/3;
        for(int tid=0; tid<n_tris; ++tid) {
            int tid_ptr  = 3 * tid;
            int vid0     = tris[tid_ptr + 0];
            int vid1     = tris[tid_ptr + 1];
            int vid2     = tris[tid_ptr + 2];
            int vid0_ptr = 3 * vid0;
            int vid1_ptr = 3 * vid1;

            int vid2_ptr = 3 * vid2;


            glBegin(GL_TRIANGLES);

            glColor3fv(&(wireframe_colors[tid_ptr]));
            glNormal3dv(&(v_norm[vid0_ptr]));
            glVertex3dv(&(coords[vid0_ptr]));
            glNormal3dv(&(v_norm[vid1_ptr]));
            glVertex3dv(&(coords[vid1_ptr]));
            glNormal3dv(&(v_norm[vid2_ptr]));
            glVertex3dv(&(coords[vid2_ptr]));
            glEnd();
        }
    }

}
