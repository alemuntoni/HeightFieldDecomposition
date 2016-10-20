/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "drawable_dcel.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

DrawableDcel::DrawableDcel() : Dcel() {
    init();
}

/**
 * \~Italian
 * @brief Crea una DrawableDcel a partire dalla Dcel passata come parametro (di cui ne verrà fatta una copia)
 * @param[in] d: Dcel che verrà copiata e resa visualizzabile
 */
DrawableDcel::DrawableDcel(const Dcel &d) : Dcel(d) {
    update();
    init();
}

DrawableDcel::~DrawableDcel() {
}

/**
 * \~Italian
 * @brief Setta impostazioni di visualizzazione di default.
 */
void DrawableDcel::init() {
    drawMode          = DRAW_MESH | DRAW_SMOOTH |  DRAW_FACECOLOR;
    wireframeWidth    = 3;
    wireframeColor[0] = 0.1;
    wireframeColor[1] = 0.1;
    wireframeColor[2] = 0.1;
    wireframeColor[3] = 1;
    wireframe_colors.resize(colors.size());
    for (unsigned int i = 0; i < colors.size(); ++i)
        wireframe_colors[i] = 0.1;
    update();
}

/**
 * \~Italian
 * @brief esegue una clear della Dcel e della DrawableDcel
 */
void DrawableDcel::clear() {
    Dcel::clear();
    init();
    coords.clear();
    v_norm.clear();
    tris.clear();
    colors.clear();
}

/**
 * \~Italian
 * @brief Calcola e restituisce il centro della Dcel.
 * @return il punto rappresentante il centro della mesh.
 */
Pointd DrawableDcel::sceneCenter() const {
    BoundingBox bb = Dcel::getBoundingBox();
    Pointd p((bb.getMinX() + bb.getMaxX()) * 0.5, (bb.getMinY() + bb.getMaxY()) * 0.5, (bb.getMinZ() + bb.getMaxZ()) * 0.5);
    return p;
}

/**
 * \~Italian
 * @brief Calcola il raggio della Dcel.
 * @return Restituisce un valore rappresentante il raggio della mesh
 */
double DrawableDcel::sceneRadius() const {
    BoundingBox bb = Dcel::getBoundingBox();
    return (bb.getMin() - bb.getMax()).getLength();
}

/**
 * \~Italian
 * @brief Metodo che verifica se l'oggetto è visibile
 * @return true se la DrawableDcel è visibile, false altrimenti
 */
bool DrawableDcel::isVisible() const {
    return (drawMode & DRAW_MESH);
}

/**
 * \~Italian
 * @brief Metodo che permette di settare la visibilità dell'oggetto
 * @param[in] b: booleano indicante la vsibilità della DrawableDcel
 */
void DrawableDcel::setVisible(bool b) {
    if (b) drawMode |=  DRAW_MESH;
    else   drawMode &= ~DRAW_MESH;
}

/**
 * \~Italian
 * @brief DrawableDcel::update()
 * Aggiorna i vettori visualuizzati dalla draw() con le informazioni contenute nella Dcel.
 * Fa in modo che la mesh visualizzata sia effettivamente quella contenuta all'interno della struttura dati
 * Dcel. Deve essere chiamata ogni volta che è stata fatta una modifica nella Dcel e si vuole visualizzare tale
 * modifica.
 */
void DrawableDcel::update() {

    coords.clear();
    v_norm.clear();
    tris.clear();
    colors.clear();
    t_norm.clear();
    faces_wireframe.clear();
    coords.reserve(getNumberVertices()*3);
    v_norm.reserve(getNumberVertices()*3);
    tris.reserve(getNumberFaces()*3);
    colors.reserve(getNumberFaces()*3);
    t_norm.reserve(getNumberFaces()*3);
    std::map<int, int> v_ids;
    int vi = 0;

    for (ConstVertexIterator vit = vertexBegin(); vit != vertexEnd(); ++vit) {
        Pointd p = (*vit)->getCoordinate();
        Vec3 n = (*vit)->getNormal();
        coords.push_back(p.x());
        coords.push_back(p.y());
        coords.push_back(p.z());
        v_norm.push_back(n.x());
        v_norm.push_back(n.y());
        v_norm.push_back(n.z());

        v_ids[(*vit)->getId()] = vi;
        vi++;
    }
    #ifdef CGAL_DEFINED
    triangles_face.clear();
    for (FaceIterator fit = faceBegin(); fit != faceEnd(); ++fit) {
        for(Dcel::Face::IncidentHalfEdgeIterator heit = (*fit)->incidentHalfEdgeBegin(); heit != (*fit)->incidentHalfEdgeEnd(); heit++){
            unsigned int p1, p2;
            p1 = v_ids[(*heit)->getFromVertex()->getId()];
            p2 = v_ids[(*heit)->getToVertex()->getId()];
            std::pair<unsigned int, unsigned int> edge(p1,p2);
            std::vector<std::pair<unsigned int, unsigned int> >::iterator it = std::find(faces_wireframe.begin(), faces_wireframe.end(), edge);
            if(it==faces_wireframe.end())
                faces_wireframe.push_back(edge);
        }
        if ((*fit)->isTriangle()){
            Dcel::Face::ConstIncidentVertexIterator vit = (*fit)->incidentVertexBegin();
            tris.push_back(v_ids[(*vit)->getId()]);
            ++vit;
            tris.push_back(v_ids[(*vit)->getId()]);
            ++vit;
            tris.push_back(v_ids[(*vit)->getId()]);
            colors.push_back((*fit)->getColor().redF());
            colors.push_back((*fit)->getColor().greenF());
            colors.push_back((*fit)->getColor().blueF());
            t_norm.push_back((*fit)->getNormal().x());
            t_norm.push_back((*fit)->getNormal().y());
            t_norm.push_back((*fit)->getNormal().z());
            triangles_face.push_back((*fit)->getId());
        }
        else {
            //Si ottiene la triangolazione della faccia e si inseriscono i triangoli
            //prodotti nell'array tris.
            std::vector<std::array<const Dcel::Vertex*, 3> > face_triangles;
            (*fit)->getTriangulation(face_triangles);
            std::array<const Dcel::Vertex*, 3> t;
            for(unsigned int i = 0; i<face_triangles.size(); ++i){
                t = face_triangles[i];
                const Dcel::Vertex* v1 = t[0];
                const Dcel::Vertex* v2 = t[1];
                const Dcel::Vertex* v3 = t[2];
                tris.push_back(v_ids[v1->getId()]);
                tris.push_back(v_ids[v3->getId()]);
                tris.push_back(v_ids[v2->getId()]);
            }
            //Si crea una mappatura triangolo->faccia di appartenenza
            //Per ogni triangolo prodotto dalla triangolazione della faccia si aggiunge
            //un colore (composto da una tripla di valori)
            for(unsigned int ti = 0; ti < face_triangles.size(); ti++){
                triangles_face.push_back((*fit)->getId());
                colors.push_back((*fit)->getColor().redF());
                colors.push_back((*fit)->getColor().greenF());
                colors.push_back((*fit)->getColor().blueF());
                t_norm.push_back((*fit)->getNormal().x());
                t_norm.push_back((*fit)->getNormal().y());
                t_norm.push_back((*fit)->getNormal().z());
            }
            /***********************************************************************/
        }
    }
    #else
    for (ConstFaceIterator fit = faceBegin(); fit != faceEnd(); ++fit) {
        Dcel::Face::ConstIncidentVertexIterator vit = (*fit)->incidentVertexBegin();
        tris.push_back(v_ids[(*vit)->getId()]);
        ++vit;
        tris.push_back(v_ids[(*vit)->getId()]);
        ++vit;
        tris.push_back(v_ids[(*vit)->getId()]);
        colors.push_back((*fit)->getColor().redF());
        colors.push_back((*fit)->getColor().greenF());
        colors.push_back((*fit)->getColor().blueF());
        t_norm.push_back((*fit)->getNormal().x());
        t_norm.push_back((*fit)->getNormal().y());
        t_norm.push_back((*fit)->getNormal().z());
    }
    #endif
    wireframe_colors.resize(colors.size());
    for (unsigned int i = 0; i < colors.size(); ++i)
        wireframe_colors[i] = 0.1;
}

/**
 * \~Italian
 * @brief Rendering della mesh
 */
void DrawableDcel::renderPass() const {
    if (drawMode & DRAW_POINTS) {
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, coords.data());

        //glEnableClientState(GL_COLOR_ARRAY);
        //glColorPointer (3, GL_FLOAT, 0, colors.data());

        glDrawArrays(GL_POINTS, 0, getNumberVertices());

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
                glNormal3dv(&(t_norm[tid_ptr]));
                glVertex3dv(&(coords[vid0_ptr]));
                glNormal3dv(&(t_norm[tid_ptr]));
                glVertex3dv(&(coords[vid1_ptr]));
                glNormal3dv(&(t_norm[tid_ptr]));
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
        /*glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_DOUBLE, 0, coords.data());

        glLineWidth(1);

        glColor4fv(wireframeColor);

        glDrawElements(GL_TRIANGLES, tris.size(), GL_UNSIGNED_INT, tris.data());

        glDisableClientState(GL_VERTEX_ARRAY);*/
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
            glNormal3dv(&(t_norm[tid_ptr]));
            glVertex3dv(&(coords[vid0_ptr]));
            glNormal3dv(&(t_norm[tid_ptr]));
            glVertex3dv(&(coords[vid1_ptr]));
            glNormal3dv(&(t_norm[tid_ptr]));
            glVertex3dv(&(coords[vid2_ptr]));
            glEnd();
        }
    }

    if(drawMode & DRAW_FACES_WIREFRAME){

        for(unsigned int i=0; i<faces_wireframe.size(); i++){
            glLineWidth(1);
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            glVertex3dv(&(coords[3*(faces_wireframe[i].first)]));
            glVertex3dv(&(coords[3*(faces_wireframe[i].second)]));
            glEnd();
        }

    }
}

/**
 * \~Italian
 * @brief Metodo che si occupa di disegnare la dcel presente nei vettori della drawableDcel (aggiornati alla ultima update)
 * all'interno della canvas. è chiamato automaticamente dalla canvas ad ogni frame.
 */
void DrawableDcel::draw() const {
    if (drawMode & DRAW_MESH) {
        if (drawMode & DRAW_WIREFRAME) {
            if (drawMode & DRAW_POINTS) {
                glDisable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                glDepthRange(0.0, 1.0);
                renderPass();
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            }

            else if (drawMode & DRAW_FLAT) {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                glDepthRange(0.01, 1.0);
                renderPass();

                if (drawMode & DRAW_WIREFRAME) {
                    glDisable(GL_LIGHTING);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    glDepthRange(0.0, 1.0);
                    glDepthFunc(GL_LEQUAL);
                    renderPass();
                    glDepthFunc(GL_LESS);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                }
            }

            else if (drawMode & DRAW_SMOOTH) {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_SMOOTH);
                glDepthRange(0.01, 1.0);
                renderPass();

                if (drawMode & DRAW_WIREFRAME) {
                    glDisable(GL_LIGHTING);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                    glDepthRange(0.0, 1.0);
                    glDepthFunc(GL_LEQUAL);
                    renderPass();
                    glDepthFunc(GL_LESS);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                }
            }
        }else {
            if (drawMode & DRAW_POINTS) {
                glDisable(GL_LIGHTING);
                //glPointSize(10);
                renderPass();
            }

            else if (drawMode & DRAW_FLAT) {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_FLAT);
                renderPass();
            }

            else if (drawMode & DRAW_SMOOTH) {
                glEnable(GL_LIGHTING);
                glShadeModel(GL_SMOOTH);
                renderPass();
            }
        }
    }

}

void DrawableDcel::setWireframe(bool b) {
    if (b) drawMode |=  DRAW_WIREFRAME;
    else   drawMode &= ~DRAW_WIREFRAME;
}

void DrawableDcel::setFacesWireframe(bool b) {
    if (b) drawMode |=  DRAW_FACES_WIREFRAME;
    else   drawMode &= ~DRAW_FACES_WIREFRAME;
}

void DrawableDcel::setFlatShading() {
    drawMode |=  DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
    drawMode &= ~DRAW_POINTS;
}

void DrawableDcel::setSmoothShading() {
    drawMode |=  DRAW_SMOOTH;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_POINTS;
}

void DrawableDcel::setPointsShading() {
    drawMode |=  DRAW_POINTS;
    drawMode &= ~DRAW_FLAT;
    drawMode &= ~DRAW_SMOOTH;
}

void DrawableDcel::setEnableVertexColor() {
    drawMode |=  DRAW_VERTEXCOLOR;
    drawMode &= ~DRAW_FACECOLOR;
}

void DrawableDcel::setEnableTriangleColor() {
    drawMode |=  DRAW_FACECOLOR;
    drawMode &= ~DRAW_VERTEXCOLOR;
}

void DrawableDcel::setWireframeColor(float r, float g, float b) {
    wireframeColor[0] = r;
    wireframeColor[1] = g;
    wireframeColor[2] = b;
}

void DrawableDcel::setWireframeWidth(float width) {
    wireframeWidth = width;
}

void _check_gl_error(const char *file, int line)
{
  GLenum err (glGetError());

  while(err!=GL_NO_ERROR)
  {
    std::string error;

    switch(err)
    {
      case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
      case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
      case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
      case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
    }

    std::cerr << "GL_" << error.c_str() << " - " << file << ":" << line << std::endl;
    err = glGetError();
  }
}
