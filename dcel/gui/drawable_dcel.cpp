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

DrawableDcel::DrawableDcel() : Dcel(){
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
    DrawableMesh::init();
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
    vertexNormals.clear();
    vertexColors.clear();
    triangleNormals.clear();
    triangles.clear();
    triangleColors.clear();
}

void DrawableDcel::draw() const {
    DrawableMesh::draw(coords.size()/3, triangles.size()/3, coords.data(), triangles.data(), vertexNormals.data(), vertexColors.data(), triangleNormals.data(), triangleColors.data());
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
 * @brief DrawableDcel::update()
 * Aggiorna i vettori visualuizzati dalla draw() con le informazioni contenute nella Dcel.
 * Fa in modo che la mesh visualizzata sia effettivamente quella contenuta all'interno della struttura dati
 * Dcel. Deve essere chiamata ogni volta che è stata fatta una modifica nella Dcel e si vuole visualizzare tale
 * modifica.
 */
void DrawableDcel::update() {
    coords.clear();
    vertexNormals.clear();
    triangles.clear();
    triangleColors.clear();
    vertexColors.clear();
    triangleNormals.clear();
    faces_wireframe.clear();
    coords.reserve(getNumberVertices()*3);
    vertexNormals.reserve(getNumberVertices()*3);
    triangles.reserve(getNumberFaces()*3);
    triangleColors.reserve(getNumberFaces()*3);
    triangleNormals.reserve(getNumberFaces()*3);
    vertexColors.resize(getNumberVertices()*3,0.5);
    std::map<int, int> v_ids;
    int vi = 0;

    for (ConstVertexIterator vit = vertexBegin(); vit != vertexEnd(); ++vit) {
        Pointd p = (*vit)->getCoordinate();
        Vec3 n = (*vit)->getNormal();
        coords.push_back(p.x());
        coords.push_back(p.y());
        coords.push_back(p.z());
        vertexNormals.push_back(n.x());
        vertexNormals.push_back(n.y());
        vertexNormals.push_back(n.z());

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
            triangles.push_back(v_ids[(*vit)->getId()]);
            ++vit;
            triangles.push_back(v_ids[(*vit)->getId()]);
            ++vit;
            triangles.push_back(v_ids[(*vit)->getId()]);
            triangleColors.push_back((*fit)->getColor().redF());
            triangleColors.push_back((*fit)->getColor().greenF());
            triangleColors.push_back((*fit)->getColor().blueF());
            triangleNormals.push_back((*fit)->getNormal().x());
            triangleNormals.push_back((*fit)->getNormal().y());
            triangleNormals.push_back((*fit)->getNormal().z());
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
                triangles.push_back(v_ids[v1->getId()]);
                triangles.push_back(v_ids[v3->getId()]);
                triangles.push_back(v_ids[v2->getId()]);
            }
            //Si crea una mappatura triangolo->faccia di appartenenza
            //Per ogni triangolo prodotto dalla triangolazione della faccia si aggiunge
            //un colore (composto da una tripla di valori)
            for(unsigned int ti = 0; ti < face_triangles.size(); ti++){
                triangles_face.push_back((*fit)->getId());
                triangleColors.push_back((*fit)->getColor().redF());
                triangleColors.push_back((*fit)->getColor().greenF());
                triangleColors.push_back((*fit)->getColor().blueF());
                triangleNormals.push_back((*fit)->getNormal().x());
                triangleNormals.push_back((*fit)->getNormal().y());
                triangleNormals.push_back((*fit)->getNormal().z());
            }
            /***********************************************************************/
        }
    }
    #else
    for (ConstFaceIterator fit = faceBegin(); fit != faceEnd(); ++fit) {
        Dcel::Face::ConstIncidentVertexIterator vit = (*fit)->incidentVertexBegin();
        triangles.push_back(v_ids[(*vit)->getId()]);
        ++vit;
        triangles.push_back(v_ids[(*vit)->getId()]);
        ++vit;
        triangles.push_back(v_ids[(*vit)->getId()]);
        triangleColors.push_back((*fit)->getColor().redF());
        triangleColors.push_back((*fit)->getColor().greenF());
        triangleColors.push_back((*fit)->getColor().blueF());
        triangleNormals.push_back((*fit)->getNormal().x());
        triangleNormals.push_back((*fit)->getNormal().y());
        triangleNormals.push_back((*fit)->getNormal().z());
    }
    #endif
}

/**
 * \~Italian
 * @brief Rendering della mesh
 */
void DrawableDcel::renderPass(unsigned int nv, unsigned int nt, const double* coords, const int* triangles, const double* vertexNormals, const float* vertexColors, const double* triangleNormals, const float* triangleColors) const {
    DrawableMesh::renderPass(nv, nt, coords, triangles, vertexNormals, vertexColors, triangleNormals, triangleColors);

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

void DrawableDcel::setFacesWireframe(bool b) {
    if (b) drawMode |=  DRAW_FACES_WIREFRAME;
    else   drawMode &= ~DRAW_FACES_WIREFRAME;
}
