/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "pickable_dcel.h"

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

PickableDcel::PickableDcel(){
}

PickableDcel::PickableDcel(const Dcel& d) : DrawableDcel(d) {
}

/********************************Andreas*********************************/
/**
 * \~Italian
 * @brief DrawableDcel::drawWithNames Metodo che si occupa di disegnare le facce assegnando a esse un identificativo
 * riconoscibile nella postSelection (classe glCanvas) in modo da poterne effettuare il picking.
 */
void PickableDcel::drawWithNames() const{

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_DOUBLE, 0, coords.data());

    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_DOUBLE, 0, v_norm.data());

    //Per ogni faccia si effettua il push dell'id associato e quindi la si disegna
    for (ConstFaceIterator fit = faceBegin(); fit != faceEnd(); ++fit) {
        glPushName((*fit)->getId());
        drawFace((*fit));
        glPopName();
    }

    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
}


/**
 * \~Italian
 * @brief DrawableDcel::drawFace Metodo che si occupa di effettuare il rendering di una faccia
 * @param f La faccia da renderizzare
 */
void PickableDcel::drawFace(const Face* f) const{

    std::vector<int> face_triangles = obtainFaceTriangles(f);

    glDrawElements(GL_TRIANGLES, face_triangles.size(), GL_UNSIGNED_INT, face_triangles.data());

}

/**
 * \~Italian
 * @brief DrawableDcel::obtainFaceTriangles Metodo che si occupa della ricerca dei triangoli
 * appartenenti a una data faccia all'interno della lista tris
 * @param f la faccia a cui apparterranno i triangoli
 * @return una lista di triangoli (da interpretare nello stesso modo di tris)
 */
std::vector<int> PickableDcel::obtainFaceTriangles(const Face* f) const{
    std::vector<int> face_triangles;

    //Ricerca dei triangoli appartenenti alla faccia
    for(unsigned int i=0; i<triangles_face.size(); i++)
        if(triangles_face.at(i)==f->getId()){
            face_triangles.push_back(tris.at(i*3));
            face_triangles.push_back(tris.at(i*3+1));
            face_triangles.push_back(tris.at(i*3+2));
        }

    return face_triangles;
}
/************************************************************************/
