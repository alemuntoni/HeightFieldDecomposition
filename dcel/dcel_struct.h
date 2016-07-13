/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_STRUCT_H
#define DCEL_STRUCT_H

#include <boost/tokenizer.hpp>
#include <fstream>
#include <sstream>
#include <QFileInfo>
#include <map>
#include <set>
#include <array>
#include <omp.h>

#include "../common/bounding_box.h"

#ifdef CGAL_DEFINED
#include "../cgal/cgalinterface.h"
#endif

/**
 * \~Italian
 * @class Dcel
 * @brief Struttura dati Double Conntected Edge List.
 *
 * La classe Dcel è composta principalmente da tre liste di:
 * - Dcel::Vertex*
 * - Dcel::HalfEdge*
 * - Dcel::Face*
 *
 *
 * La struttura è Half-Edge based: tutte le relazioni di adiacenza e incidenza locali di un half edge sono contenuti
 * all'interno della classe Dcel::HalfEdge, ci si può accedere mediante semplici operazioni di get e si possono modificare
 * mediante semplici operazioni di set. Dcel::Vertex e Dcel::Face hanno invece dei riferimenti ad alcuni degli half edge incidenti
 * (nel caso del Dcel::Vertex solo uno, nel caso della Dcel::Face uno esterno e uno per ogni buco), e le restanti adiacenze/incidenze
 * sono reperibili mediante le operazioni messe a disposizione da Dcel::HalfEdge.
 * La Dcel mette a disposizione degli iteratori per poter accedere a tutti i Vertici, Half Edge e Facce contenunti in essa
 * e non permette di poter accedere direttamente ai vettori che li contengono, per poter preservare quelli che sono i dettagli
 * implementativi che permettono al programmatore di utilizzare una struttura dati il più sicura possibile.
 *
 * Si può ciclare, per esempio, su tutti i vertici contenuti nella Dcel con l'iteratore sui vertici:
 *
 * \code{.cpp}
 * for (Dcel::VertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
 *     Dcel::Vertex* v = *vit;
 *     // operazioni su v
 * }
 * \endcode
 *
 * La stessa cosa si può fare con Dcel::HalfEdgeIterator e Dcel::FaceIterator. Per ciclare su const Dcel, invece, ci sono gli iteratori
 * Dcel::ConstVertexIterator, Dcel::ConstHalfEdgeIterator e Dcel::ConstFaceIterator.
 * La classi Dcel::Vertex e Dcel::Face mettono a disposizione altri iteratori (vedere le relative pagine), tuttavia i loro iteratori sono
 * completamente diversi rispetto a questi. Gli iteratori della Dcel servono per ciclare su tutti gli elementi contenuti nella Dcel, senza
 * tenere conto di nessuna relazione di adiacenza o incidenza.
 *
 * Gli iteratori presenti in Dcel::Vertex e Dcel::Face invece servono solo ed esclusivamente a visitare tutte le componenti adiacenti o incidenti
 * ad un Dcel::Vertex o una Dcel::Face. Non vi sono iteratori nella classe Dcel::HalfEdge in quanto tutte le relazioni di incidenza/adiacenza
 * sono ottenibili mediante operazioni di get.
 */

class Dcel : public SerializableObject {

    public:

        /*********************
        * Associated Classes *
        **********************/

        class Vertex;
        class Face;
        class HalfEdge;

        /************
        * Iterators *
        *************/

        class VertexIterator;
        class ConstVertexIterator;
        class HalfEdgeIterator;
        class ConstHalfEdgeIterator;
        class FaceIterator;
        class ConstFaceIterator;

        /***************
        * Constructors *
        ****************/

        Dcel();
        Dcel(const Dcel &dcel);
        ~Dcel();

        /************************
        * Public Inline Methods *
        *************************/

        unsigned int getNumberVertices()        const;
        unsigned int getNumberHalfEdges()       const;
        unsigned int getNumberFaces()           const;
        ConstVertexIterator vertexBegin()       const;
        ConstVertexIterator vertexEnd()         const;
        ConstHalfEdgeIterator halfEdgeBegin()   const;
        ConstHalfEdgeIterator halfEdgeEnd()     const;
        ConstFaceIterator faceBegin()           const;
        ConstFaceIterator faceEnd()             const;

        VertexIterator vertexBegin();
        VertexIterator vertexEnd();
        HalfEdgeIterator halfEdgeBegin();
        HalfEdgeIterator halfEdgeEnd();
        FaceIterator faceBegin();
        FaceIterator faceEnd();

        /*****************
        * Public Methods *
        ******************/

        const Vertex* getVertex(unsigned int idVertex)               const;
        const HalfEdge* getHalfEdge(unsigned int idHalfEdge)         const;
        const Face* getFace(unsigned int idFace)                     const;
        BoundingBox getBoundingBox()                        const;
        bool isTriangleMesh()                               const;
        void saveOnObjFile(std::string fileNameObj)     const;
        void saveOnPlyFile(std::string fileNamePly)     const;
        void saveOnDcelFile(std::string fileNameDcel)   const;
        void serialize(std::ofstream& binaryFile) const;

        Vertex* getVertex(unsigned int idVertex);
        HalfEdge* getHalfEdge(unsigned int idHalfEdge);
        Face* getFace(unsigned int idFace);
        Vertex* addVertex();
        Vertex* addVertex(const Vertex& v);
        Vertex* addVertex(const Pointd& p);
        HalfEdge* addHalfEdge();
        HalfEdge* addHalfEdge(const HalfEdge& he);
        Face* addFace();
        Face* addFace(const Face& f);
        bool deleteVertex (Vertex* v);
        VertexIterator deleteVertex(const VertexIterator& vit);
        bool deleteHalfEdge (HalfEdge* he);
        HalfEdgeIterator deleteHalfEdge(const HalfEdgeIterator& heit);
        bool deleteFace (Face* f);
        FaceIterator deleteFace(const FaceIterator& fit);
        void updateFaceNormals();
        void updateVertexNormals();
        BoundingBox updateBoundingBox();
        void scale(const BoundingBox &newBoundingBox);
        void rotate(const Eigen::Matrix3d& matrix, const Pointd& centroid = Pointd());
        void rotate(double matrix[3][3], const Pointd& centroid = Pointd());
        void recalculateIds();
        void resetFaceColors();
        void clear();
        #ifdef CGAL_DEFINED
        unsigned int triangulateFace(Dcel::Face* f);
        void triangulate();
        #endif
        std::string loadFromObjFile(const std::string& filename, bool regular = true);
        std::string loadFromPlyFile(const std::string& filename, bool regular = true);
        std::string loadFromDcelFile(const std::string& filename);
        void deserialize(std::ifstream& binaryFile);
        std::string loadFromOldDcelFile(const std::string& filename);
        std::string loadFromOldOldDcelFile(const std::string& filename);
        Dcel& operator= (const Dcel& dcel);

    protected:

        /*************
        * Attributes *
        **************/

        std::vector<Vertex* >   vertices;       /**< \~Italian @brief Lista di vertici della Dcel. */
        std::vector<HalfEdge* > halfEdges;      /**< \~Italian @brief Lista di half edge della Dcel. */
        std::vector<Face* >     faces;          /**< \~Italian @brief Lista di facce della Dcel. */
        std::set<int>           unusedVids;     /**< \~Italian @brief Insieme degli id dei vertici non utilizzati (nullptr). */
        std::set<int>           unusedHeids;    /**< \~Italian @brief Insieme degli id degli half edge non utilizzati (nullptr). */
        std::set<int>           unusedFids;     /**< \~Italian @brief Insieme degli id delle facce non utilizzati (nullptr). */
        unsigned int            nVertices;      /**< \~Italian @brief Prossimo id del vertice. */
        unsigned int            nHalfEdges;     /**< \~Italian @brief Prossimo id dell'half edge. */
        unsigned int            nFaces;         /**< \~Italian @brief Prossimo id della faccia. */
        BoundingBox             boundingBox;    /**< \~Italian @brief Bounding box della mesh. */

        /******************
        * Private Methods *
        *******************/

        Vertex* addVertex(int id);
        HalfEdge* addHalfEdge(int id);
        Face* addFace(int id);

        std::vector<const Vertex*> makeSingleBorder(const Face *f)     const;

        void copyFrom(const Dcel &d);

};

#include "dcel_iterators.h"

/**
 * \~Italian
 * @brief Restituisce il numero di vertici presenti nella Dcel.
 * @return Numero di vertici.
 */
inline unsigned int Dcel::getNumberVertices () const {
    return nVertices;
}

/**
 * \~Italian
 * @brief Restituisce il numero di half edge presenti nella Dcel.
 * @return Numero di half edge.
 */
inline unsigned int Dcel::getNumberHalfEdges () const {
    return nHalfEdges;
}

/**
 * \~Italian
 * @brief Restituisce il numero di facce presenti nella Dcel.
 * @return Numero di facce.
 */
inline unsigned int Dcel::getNumberFaces () const {
    return nFaces;
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::ConstVertexIterator
 * @return Un iteratore che punta al primo vertice della Dcel
 */
inline Dcel::ConstVertexIterator Dcel::vertexBegin() const {
    unsigned int i = 0;
    while (i < vertices.size() && vertices[i] == nullptr) ++i;
    return ConstVertexIterator(i, vertices);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::ConstVertexIterator
 * @return Un iteratore che punta all'ultimo vertice della Dcel
 */
inline Dcel::ConstVertexIterator Dcel::vertexEnd() const {
    return ConstVertexIterator(vertices.size(), vertices);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::ConstHalfEdgeIterator
 * @return Un iteratore che punta al primo half edge della Dcel
 */
inline Dcel::ConstHalfEdgeIterator Dcel::halfEdgeBegin() const {
    unsigned int i = 0;
    while (i < halfEdges.size() && halfEdges[i] == nullptr) ++i;
    return ConstHalfEdgeIterator(i, halfEdges);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::ConstHalfEdgeIterator
 * @return Un iteratore che punta all'ultimo half edge della Dcel
 */
inline Dcel::ConstHalfEdgeIterator Dcel::halfEdgeEnd() const {
    return ConstHalfEdgeIterator(halfEdges.size(), halfEdges);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::ConstFaceIterator
 * @return Un iteratore che punta alla prima faccia della Dcel
 */
inline Dcel::ConstFaceIterator Dcel::faceBegin() const {
    unsigned int i = 0;
    while (i < faces.size() && faces[i] == nullptr) ++i;
    return ConstFaceIterator(i, faces);
}

/**
 * @brief Funzione di finalizzazione di Dcel::ConstFaceIterator
 * @return Un iteratore che punta all'ultima faccia della Dcel
 */
inline Dcel::ConstFaceIterator Dcel::faceEnd() const {
    return ConstFaceIterator(faces.size(), faces);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::VertexIterator
 * @return Un iteratore che punta al primo vertice della Dcel
 */
inline Dcel::VertexIterator Dcel::vertexBegin() {
    unsigned int i = 0;
    while (i < vertices.size() && vertices[i] == nullptr) ++i;
    return VertexIterator(i, vertices);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::VertexIterator
 * @return Un iteratore che punta all'ultimo vertice della Dcel
 */
inline Dcel::VertexIterator Dcel::vertexEnd() {
    return VertexIterator(vertices.size(), vertices);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::HalfEdgeIterator
 * @return Un iteratore che punta al primo half edge della Dcel
 */
inline Dcel::HalfEdgeIterator Dcel::halfEdgeBegin() {
    unsigned int i = 0;
    while (i < halfEdges.size() && halfEdges[i] == nullptr) ++i;
    return HalfEdgeIterator(i, halfEdges);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::HalfEdgeIterator
 * @return Un iteratore che punta all'ultimo half edge della Dcel
 */
inline Dcel::HalfEdgeIterator Dcel::halfEdgeEnd() {
    return HalfEdgeIterator(halfEdges.size(), halfEdges);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::FaceIterator
 * @return Un iteratore che punta alla prima faccia della Dcel
 */
inline Dcel::FaceIterator Dcel::faceBegin() {
    unsigned int i = 0;
    while (i < faces.size() && faces[i] == nullptr) ++i;
    return FaceIterator(i, faces);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::FaceIterator
 * @return Un iteratore che punta all'ultima faccia della Dcel
 */
inline Dcel::FaceIterator Dcel::faceEnd() {
    return FaceIterator(faces.size(), faces);
}

#endif // DCEL_STRUCT_H
