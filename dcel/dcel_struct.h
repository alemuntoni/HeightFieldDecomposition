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

        class ConstVertexRangeBasedIterator {
                friend class Dcel;
            public:
                ConstVertexIterator begin() const;
                ConstVertexIterator end() const;
            private:
                ConstVertexRangeBasedIterator(const Dcel *d) : d(d) {}
                const Dcel *d;
        };

        class ConstHalfEdgeRangeBasedIterator {
                friend class Dcel;
            public:
                ConstHalfEdgeIterator begin() const;
                ConstHalfEdgeIterator end() const;
            private:
                ConstHalfEdgeRangeBasedIterator(const Dcel *d) : d(d) {}
                const Dcel *d;
        };

        class ConstFaceRangeBasedIterator {
                friend class Dcel;
            public:
                ConstFaceIterator begin() const;
                ConstFaceIterator end() const;
            private:
                ConstFaceRangeBasedIterator(const Dcel *d) : d(d) {}
                const Dcel *d;
        };

        class VertexRangeBasedIterator {
                friend class Dcel;
            public:
                VertexIterator begin();
                VertexIterator end();
            private:
                VertexRangeBasedIterator(Dcel *d) : d(d) {}
                Dcel *d;
        };

        class HalfEdgeRangeBasedIterator {
                friend class Dcel;
            public:
                HalfEdgeIterator begin();
                HalfEdgeIterator end();
            private:
                HalfEdgeRangeBasedIterator(Dcel *d) : d(d) {}
                Dcel *d;
        };

        class FaceRangeBasedIterator {
                friend class Dcel;
            public:
                FaceIterator begin();
                FaceIterator end();
            private:
                FaceRangeBasedIterator(Dcel *d) : d(d) {}
                Dcel *d;
        };

        const ConstVertexRangeBasedIterator vertexIterator() const;
        VertexRangeBasedIterator vertexIterator();

        const ConstHalfEdgeRangeBasedIterator halfEdgeIterator() const;
        HalfEdgeRangeBasedIterator halfEdgeIterator();

        const ConstFaceRangeBasedIterator faceIterator() const;
        FaceRangeBasedIterator faceIterator();

        /*****************
        * Public Methods *
        ******************/

        const Vertex* getVertex(unsigned int idVertex)          const;
        const HalfEdge* getHalfEdge(unsigned int idHalfEdge)    const;
        const Face* getFace(unsigned int idFace)                const;
        BoundingBox getBoundingBox()                            const;
        bool isTriangleMesh()                                   const;
        double getSurfaceArea()                                 const;
        void saveOnObjFile(std::string fileNameObj)             const;
        void saveOnPlyFile(std::string fileNamePly)             const;
        void saveOnDcelFile(std::string fileNameDcel)           const;
        void serialize(std::ofstream& binaryFile)               const;

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
    return std::move(ConstVertexIterator(i, vertices));
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::ConstVertexIterator
 * @return Un iteratore che punta all'ultimo vertice della Dcel
 */
inline Dcel::ConstVertexIterator Dcel::vertexEnd() const {
    return std::move(ConstVertexIterator(vertices.size(), vertices));
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::ConstHalfEdgeIterator
 * @return Un iteratore che punta al primo half edge della Dcel
 */
inline Dcel::ConstHalfEdgeIterator Dcel::halfEdgeBegin() const {
    unsigned int i = 0;
    while (i < halfEdges.size() && halfEdges[i] == nullptr) ++i;
    return std::move(ConstHalfEdgeIterator(i, halfEdges));
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::ConstHalfEdgeIterator
 * @return Un iteratore che punta all'ultimo half edge della Dcel
 */
inline Dcel::ConstHalfEdgeIterator Dcel::halfEdgeEnd() const {
    return std::move(ConstHalfEdgeIterator(halfEdges.size(), halfEdges));
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::ConstFaceIterator
 * @return Un iteratore che punta alla prima faccia della Dcel
 */
inline Dcel::ConstFaceIterator Dcel::faceBegin() const {
    unsigned int i = 0;
    while (i < faces.size() && faces[i] == nullptr) ++i;
    return std::move(ConstFaceIterator(i, faces));
}

/**
 * @brief Funzione di finalizzazione di Dcel::ConstFaceIterator
 * @return Un iteratore che punta all'ultima faccia della Dcel
 */
inline Dcel::ConstFaceIterator Dcel::faceEnd() const {
    return std::move(ConstFaceIterator(faces.size(), faces));
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::VertexIterator
 * @return Un iteratore che punta al primo vertice della Dcel
 */
inline Dcel::VertexIterator Dcel::vertexBegin() {
    unsigned int i = 0;
    while (i < vertices.size() && vertices[i] == nullptr) ++i;
    return std::move(VertexIterator(i, vertices));
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::VertexIterator
 * @return Un iteratore che punta all'ultimo vertice della Dcel
 */
inline Dcel::VertexIterator Dcel::vertexEnd() {
    return std::move(VertexIterator(vertices.size(), vertices));
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::HalfEdgeIterator
 * @return Un iteratore che punta al primo half edge della Dcel
 */
inline Dcel::HalfEdgeIterator Dcel::halfEdgeBegin() {
    unsigned int i = 0;
    while (i < halfEdges.size() && halfEdges[i] == nullptr) ++i;
    return std::move(HalfEdgeIterator(i, halfEdges));
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::HalfEdgeIterator
 * @return Un iteratore che punta all'ultimo half edge della Dcel
 */
inline Dcel::HalfEdgeIterator Dcel::halfEdgeEnd() {
    return std::move(HalfEdgeIterator(halfEdges.size(), halfEdges));
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::FaceIterator
 * @return Un iteratore che punta alla prima faccia della Dcel
 */
inline Dcel::FaceIterator Dcel::faceBegin() {
    unsigned int i = 0;
    while (i < faces.size() && faces[i] == nullptr) ++i;
    return std::move(FaceIterator(i, faces));
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::FaceIterator
 * @return Un iteratore che punta all'ultima faccia della Dcel
 */
inline Dcel::FaceIterator Dcel::faceEnd() {
    return std::move(FaceIterator(faces.size(), faces));
}

inline const Dcel::ConstVertexRangeBasedIterator Dcel::vertexIterator() const {
    return std::move(ConstVertexRangeBasedIterator(this));
}

inline Dcel::VertexRangeBasedIterator Dcel::vertexIterator() {
    return std::move(VertexRangeBasedIterator(this));
}

inline const Dcel::ConstHalfEdgeRangeBasedIterator Dcel::halfEdgeIterator() const {
    return std::move(ConstHalfEdgeRangeBasedIterator(this));
}

inline Dcel::HalfEdgeRangeBasedIterator Dcel::halfEdgeIterator() {
    return std::move(HalfEdgeRangeBasedIterator(this));
}

inline const Dcel::ConstFaceRangeBasedIterator Dcel::faceIterator() const {
    return std::move(ConstFaceRangeBasedIterator(this));
}

inline Dcel::FaceRangeBasedIterator Dcel::faceIterator() {
    return std::move(FaceRangeBasedIterator(this));
}

inline Dcel::ConstVertexIterator Dcel::ConstVertexRangeBasedIterator::begin() const {
    return std::move(d->vertexBegin());
}

inline Dcel::ConstVertexIterator Dcel::ConstVertexRangeBasedIterator::end() const {
    return std::move(d->vertexEnd());
}

inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeRangeBasedIterator::begin() const {
    return std::move(d->halfEdgeBegin());
}

inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeRangeBasedIterator::end() const {
    return std::move(d->halfEdgeEnd());
}

inline Dcel::ConstFaceIterator Dcel::ConstFaceRangeBasedIterator::begin() const {
    return std::move(d->faceBegin());
}

inline Dcel::ConstFaceIterator Dcel::ConstFaceRangeBasedIterator::end() const {
    return std::move(d->faceEnd());
}

inline Dcel::VertexIterator Dcel::VertexRangeBasedIterator::begin() {
    return std::move(d->vertexBegin());
}

inline Dcel::VertexIterator Dcel::VertexRangeBasedIterator::end() {
    return std::move(d->vertexEnd());
}

inline Dcel::HalfEdgeIterator Dcel::HalfEdgeRangeBasedIterator::begin() {
    return std::move(d->halfEdgeBegin());
}

inline Dcel::HalfEdgeIterator Dcel::HalfEdgeRangeBasedIterator::end() {
    return std::move(d->halfEdgeEnd());
}

inline Dcel::FaceIterator Dcel::FaceRangeBasedIterator::begin() {
    return std::move(d->faceBegin());
}

inline Dcel::FaceIterator Dcel::FaceRangeBasedIterator::end() {
    return std::move(d->faceEnd());
}

#endif // DCEL_STRUCT_H
