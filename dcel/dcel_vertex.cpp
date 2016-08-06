/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "dcel_vertex_iterators.h"
#include "dcel_face.h"

/****************
 * Constructors *
 ****************/

/**
 * \~Italian
 * @brief Costruttore di default.
 *
 * Crea un vertice con:
 * - posizione pari a (0, 0, 0);
 * - half edge incidente settato a nullptr;
 * - vettore normale pari a (0, 0, 0);
 * - cardinalità pari a 0;
 * - id pari a 0;
 * - flag pari a 0.
 */
Dcel::Vertex::Vertex() : incidentHalfEdge(nullptr), cardinality(0), id(0), flag(0){
}

/**
 * \~Italian
 * @brief Costruttore di default.
 *
 * Crea un vertice con:
 * - posizione pari al parametro in input p;
 * - half edge incidente settato a nullptr;
 * - vettore normale pari a (0, 0, 0);
 * - cardinalità pari a 0;
 * - id pari a 0;
 * - flag pari a 0.
 *
 * @param[in] p: Poind rappresentante la posizione assegnata al vertice
 */
Dcel::Vertex::Vertex(const Pointd& p) : coordinate(p), incidentHalfEdge(nullptr), cardinality(0), id(0), flag(0) {
}

/**
 * \~Italian
 * @brief Costruttore di default.
 *
 * Crea un vertice con:
 * - posizione pari al parametro in input p;
 * - half edge incidente pari al parametro in input halfEdge;
 * - vettore normale pari a (0, 0, 0);
 * - cardinalità pari a 0;
 * - id pari a 0;
 * - flag pari a 0.
 *
 * @param[in] p: Poind rappresentante la posizione assegnata al vertice
 * @param[in] halfEdge: puntatore all'half edge incidente assegnato al vertice
 */
Dcel::Vertex::Vertex(const Pointd& p, Dcel::HalfEdge* halfEdge) : coordinate(p), incidentHalfEdge(halfEdge), cardinality(0), id(0), flag(0) {
}

/**
 * \~Italian
 * @brief Costruttore di default.
 *
 * Crea un vertice con:
 * - posizione pari al parametro in input p;
 * - half edge incidente pari al parametro in input halfEdge;
 * - vettore normale pari a (0, 0, 0);
 * - cardinalità pari al parametro in input cardinality;
 * - id pari a 0;
 * - flag pari a 0.
 *
 * @param[in] p: Poind rappresentante la posizione assegnata al vertice
 * @param[in] halfEdge: puntatore all'half edge incidente assegnato al vertice
 * @param[in] cardinality: cardinalità settata al vertice
 */
Dcel::Vertex::Vertex(const Pointd& p, Dcel::HalfEdge* halfEdge, int cardinality) : coordinate(p), incidentHalfEdge(halfEdge), cardinality(cardinality), id(0), flag(0) {
}

/**
 * \~Italian
 * @brief Distruttore vuoto.
 *
 * La classe Dcel dovrà occuparsi di eliminare tutti i riferimenti in essa contenuti (e quindi contenuti di conseguenza anche nella classe Dcel::Vertex).
 */
Dcel::Vertex::~Vertex(void) {
}

/******************
 * Public Methods *
 ******************/
/**
 * \~Italian
 * @brief Calcola e restituisce il numero di half edge (non di edge) incidenti sul vertice
 * @warning Utilizza Dcel::Vertex::ConstIncidentHalfEdgeIterator
 * @return Il numero di edge incidenti sul vertice
 * @par Complessità:
 *      \e O(Cardinality)
 */
int Dcel::Vertex::getNumberIncidentHalfEdges() const {
    int n = 0;
    ConstIncidentHalfEdgeIterator it;
    for (it = incidentHalfEdgeBegin(); it != incidentHalfEdgeEnd(); ++it) n++;
    return n;
}

/**
 * \~Italian
 * @brief Calcola e restituisce il numero di facce incidenti sul vertice
 * @note Se un'unica faccia incide sul vertice due volte, questa verrà contata due volte
 * @warning Utilizza Dcel::Vertex::ConstIncidentFaceIterator
 * @return Il numero di facce incidenti sul vertice
 * @par Complessità:
 *      \e O(Cardinality)
 */
int Dcel::Vertex::getNumberIncidentFaces() const {
    int n = 0;
    ConstIncidentFaceIterator it;
    for (it = incidentFaceBegin(); it != incidentFaceEnd(); ++it) n++;
    return n;
}

/**
 * \~Italian
 * @brief Calcola e restituisce il numero di vertici adiacenti al vertice
 * @warning Utilizza Dcel::Vertex::ConstAdjacentVertexIterator
 * @return Il numero di vertici adiacenti al vertice
 * @par Complessità:
 *      \e O(Cardinality)
 */
int Dcel::Vertex::getNumberAdjacentVertices() const {
    int n = 0;
    ConstAdjacentVertexIterator it;
    for (it = adjacentVertexBegin(); it != adjacentVertexEnd(); ++it) n++;
    return n;
}

/**
 * \~Italian
 * @brief Funzione che cerca e restituisce l'half edge costante condiviso dal vertice this e vertex
 *
 * Nello specifico, se viene cercato l'half edge che ha come origine il vertice this e
 * come destinazione il vertice in input vertex. Se l'half edge non viene trovato (non esiste
 * un half edge condiviso dai due vertici) viene restituito nullptr;
 *
 * @warning Utilizza Dcel::Vertex::ConstOutgoingHalfEdgeIterator
 * @param[in] vertex: vertice con cui viene cercato l'half edge condiviso
 * @return l'half edge condiviso se esiste, nullptr altrimenti
 * @par Complessità:
 *      \e O(Cardinality)
 */
const Dcel::HalfEdge* Dcel::Vertex::findSharedHalfEdge(const Dcel::Vertex* vertex) const {
    ConstOutgoingHalfEdgeIterator he;
    for (he = outgoingHalfEdgeBegin(); he != outgoingHalfEdgeEnd(); ++he){
        if ((*he)->getToVertex() == vertex) return *he;
    }
    return nullptr;
}

/**
 * \~Italian
 * @brief Funzione toString di un vertice
 * @return Una stringa rappresentativa del vertice
 * @todo Da aggiornare
 */
std::string Dcel::Vertex::toString() const {
    std::stringstream ss;

    ss << "ID: " << id << "; Position: " << coordinate.toString() << "; Normal: " << normal.toString()
       << "; Half-Edge: " ;
    if (incidentHalfEdge == nullptr) ss << "nullptr";
    else ss << incidentHalfEdge->getId();
    ss << "; Card:" << cardinality << ".";

    std::string s1 = ss.str();
    return s1;
}

/**
 * \~Italian
 * @brief Ricalcola e restituisce la normale al vertice, e aggiorna la cardinalità del vertice
 * @warning Utilizza Dcel::Vertex::ConstIncidentFaceIterator
 * @return La normale al vertice appena calcolata
 * @par Complessità:
 *      \e O(Cardinality)
 */
Vec3 Dcel::Vertex::updateNormal() {
    normal.set(0,0,0);
    unsigned int n = 0;
    ConstIncidentFaceIterator f;
    for (f = incidentFaceBegin(); f != incidentFaceEnd(); ++f) {
        normal += (*f)->getNormal();
        n++;
    }
    normal /= n;
    cardinality = n;
    return normal;
}

/**
 * \~Italian
 * @brief Ricalcola e restituisce la cardinalità del vertice, ossia il numero di \b edge (non half edge!) incidenti
 * @warning Utilizza Dcel::Vertex::ConstOutgoingHalfEdgeIterator
 * @return La cardinalità del vertice appena calcolata
 * @par Complessità:
 *      \e O(Cardinality)
 */
unsigned int Dcel::Vertex::updateCardinality() {
    unsigned int c = 0;
    ConstOutgoingHalfEdgeIterator heit;
    for (heit = outgoingHalfEdgeBegin(); heit != outgoingHalfEdgeEnd(); ++heit)
        c++;
    return c;
}

/**
 * \~Italian
 * @brief Funzione che cerca e restituisce l'half edge condiviso dal vertice this e vertex
 *
 * Nello specifico, se viene cercato l'half edge che ha come origine il vertice this e
 * come destinazione il vertice in input vertex. Se l'half edge non viene trovato (non esiste
 * un half edge condiviso dai due vertici) viene restituito nullptr;
 *
 * @warning Utilizza Dcel::Vertex::OutgoingHalfEdgeIterator
 * @param[in] vertex: vertice con cui viene cercato l'half edge condiviso
 * @return l'half edge condiviso se esiste, nullptr altrimenti
 * @par Complessità:
 *      \e O(Cardinality)
 */
Dcel::HalfEdge* Dcel::Vertex::findSharedHalfEdge(const Vertex* vertex) {
    OutgoingHalfEdgeIterator he;
    for (he = this->outgoingHalfEdgeBegin(); he != this->outgoingHalfEdgeEnd(); ++he){
        if ((*he)->getToVertex() == vertex) return *he;
    }
    return nullptr;
}



/*Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::ConstAdjacentVertexRangeBasedIterator::begin() const {
    return std::move(v->adjacentVertexBegin());
}

Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::ConstAdjacentVertexRangeBasedIterator::end() const {
    return std::move(v->adjacentVertexEnd());
}

Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::AdjacentVertexRangeBasedIterator::begin() {
    return std::move(v->adjacentVertexBegin());
}

Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::AdjacentVertexRangeBasedIterator::end() {
    return std::move(v->adjacentVertexEnd());
}

Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::ConstOutgoingHalfEdgeRangeBasedIterator::begin() const {
    return std::move(v->outgoingHalfEdgeBegin());
}

Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::ConstOutgoingHalfEdgeRangeBasedIterator::end() const {
    return std::move(v->outgoingHalfEdgeEnd());
}

Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::OutgoingHalfEdgeRangeBasedIterator::begin() {
    return std::move(v->outgoingHalfEdgeBegin());
}

Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::OutgoingHalfEdgeRangeBasedIterator::end() {
    return std::move(v->outgoingHalfEdgeEnd());
}

Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::ConstIncomingHalfEdgeRangeBasedIterator::begin() const {
    return std::move(v->incomingHalfEdgeBegin());
}

Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::ConstIncomingHalfEdgeRangeBasedIterator::end() const {
    return std::move(v->incomingHalfEdgeEnd());
}

Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::IncomingHalfEdgeRangeBasedIterator::begin() {
    return std::move(v->incomingHalfEdgeBegin());
}

Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::IncomingHalfEdgeRangeBasedIterator::end() {
    return std::move(v->incomingHalfEdgeEnd());
}

Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeRangeBasedIterator::begin() const {
    return std::move(v->incidentHalfEdgeBegin());
}

Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeRangeBasedIterator::end() const {
    return std::move(v->incidentHalfEdgeEnd());
}

Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeRangeBasedIterator::begin() {
    return std::move(v->incidentHalfEdgeBegin());
}

Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeRangeBasedIterator::end() {
    return std::move(v->incidentHalfEdgeEnd());
}

Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::ConstIncidentFaceRangeBasedIterator::begin() const {
    return std::move(v->incidentFaceBegin());
}

Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::ConstIncidentFaceRangeBasedIterator::end() const {
    return std::move(v->incidentFaceEnd());
}

Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::IncidentFaceRangeBasedIterator::begin() {
    return std::move(v->incidentFaceBegin());
}

Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::IncidentFaceRangeBasedIterator::end() {
    return std::move(v->incidentFaceEnd());
}*/
