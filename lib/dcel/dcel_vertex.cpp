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
Dcel::Vertex::Vertex() : incidentHalfEdge(nullptr), cardinality(0), id(0), flag(0) {
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
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstAdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'incidentHalfEdge.
 *
 * @return Un iteratore che punta al toVertex dell'incidentHalfEdge
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin() const {
    return ConstAdjacentVertexIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::ConstAdjacentVertexIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexEnd() const {
    return ConstAdjacentVertexIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstAdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromoVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(const HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstAdjacentVertexIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstAdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'half edge start e fino al toVertex dell'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, il cui toVertex \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromoVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(const HalfEdge* start, const HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstAdjacentVertexIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstAdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal vertice start.\n
 * È meno efficiente rispetto a Dcel::Vertex::constAdjacentVertexBegin(const Dcel::HalfEdge* start).
 *
 * @param[in] start: vertice di partenza
 * @warning Se start non risulta essere adiacente al vertice this (ossia non esiste un half edge condiviso tra this e start),
 * viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 * @par Complessità:
 *      \e O(Cardinality(start))
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(const Dcel::Vertex* start) const {
    Vertex::ConstIncomingHalfEdgeIterator heit;
    for (heit = start->incomingHalfEdgeBegin(); heit != start->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFromVertex() == this) return ConstAdjacentVertexIterator(*heit, *heit, this);
    }
    std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
    assert(0);
    return ConstAdjacentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstAdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal vertice start e fino al vertice end.\n
 * È meno efficiente rispetto a Dcel::Vertex::constAdjacentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end).
 *
 * @param[in] start: vertice di partenza
 * @param[in] end: vertice di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere adiacenti al vertice this (ossia non esiste un half edge condiviso tra this e start e tra this e end),
 * viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 * @par Complessità:
 *      \e O(Cardinality(start)) + \e O(Cardinality(end))
 */
Dcel::Vertex::ConstAdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(const Dcel::Vertex* start, const Dcel::Vertex* end) const {
    Vertex::ConstIncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
    while (heit != start->incomingHalfEdgeEnd() && ((*heit)->getFromVertex() != this)) ++heit;
    #ifdef DEBUG
    if ((*heit)->getFromVertex() != this) {
        qDebug() << "ERROR: start vertex " << start->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
        assert(0);
    }
    #endif
    const HalfEdge* s = *heit;
    for (heit = end->incomingHalfEdgeBegin(); heit != end->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFromVertex() == this) return ConstAdjacentVertexIterator(s, *heit, this);
    }
    std::cerr << "ERROR: end vertex " << end->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
    assert(0);
    return ConstAdjacentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstOutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'incidentHalfEdge.
 *
 * @return Un iteratore che punta all'incidentHalfEdge del vertice
 */
Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin() const {
    return ConstOutgoingHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::ConstOutgoingHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeEnd() const {
    return ConstOutgoingHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstOutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin(const Dcel::HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstOutgoingHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstOutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstOutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin(const HalfEdge* start, const HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstOutgoingHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dal twin dell'incidentHalfEdge.
 *
 * @return Un iteratore che punta al twin dell'incidentHalfEdge del vertice
 */
Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin() const {
    return ConstIncomingHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeEnd() const {
    return ConstIncomingHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come toVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin(const HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getToVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncomingHalfEdgeIterator(start->getTwin(), start->getTwin(), this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come toVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstIncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin(const HalfEdge* start, const HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getToVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    if (end->getToVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncomingHalfEdgeIterator(start->getTwin(), end->getTwin(), this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti, sia uscenti che entranti nel vertice, partendo dall'incidentHalfEdge.
 *
 * @return Un iteratore che punta all'incidentHalfEdge del vertice
 */
Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin() const {
    return ConstIncidentHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::ConstIncidentHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeEnd() const {
    return ConstIncidentHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti e entranti nel vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come toVertex o fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin(const HalfEdge *start) const {
    #ifdef DEBUG
    if (start->getToVertex() != this || start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncidentHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti e entranti nel vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come toVertex o fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin(const HalfEdge *start, const HalfEdge *end) const {
    #ifdef DEBUG
    if (start->getToVertex() != this && start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    if (end->getToVertex() != this && end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncidentHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'incidentHalfEdge.
 *
 * @return Un iteratore che punta alla faccia incidente sull'incidentHalfEdge del vertice
 */
Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::incidentFaceBegin() const {
    return ConstIncidentFaceIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::ConstIncidentFaceIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::incidentFaceEnd() const {
    return ConstIncidentFaceIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta alla faccia incidente sull'half edge start
 */
Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::incidentFaceBegin(const HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncidentFaceIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::ConstIncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'half edge start e fino alla faccia incidente sull'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta alla faccia incidente sull'half edge start
 */
Dcel::Vertex::ConstIncidentFaceIterator Dcel::Vertex::incidentFaceBegin(const HalfEdge* start, const HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return ConstIncidentFaceIterator(start, end, this);
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

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::AdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'incidentHalfEdge.
 *
 * @return Un iteratore che punta al toVertex dell'incidentHalfEdge
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin() {
    #ifdef DEBUG
    checkIncidentHalfEdge();
    #endif
    return AdjacentVertexIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::AdjacentVertexIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexEnd() {
    return AdjacentVertexIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::AdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromoVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(HalfEdge* start) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return AdjacentVertexIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::AdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal toVertex dell'half edge start e fino al toVertex dell'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, il cui toVertex \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromoVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(HalfEdge* start, HalfEdge* end) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return AdjacentVertexIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::AdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal vertice start.\n
 * È meno efficiente rispetto a Dcel::Vertex::constAdjacentVertexBegin(const Dcel::HalfEdge* start).
 *
 * @param[in] start: vertice di partenza
 * @warning Se start non risulta essere adiacente al vertice this (ossia non esiste un half edge condiviso tra this e start),
 * viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 * @par Complessità:
 *      \e O(Cardinality(start))
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(Vertex* start) {
    Vertex::IncomingHalfEdgeIterator heit;
    for (heit= start->incomingHalfEdgeBegin(); heit!= start->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFromVertex() == this) return AdjacentVertexIterator(*heit, *heit, this);
    }
    std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
    assert(0);
    return AdjacentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::AdjacentVertexIterator.
 *
 * Permette di ciclare sui vertici adiacenti al vertice, partendo dal vertice start e fino al vertice end.\n
 * È meno efficiente rispetto a Dcel::Vertex::constAdjacentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end).
 *
 * @param[in] start: vertice di partenza
 * @param[in] end: vertice di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere adiacenti al vertice this (ossia non esiste un half edge condiviso tra this e start e tra this e end),
 * viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 * @par Complessità:
 *      \e O(Cardinality(start)) + \e O(Cardinality(end))
 */
Dcel::Vertex::AdjacentVertexIterator Dcel::Vertex::adjacentVertexBegin(Vertex* start, Vertex* end) {
    Vertex::IncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
    while (heit!= start->incomingHalfEdgeEnd() && ((*heit)->getFromVertex() != this)) ++heit;
    #ifdef DEBUG
    if ((*heit)->getFromVertex() != this) {
        qDebug() << "ERROR: start vertex " << start->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
        assert(0);
    }
    #endif
    HalfEdge* s = *heit;
    for (heit= end->incomingHalfEdgeBegin(); heit!= end->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFromVertex() == this) return AdjacentVertexIterator(s, *heit, this);
    }
    std::cerr << "ERROR: end vertex " << end->getId() << " hasn't this vertex " << this->getId() << " as adjacent vertex.\n";
    assert(0);
    return AdjacentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::OutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'incidentHalfEdge.
 *
 * @return Un iteratore che punta all'incidentHalfEdge del vertice
 */
Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin() {
    #ifdef DEBUG
    checkIncidentHalfEdge();
    #endif
    return OutgoingHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::OutgoingHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeEnd() {
    return OutgoingHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::OutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin(HalfEdge* start) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return OutgoingHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::OutgoingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti dal vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::OutgoingHalfEdgeIterator Dcel::Vertex::outgoingHalfEdgeBegin(HalfEdge* start, HalfEdge* end) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return OutgoingHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dal twin dell'incidentHalfEdge.
 *
 * @return Un iteratore che punta al twin dell'incidentHalfEdge del vertice
 */
Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin() {
    #ifdef DEBUG
    checkIncidentHalfEdge();
    #endif
    return IncomingHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::IncomingHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeEnd() {
    return IncomingHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come toVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin(HalfEdge* start) {
    #ifdef DEBUG
    if (start->getToVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    #endif
    return IncomingHalfEdgeIterator(start->getTwin(), start->getTwin(), this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncomingHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti entranti nel vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come toVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::IncomingHalfEdgeIterator Dcel::Vertex::incomingHalfEdgeBegin(HalfEdge* start, HalfEdge* end) {
    #ifdef DEBUG
    if (start->getToVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    if (end->getToVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as to vertex.\n";
        assert(0);
    }
    #endif
    return IncomingHalfEdgeIterator(start->getTwin(), end->getTwin(), this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti, sia uscenti che entranti nel vertice, partendo dall'incidentHalfEdge.
 *
 * @return Un iteratore che punta all'incidentHalfEdge del vertice
 */
Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin() {
    #ifdef DEBUG
    checkIncidentHalfEdge();
    #endif
    return IncidentHalfEdgeIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::IncidentHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeEnd() {
    return IncidentHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti e entranti nel vertice, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come toVertex o fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin(HalfEdge* start) {
    #ifdef DEBUG
    if (start->getToVertex() != this || start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    #endif
    return IncidentHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti uscenti e entranti nel vertice, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come toVertex o fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::incidentHalfEdgeBegin(HalfEdge* start, HalfEdge* end) {
    #ifdef DEBUG
    if (start->getToVertex() != this && start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    if (end->getToVertex() != this && end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as to or from vertex.\n";
        assert(0);
    }
    #endif
    return IncidentHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'incidentHalfEdge.
 *
 * @return Un iteratore che punta alla faccia incidente sull'incidentHalfEdge del vertice
 */
Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::incidentFaceBegin() {
    #ifdef DEBUG
    checkIncidentHalfEdge();
    #endif
    return IncidentFaceIterator(incidentHalfEdge, incidentHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Vertex::IncidentFaceIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::incidentFaceEnd() {
    return IncidentFaceIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta alla faccia incidente sull'half edge start
 */
Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::incidentFaceBegin(HalfEdge* start) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return IncidentFaceIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Vertex::IncidentFaceIterator.
 *
 * Permette di ciclare sulle facce incidenti sul vertice, partendo dalla faccia incidente sull'half edge start e fino alla faccia incidente sull'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, che \b non \b è \b compreso
 * @warning Se start e end non risultano avere il vertice this come fromVertex, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta alla faccia incidente sull'half edge start
 */
Dcel::Vertex::IncidentFaceIterator Dcel::Vertex::incidentFaceBegin(HalfEdge* start, HalfEdge* end) {
    #ifdef DEBUG
    if (start->getFromVertex() != this){
        qDebug() << "ERROR: start half edge " << start->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    if (end->getFromVertex() != this){
        qDebug() << "ERROR: end half edge " << end->getId() << " hasn't this vertex " << this->getId() << " as from vertex.\n";
        assert(0);
    }
    #endif
    return IncidentFaceIterator(start, end, this);
}
