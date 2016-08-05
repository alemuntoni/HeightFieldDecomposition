/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_VERTEX_ITERATORS_H
#define DCEL_VERTEX_ITERATORS_H

#include "dcel_vertex.h"
#include "dcel_half_edge.h"

class Dcel::Vertex::GenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructor
        GenericIterator();

        //Public Operators
        void* operator * () const {return nullptr;}
        bool operator == (const GenericIterator& right)  const;
        bool operator != (const GenericIterator& right)  const;

        GenericIterator operator ++ ();
        GenericIterator operator ++ (int);
        GenericIterator operator -- ();
        GenericIterator operator -- (int);

    protected:
        //Protected Attributes
        Dcel::Vertex*   v;      /**< \~Italian @brief Vertice su cui vengono iterati i vertici adiacenti */
        Dcel::HalfEdge* start;  /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        Dcel::HalfEdge* pos;    /**< \~Italian @brief Posizione attuale dell'iteratore */
        Dcel::HalfEdge* end;    /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        GenericIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

class Dcel::Vertex::ConstGenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstGenericIterator();
        ConstGenericIterator(const Dcel::Vertex::GenericIterator& it);

        //Public Operators
        const void* operator * () const {return nullptr;}
        bool operator == (const ConstGenericIterator& right) const;
        bool operator != (const ConstGenericIterator& right) const;

        ConstGenericIterator operator ++ ();
        ConstGenericIterator operator ++ (int);
        ConstGenericIterator operator -- ();
        ConstGenericIterator operator -- (int);

    protected:
        //Protected Attributes
        const Dcel::Vertex*   v;        /**< \~Italian @brief Vertice su cui vengono iterati i vertici adiacenti */
        const Dcel::HalfEdge* start;    /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        const Dcel::HalfEdge* pos;      /**< \~Italian @brief Posizione attuale dell'iteratore */
        const Dcel::HalfEdge* end;      /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        ConstGenericIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::AdjacentVertexIterator
 * @brief Iteratore che permette di ciclare sui vertici adiacenti ad un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e uscenti
 * dal vertice su cui è inizializzaro l'iteratore, permette di ciclare su tutti i toVertex degli
 * half edge uscenti compresi tra start e end (ossia di ciclare sui vertici adiacenti). \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce il toVertex dell'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti i vertici adiacenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::AdjacentVertexIterator vit = v->adjacentVertexBegin(); vit != v->adjacentVertexEnd(); ++vit){
 *     Dcel::Vertex* av = *vit;
 *     // operazioni su av
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti i vertici adiacenti compresi tra due dati vertici adiacenti. In questo caso sarà necessario
 * passare i vertici estremi dell'intervallo all'inizializzatore Dcel::Vertex::adjacentVertexBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità del vertice e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Vertex. Per const Dcel::Vertex, vedere Dcel::Vertex::ConstAdjacentVertexIterator.
 */
class Dcel::Vertex::AdjacentVertexIterator : public Dcel::Vertex::GenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructor
        AdjacentVertexIterator();

        //Public Operators
        Dcel::Vertex* operator * ()                             const;

    protected:

        //Protected Constructor
        AdjacentVertexIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::ConstAdjacentVertexIterator
 * @brief Iteratore che permette di ciclare sui vertici adiacenti ad un vertice, garantendone l'immutabilità.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e uscenti
 * dal vertice su cui è inizializzaro l'iteratore, permette di ciclare su tutti i toVertex degli
 * half edge uscenti compresi tra start e end (ossia di ciclare sui vertici adiacenti). \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce il toVertex dell'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti i vertici adiacenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::ConstAdjacentVertexIterator vit = v->adjacentVertexBegin(); vit != v->adjacentVertexEnd(); ++vit){
 *     const Dcel::Vertex* av = *vit;
 *     // operazioni const su av
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti i vertici adiacenti compresi tra due dati vertici adiacenti. In questo caso sarà necessario
 * passare i vertici estremi dell'intervallo all'inizializzatore Dcel::Vertex::adjacentVertexBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità del vertice della Dcel a cui appartiene, e quindi è possibile utilizzarlo solo su const Dcel::Vertex.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::Vertex::AdjacentVertexIterator.
 */
class Dcel::Vertex::ConstAdjacentVertexIterator : public Dcel::Vertex::ConstGenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstAdjacentVertexIterator();
        ConstAdjacentVertexIterator(const Dcel::Vertex::AdjacentVertexIterator& it);

        //Public Operators
        const Dcel::Vertex* operator * ()                           const;

    protected:

        //Protected Constructor
        ConstAdjacentVertexIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::OutgoingHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge uscenti da un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e uscenti
 * dal vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge uscenti
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce l'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti gli half edge uscenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::OutgoingHalfEdgeIterator heit = v->outgoingHalfEdgeBegin(); heit != v->outgoingHalfEdgeEnd(); ++heit){
 *     Dcel::HalfEdge* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge uscenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::outgoingHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità del vertice e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Vertex. Per const Dcel::Vertex, vedere Dcel::Vertex::ConstOutgoingHalfEdgeIterator.
 */
class Dcel::Vertex::OutgoingHalfEdgeIterator : public Dcel::Vertex::GenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructor
        OutgoingHalfEdgeIterator();

        //Public Operators
        Dcel::HalfEdge* operator * ()                               const;

    protected:

        //Protected Constructor
        OutgoingHalfEdgeIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::ConstOutgoingHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge uscenti da un vertice, garantendone l'immutabilità.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e uscenti
 * dal vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge uscenti
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce l'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti gli half edge uscenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::ConstOutgoingHalfEdgeIterator heit = v->outgoingHalfEdgeBegin(); heit != v->outgoingHalfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge uscenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::outgoingHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità del vertice della Dcel a cui appartiene, e quindi è possibile utilizzarlo solo su const Dcel::Vertex.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::Vertex::OutgoingHalfEdgeIterator.
 */
class Dcel::Vertex::ConstOutgoingHalfEdgeIterator : public Dcel::Vertex::ConstGenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstOutgoingHalfEdgeIterator();
        ConstOutgoingHalfEdgeIterator(const Dcel::Vertex::OutgoingHalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * ()                             const;

    protected:

        //Protected Constructor
        ConstOutgoingHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::IncomingHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge entranti in un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e \b uscenti
 * dal vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge entranti
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce il twin dell'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti gli half edge entranti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::IncomingHalfEdgeIterator heit = v->incomingHalfEdgeBegin(); heit != v->incomingHalfEdgeEnd(); ++heit){
 *     Dcel::HalfEdge* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge entranti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incomingHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità del vertice e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Vertex. Per const Dcel::Vertex, vedere Dcel::Vertex::ConstIncomingHalfEdgeIterator.
 */
class Dcel::Vertex::IncomingHalfEdgeIterator : public GenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructor
        IncomingHalfEdgeIterator();

        //Public Operators
        Dcel::HalfEdge* operator * ()                               const;

    protected:

        //Protected Constructor
        IncomingHalfEdgeIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge entranti in un vertice, garantendone l'immutabilità.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti e \b uscenti
 * dal vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge entranti
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce il twin dell'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti gli half edge entranti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::ConstIncomingHalfEdgeIterator heit = v->incomingHalfEdgeBegin(); heit != v->incomingHalfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge entranti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incomingHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità del vertice della Dcel a cui appartiene, e quindi è possibile utilizzarlo solo su const Dcel::Vertex.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::Vertex::IncomingHalfEdgeIterator.
 */
class Dcel::Vertex::ConstIncomingHalfEdgeIterator : public Dcel::Vertex::ConstGenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstIncomingHalfEdgeIterator();
        ConstIncomingHalfEdgeIterator(const Dcel::Vertex::IncomingHalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * ()                             const;

    protected:

        //Protected Constructor
        ConstIncomingHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::IncidentHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge uscenti e entranti in un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti
 * sul vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione di \c twin() se l'half edge è uscente e \c next()
 * se l'half edge è entrante nel vertice, le operazioni di decremento eseguono un'operazione di \c prev()
 * se l'half edge è uscente e \c twin() se l'half edge è entrante nel vertice, e l'operazione di
 * dereferenziazione restituisce l'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti gli half edge incidenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::IncidentHalfEdgeIterator heit = v->incidentHalfEdgeBegin(); heit != v->incidentHalfEdgeEnd(); ++heit){
 *     Dcel::HalfEdge* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge incidenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incidentHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità del vertice e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Vertex. Per const Dcel::Vertex, vedere Dcel::Vertex::ConstIncidentHalfEdgeIterator.
 */
class Dcel::Vertex::IncidentHalfEdgeIterator : public Dcel::Vertex::GenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructor
        IncidentHalfEdgeIterator();

        //Public Operators
        Dcel::HalfEdge* operator * ()                               const;

        IncidentHalfEdgeIterator operator ++ ();
        IncidentHalfEdgeIterator operator ++ (int);
        IncidentHalfEdgeIterator operator -- ();
        IncidentHalfEdgeIterator operator -- (int);

    protected:

        //Protected Constructor
        IncidentHalfEdgeIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::ConstIncidentHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge uscenti e entranti in un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti
 * sul vertice su cui è inizializzato l'iteratore, permette di ciclare su tutti gli half edge
 * compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione di \c twin() se l'half edge è uscente e \c next()
 * se l'half edge è entrante nel vertice, le operazioni di decremento eseguono un'operazione di \c prev()
 * se l'half edge è uscente e \c twin() se l'half edge è entrante nel vertice, e l'operazione di
 * dereferenziazione restituisce l'half edge puntato dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti i gli half edge incidenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::ConstIncidentHalfEdgeIterator heit = v->incidentHalfEdgeBegin(); heit != v->incidentHalfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge incidenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incidentHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità del vertice della Dcel a cui appartiene, e quindi è possibile utilizzarlo solo su const Dcel::Vertex.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::Vertex::IncidentHalfEdgeIterator.
 */
class Dcel::Vertex::ConstIncidentHalfEdgeIterator : public ConstGenericIterator{

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstIncidentHalfEdgeIterator();
        ConstIncidentHalfEdgeIterator(const Dcel::Vertex::IncidentHalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * ()                             const;

        ConstIncidentHalfEdgeIterator operator ++ ();
        ConstIncidentHalfEdgeIterator operator ++ (int);
        ConstIncidentHalfEdgeIterator operator -- ();
        ConstIncidentHalfEdgeIterator operator -- (int);

    protected:

        //Protected Constructor
        ConstIncidentHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::IncidentFaceIterator
 * @brief Iteratore che permette di ciclare sulle facce incidenti su un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti
 * sul vertice su cui è inizializzato l'iteratore, permette di ciclare su tutte le facce
 * incidenti sugli half edge compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce la faccia incidente sull'half edge puntato
 * dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti le facce incidenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::IncidentFaceIterator heit = v->incidentFaceBegin(); heit != v->incidentFaceEnd(); ++heit){
 *     Dcel::Face* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti le facce incidenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incidentFaceBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità del vertice e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Vertex. Per const Dcel::Vertex, vedere Dcel::Vertex::ConstFaceIterator.
 *
 * @warning Nel caso di Dcel composte da poligoni generici, è possibile che un vertice abbia la stessa faccia incidente su due punti diversi.
 */
class Dcel::Vertex::IncidentFaceIterator : public Dcel::Vertex::GenericIterator{

        friend class Dcel::Vertex;

    public:
        //Constructor
        IncidentFaceIterator();

        //Public Operators
        Dcel::Face* operator * ()                               const;

    protected:

        //Protected Constructor
        IncidentFaceIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Vertex* v);
};

/**
 * \~Italian
 * @class Dcel::Vertex::ConstIncidentFaceIterator
 * @brief Iteratore che permette di ciclare sulle facce incidenti su un vertice.
 *
 * Partendo da un half edge start e un half edge end (non compreso), entrambi incidenti
 * sul vertice su cui è inizializzato l'iteratore, permette di ciclare su tutte le facce
 * incidenti sugli half edge compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono le due operazioni di \c twin() e \c next() sull'half edge,
 * le operazioni di decremento eseguono le due operazioni di \c prev() e \c twin() sull'half edge,
 * e l'operazione di dereferenziazione restituisce la faccia incidente sull'half edge puntato
 * dall'iteratore.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * (entranti e uscenti) hanno settato correttamente le seguenti relazioni di incidenza:
 * - from vertex (per gli uscenti deve essere il vertice su cui si itera)
 * - to vertex (per gli entranti deve essere il vertice su cui si itera)
 * - twin (sia per le operazioni di incremento che per quelle di decremento)
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso orario o antiorario (operatore \--)
 * su tutti le facce incidenti ad un dato vertice \c v. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Vertex::ConstIncidentFaceIterator heit = v->incidentFaceBegin(); heit != v->incidentFaceEnd(); ++heit){
 *     const Dcel::Face* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti le facce incidenti compresi tra due dati half edge incidenti. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Vertex::incidentFaceBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità del vertice della Dcel a cui appartiene, e quindi è possibile utilizzarlo solo su const Dcel::Vertex.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::Vertex::ConstIncidentFaceIterator.
 *
 * @warning Nel caso di Dcel composte da poligoni generici, è possibile che un vertice abbia la stessa faccia incidente su due punti diversi.
 */
class Dcel::Vertex::ConstIncidentFaceIterator : public Dcel::Vertex::ConstGenericIterator {

        friend class Dcel::Vertex;

    public:
        //Constructors
        ConstIncidentFaceIterator();
        ConstIncidentFaceIterator(const Dcel::Vertex::IncidentFaceIterator& it);

        //Public Operators
        const Dcel::Face* operator * ()                             const;

    private:

        //Protected Constructor
        ConstIncidentFaceIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v);
};

/****************************************
 * DCEL::Vertex::GenericIterator *
 ****************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::GenericIterator::GenericIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra iteratori.
 *
 * Due AdjacentVertexIterator sono considerati uguali se:
 * - puntano allo stesso vertice (posizione sullo stesso half edge);
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Vertex::GenericIterator::operator == (const GenericIterator& right) const {
    if (this->pos == right.pos && this->v == right.v) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra iteratori
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return true se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Vertex::GenericIterator::operator != (const GenericIterator& right) const {
    return !(*this == right);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'iteratore.
 *
 * Esegue un'operazione di \c twin() e \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Vertex::GenericIterator Dcel::Vertex::GenericIterator::operator ++ () {
    pos = pos->getTwin()->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'iteratore.
 *
 * Esegue un'operazione di \c twin() e \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Vertex::GenericIterator Dcel::Vertex::GenericIterator::operator ++ (int) {
    GenericIterator old_value = *this;
    pos = pos->getTwin()->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'iteratore.
 *
 * Esegue un'operazione di \c prev() e \c twin() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Vertex::GenericIterator Dcel::Vertex::GenericIterator::operator -- () {
    pos = pos->getPrev()->getTwin();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'iteratore.
 *
 * Esegue un'operazione di \c prev() e \c twin() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Vertex::GenericIterator Dcel::Vertex::GenericIterator::operator -- (int) {
    GenericIterator old_value = *this;
    pos = pos->getPrev()->getTwin();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un iteratore.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati i vertici adiacenti
 */
inline Dcel::Vertex::GenericIterator::GenericIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : v(v), start(start), pos(start), end(end) {
}

/*********************************************
 * DCEL::Vertex::ConstGenericIterator *
 *********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstGenericIterator::ConstGenericIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un iteratore const.
 *
 * Inizializza un ConstAdjacentVertexIterator pari all'AdjacentVertexIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstGenericIterator::ConstGenericIterator(const Dcel::Vertex::GenericIterator& it) : v(it.v), start(it.start), pos(it.pos), end(it.end) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra iteratori const.
 *
 * Due AdjacentVertexIterator sono considerati uguali se:
 * - puntano allo stesso vertice (posizione sullo stesso half edge);
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Vertex::ConstGenericIterator::operator == (const ConstGenericIterator& right) const {
    if (this->pos == right.pos && this->v == right.v) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra iteratori const
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return true se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Vertex::ConstGenericIterator::operator !=(const ConstGenericIterator& right) const {
    return !(*this == right);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'iteratore const.
 *
 * Esegue un'operazione di \c twin() e \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Vertex::ConstGenericIterator Dcel::Vertex::ConstGenericIterator::operator ++ () {
    pos = pos->getTwin()->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'iteratore const.
 *
 * Esegue un'operazione di \c twin() e \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Vertex::ConstGenericIterator Dcel::Vertex::ConstGenericIterator::operator ++ (int) {
    ConstGenericIterator old_value = *this;
    pos = pos->getTwin()->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'iteratore const.
 *
 * Esegue un'operazione di \c prev() e \c twin() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Vertex::ConstGenericIterator Dcel::Vertex::ConstGenericIterator::operator -- () {
    pos = pos->getPrev()->getTwin();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'iteratore const.
 *
 * Esegue un'operazione di \c prev() e \c twin() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Vertex::ConstGenericIterator Dcel::Vertex::ConstGenericIterator::operator -- (int) {
    ConstGenericIterator old_value = *this;
    pos = pos->getPrev()->getTwin();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un iteratore const.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati i vertici adiacenti
 */
inline Dcel::Vertex::ConstGenericIterator::ConstGenericIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v) : v(v), start(start), pos(start), end(end) {
}

/****************************************
 * DCEL::Vertex::AdjacentVertexIterator *
 ****************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::AdjacentVertexIterator::AdjacentVertexIterator() : GenericIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'AdjacentVertexIterator
 * @return Il to vertex dell'half edge puntato dall'iteratore
 */
inline Dcel::Vertex* Dcel::Vertex::AdjacentVertexIterator::operator * () const {
    return pos->getToVertex();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un AdjacentVertexIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati i vertici adiacenti
 */
inline Dcel::Vertex::AdjacentVertexIterator::AdjacentVertexIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : GenericIterator(start, end, v) {
}

/*********************************************
 * DCEL::Vertex::ConstAdjacentVertexIterator *
 *********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstAdjacentVertexIterator::ConstAdjacentVertexIterator() : ConstGenericIterator(){
}

/**
 * \~Italian
 * @brief Costruttore di copia da un AdjacentVertexIterator.
 *
 * Inizializza un ConstAdjacentVertexIterator pari all'AdjacentVertexIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstAdjacentVertexIterator::ConstAdjacentVertexIterator(const Dcel::Vertex::AdjacentVertexIterator& it) : ConstGenericIterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'ConstAdjacentVertexIterator
 * @return Il to vertex dell'half edge puntato dall'iteratore
 */
inline const Dcel::Vertex* Dcel::Vertex::ConstAdjacentVertexIterator::operator * () const {
    return pos->getToVertex();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstAdjacentVertexIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati i vertici adiacenti
 */
inline Dcel::Vertex::ConstAdjacentVertexIterator::ConstAdjacentVertexIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v) : ConstGenericIterator(start, end, v) {
}

/******************************************
 * DCEL::Vertex::OutgoingHalfEdgeIterator *
 ******************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::OutgoingHalfEdgeIterator::OutgoingHalfEdgeIterator() : GenericIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'OutgoingHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline Dcel::HalfEdge* Dcel::Vertex::OutgoingHalfEdgeIterator::operator * () const {
    return pos;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un OutgoingHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge uscenti
 */
inline Dcel::Vertex::OutgoingHalfEdgeIterator::OutgoingHalfEdgeIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : GenericIterator(start, end, v) {
}

/***********************************************
 * DCEL::Vertex::ConstOutgoingHalfEdgeIterator *
 ***********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstOutgoingHalfEdgeIterator::ConstOutgoingHalfEdgeIterator() : ConstGenericIterator(){
}

/**
 * \~Italian
 * @brief Costruttore di copia da un OutgoingHalfEdgeIterator.
 *
 * Inizializza un ConstOutgoingHalfEdgeIterator pari all'OutgoingHalfEdgeIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstOutgoingHalfEdgeIterator::ConstOutgoingHalfEdgeIterator(const Dcel::Vertex::OutgoingHalfEdgeIterator& it) : ConstGenericIterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstOutgoingHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge* Dcel::Vertex::ConstOutgoingHalfEdgeIterator::operator * () const {
    return pos;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstOutgoingHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge uscenti
 */
inline Dcel::Vertex::ConstOutgoingHalfEdgeIterator::ConstOutgoingHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v) : ConstGenericIterator(start, end, v) {
}

/******************************************
 * DCEL::Vertex::IncomingHalfEdgeIterator *
 ******************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::IncomingHalfEdgeIterator::IncomingHalfEdgeIterator() : GenericIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'IncomingHalfEdgeIterator
 * @return Il twin dell'half edge puntato dall'iteratore
 */
inline Dcel::HalfEdge* Dcel::Vertex::IncomingHalfEdgeIterator::operator * () const {
    return pos->getTwin();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un IncomingHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge entranti
 */
inline Dcel::Vertex::IncomingHalfEdgeIterator::IncomingHalfEdgeIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : GenericIterator(start, end, v) {
}

/***********************************************
 * DCEL::Vertex::ConstIncomingHalfEdgeIterator *
 ***********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstIncomingHalfEdgeIterator::ConstIncomingHalfEdgeIterator() : ConstGenericIterator(){
}

/**
 * \~Italian
 * @brief Costruttore di copia da un IncomingHalfEdgeIterator.
 *
 * Inizializza un ConstIncomingHalfEdgeIterator pari all'IncomingHalfEdgeIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstIncomingHalfEdgeIterator::ConstIncomingHalfEdgeIterator(const Dcel::Vertex::IncomingHalfEdgeIterator& it) : ConstGenericIterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstIncomingHalfEdgeIterator
 * @return Il twin dell'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge* Dcel::Vertex::ConstIncomingHalfEdgeIterator::operator * () const {
    return pos->getTwin();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstIncomingHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (uscente dal vertice) di start
 * @param[in] end: half edge (uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge entranti
 */
inline Dcel::Vertex::ConstIncomingHalfEdgeIterator::ConstIncomingHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Vertex* v) : ConstGenericIterator(start, end, v) {
}

/******************************************
 * DCEL::Vertex::IncidentHalfEdgeIterator *
 ******************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator::IncidentHalfEdgeIterator() : GenericIterator(){
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'IncidentHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline Dcel::HalfEdge* Dcel::Vertex::IncidentHalfEdgeIterator::operator * () const {
    return pos;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c twin() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c next() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeIterator::operator ++ () {
    if (pos->getFromVertex() == v) pos = pos->getTwin();
    else pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c twin() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c next() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeIterator::operator ++ (int) {
    IncidentHalfEdgeIterator old_value = *this;
    if (pos->getFromVertex() == v) pos = pos->getTwin();
    else pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c twin() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeIterator::operator -- () {
    if (pos->getFromVertex() == v) pos = pos->getPrev();
    else pos = pos->getTwin();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c twin() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator Dcel::Vertex::IncidentHalfEdgeIterator::operator -- (int) {
    IncidentHalfEdgeIterator old_value = *this;
    if (pos->getFromVertex() == v) pos = pos->getPrev();
    else pos = pos->getTwin();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un IncidentHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (non per forza uscente dal vertice) di start
 * @param[in] end: half edge (non per forza uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Vertex::IncidentHalfEdgeIterator::IncidentHalfEdgeIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : GenericIterator(start, end, v) {
}

/***********************************************
 * DCEL::Vertex::ConstIncidentHalfEdgeIterator *
 ***********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator() : ConstGenericIterator(){
}

/**
 * \~Italian
 * @brief Costruttore di copia da un IncidentHalfEdgeIterator.
 *
 * Inizializza un ConstIncidentHalfEdgeIterator pari all'IncidentHalfEdgeIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator(const Dcel::Vertex::IncidentHalfEdgeIterator& it) : ConstGenericIterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstIncidentHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge* Dcel::Vertex::ConstIncidentHalfEdgeIterator::operator * () const {
    return pos;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c twin() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c next() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeIterator::operator ++ () {
    if (pos->getFromVertex() == v) pos = pos->getTwin();
    else pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c twin() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c next() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeIterator::operator ++ (int) {
    ConstIncidentHalfEdgeIterator old_value = *this;
    if (pos->getFromVertex() == v) pos = pos->getTwin();
    else pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c twin() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeIterator::operator -- () {
    if (pos->getFromVertex() == v) pos = pos->getPrev();
    else pos = pos->getTwin();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() se l'half edge attuale è uscente dal vertice su cui cicla l'iteratore,
 * mentre esegue un'operazione di \c twin() se l'half edge attuale è entrante sul vertice. Se l'half edge
 * ottenuto è uguale all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator Dcel::Vertex::ConstIncidentHalfEdgeIterator::operator -- (int) {
    ConstIncidentHalfEdgeIterator old_value = *this;
    if (pos->getFromVertex() == v) pos = pos->getPrev();
    else pos = pos->getTwin();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstIncidentHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (non per forza uscente dal vertice) di start
 * @param[in] end: half edge (non per forza uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Vertex::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator(const HalfEdge* start, const HalfEdge* end, const Vertex* v)  : ConstGenericIterator(start, end, v) {
}

/**************************************
 * DCEL::Vertex::IncidentFaceIterator *
 **************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::IncidentFaceIterator::IncidentFaceIterator() : GenericIterator(){
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'IncidentFaceIterator
 * @return La faccia incidente sull'half edge puntato dall'iteratore
 */
inline Dcel::Face* Dcel::Vertex::IncidentFaceIterator::operator * () const {
    return pos->getFace();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un IncidentFaceIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (non per forza uscente dal vertice) di start
 * @param[in] end: half edge (non per forza uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterate le facce incidenti
 */
inline Dcel::Vertex::IncidentFaceIterator::IncidentFaceIterator(HalfEdge* start, HalfEdge* end, Vertex* v) : GenericIterator(start, end, v) {
}

/*******************************************
 * DCEL::Vertex::ConstIncidentFaceIterator *
 *******************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Vertex::ConstIncidentFaceIterator::ConstIncidentFaceIterator() : ConstGenericIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un IncidentFaceIterator.
 *
 * Inizializza un ConstIncidentFaceIterator pari all'IncidentFaceIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Vertex::ConstIncidentFaceIterator::ConstIncidentFaceIterator(const Dcel::Vertex::IncidentFaceIterator& it) : ConstGenericIterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstIncidentFaceIterator
 * @return La faccia incidente sull'half edge puntato dall'iteratore
 */
inline const Dcel::Face* Dcel::Vertex::ConstIncidentFaceIterator::operator * () const {
    return pos->getFace();
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstIncidentFaceIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e il vertice su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Vertex nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge (non per forza uscente dal vertice) di start
 * @param[in] end: half edge (non per forza uscente dal vertice) di end
 * @param[in] v: vertice su cui vengono iterate le facce incidenti
 */
inline Dcel::Vertex::ConstIncidentFaceIterator::ConstIncidentFaceIterator(const HalfEdge* start, const HalfEdge* end, const Vertex* v) : ConstGenericIterator(start, end, v) {
}

#endif // DCEL_VERTEX_ITERATORS_H
