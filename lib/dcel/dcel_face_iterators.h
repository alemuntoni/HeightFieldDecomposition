/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_FACE_ITERATORS_H
#define DCEL_FACE_ITERATORS_H

#include "dcel_face.h"
#include "dcel_half_edge.h"

/**
 * \~Italian
 * @class Dcel::Face::ConstInnerHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sul vettore degli inner half edge associati alla faccia, garantendone
 * l'immutabilità.
 * Ogni inner half edge è associato ad un buco presente all'interno della faccia.
 * Una volta che si ha a disposizione un inner half edge, è possibile ciclare sul bordo del buco
 * mediante delle semplici operazioni di next, oppure utilizzando un ConstIncidentHalfEdgeIterator opportunamente inizializzato.
 *
 * È possibile utilizzare l'iteratore esattamente come si utilizza un iteratore su un std::vector.
 * Per esempio, data una faccia Dcel::Face* f:
 *
 * \code{.cpp}
 * for (Dcel::Face::ConstInnerHalfEdgeIterator heit = f->innerHalfEgeBegin(); heit != f->innerHalfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * Questo iteratore garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi è possibile
 * utilizzarlo solamente su const Dcel::Face. Per poter effettuare modifiche, vedere Dcel::Face::InnerHalfEdgeIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face::ConstInnerHalfEdgeIterator {

        friend class Dcel::Face;

    public:
        //Constructors
        ConstInnerHalfEdgeIterator();
        ConstInnerHalfEdgeIterator(const Dcel::Face::InnerHalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * ()                                 const;
        bool operator == (const ConstInnerHalfEdgeIterator& otherIterator)  const;
        bool operator != (const ConstInnerHalfEdgeIterator& otherIterator)  const;

        ConstInnerHalfEdgeIterator operator ++ ();
        ConstInnerHalfEdgeIterator operator ++ (int);
        ConstInnerHalfEdgeIterator operator -- ();
        ConstInnerHalfEdgeIterator operator -- (int);

    protected:
        //Protected Attributes
        std::vector<Dcel::HalfEdge*>::const_iterator iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore degli inner half edge della faccia. */

        //Protected Constructor
        ConstInnerHalfEdgeIterator(const std::vector<Dcel::HalfEdge*>::const_iterator& it);
};

/**
 * \~Italian
 * @class Dcel::Face::IncidentHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge incidenti ad una faccia.
 *
 * Partendo da un half edge di start e un half edge di end (non compreso), entrambi
 * incidenti alla faccia su cui è inizializzato l'iteratore, permette di
 * ciclare su tutti gli half edge compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione \c next() sull'half edge,
 * mentre le operazioni di decremento eseguono un'operazione di \c prev().
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * hanno settato correttamente le seguenti relazioni:
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * Più precisamente, è necessario che una volta effettuate diverse operazioni di next (o prev), si arrivi
 * di nuovo all'half edge di partenza (o a un eventuale halg edge finale settato all'inizializzazione).
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso antiorario o orario (operatore \--)
 * su tutti gli half edge incidenti ad una data faccia \c f. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Face::IncidentHalfEdgeIterator heit = f->incidentHalfEdgeBegin(); heit != f->incidentHalfEdgeEnd(); ++heit){
 *     Dcel::HalfEdge* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge incidenti compresi tra due dati half edge. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Face::incidentHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Face. Per const Dcel::Face, vedere Dcel::Face::ConstIncidentHalfEdgeIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face::IncidentHalfEdgeIterator {

        friend class Dcel::Face;

    public:
        // Constructor
        IncidentHalfEdgeIterator();

        //Public Operators
        Dcel::HalfEdge* operator * ()                                       const;
        bool operator == (const IncidentHalfEdgeIterator& otherIterator)    const;
        bool operator != (const IncidentHalfEdgeIterator& otherIterator)    const;

        IncidentHalfEdgeIterator operator ++ ();
        IncidentHalfEdgeIterator operator ++ (int);
        IncidentHalfEdgeIterator operator -- ();
        IncidentHalfEdgeIterator operator -- (int);

    protected:
        //Protected Attributes
        Dcel::Face*     f;      /**< \~Italian @brief Faccia su cui vengono iterati gli half edge incidenti */
        Dcel::HalfEdge* start;  /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        Dcel::HalfEdge* pos;    /**< \~Italian @brief Posizione attuale dell'iteratore */
        Dcel::HalfEdge* end;    /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        IncidentHalfEdgeIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Face* f);
};

/**
 * \~Italian
 * @class Dcel::Face::ConstIncidentHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sugli half edge incidenti ad una faccia, garantendone l'immutabilità.
 *
 * Partendo da un half edge di start e un half edge di end (non compreso), entrambi
 * incidenti alla faccia su cui è inizializzato l'iteratore, permette di
 * ciclare su tutti gli half edge compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione \c next() sull'half edge,
 * mentre le operazioni di decremento eseguono un'operazione di \c prev().
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * hanno settato correttamente le seguenti relazioni:
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 *
 * Più precisamente, è necessario che una volta effettuate diverse operazioni di next (o prev), si arrivi
 * di nuovo all'half edge di partenza (o a un eventuale halg edge finale settato all'inizializzazione).
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso antiorario o orario (operatore \--)
 * su tutti gli half edge incidenti ad una data faccia \c f. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = f->incidentHalfEdgeBegin(); heit != f->incidentHalfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti gli half edge incidenti compresi tra due dati half edge. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Face::incidentHalfEdgeBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi si può utilizzare solo
 * su const Dcel::Face. Per Dcel::Face, vedere Dcel::Face::IncidentHalfEdgeIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face::ConstIncidentHalfEdgeIterator {

        friend class Dcel::Face;

    public:
        //Constructors
        ConstIncidentHalfEdgeIterator();
        ConstIncidentHalfEdgeIterator(const Dcel::Face::IncidentHalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * ()                                     const;
        bool operator == (const ConstIncidentHalfEdgeIterator& otherIterator)   const;
        bool operator != (const ConstIncidentHalfEdgeIterator& otherIterator)   const;

        ConstIncidentHalfEdgeIterator operator ++ ();
        ConstIncidentHalfEdgeIterator operator ++ (int);
        ConstIncidentHalfEdgeIterator operator -- ();
        ConstIncidentHalfEdgeIterator operator -- (int);

    protected:
        //Protected Attributes
        const Dcel::Face*     f;        /**< \~Italian @brief Faccia su cui vengono iterati gli half edge incidenti */
        const Dcel::HalfEdge* start;    /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        const Dcel::HalfEdge* pos;      /**< \~Italian @brief Posizione attuale dell'iteratore */
        const Dcel::HalfEdge* end;      /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        ConstIncidentHalfEdgeIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Face* f);
};

/**
 * \~Italian
 * @class Dcel::Face::IncidentVertexIterator
 * @brief Iteratore che permette di ciclare sui vertici incidenti ad una faccia.
 *
 * Partendo da un half edge di start e un half edge di end (non compreso), entrambi
 * incidenti alla faccia su cui è inizializzato l'iteratore, permette di
 * ciclare su tutti i to vertex compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione \c next() sull'half edge,
 * le operazioni di decremento eseguono un'operazione di \c prev() , e l'operazione di
 * dereferenziazione restituisce il toVertex dell'half edge.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * hanno settato correttamente le seguenti relazioni:
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 * - to vertex (sia per operazioni di incremento che per operazioni di decremento)
 *
 * Più precisamente, è necessario che una volta effettuate diverse operazioni di next (o prev), si arrivi
 * di nuovo all'half edge di partenza (o a un eventuale halg edge finale settato all'inizializzazione).
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso antiorario o orario (operatore \--)
 * su tutti gli half edge incidenti ad una data faccia \c f. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Face::IncidentVertexIterator vit = f->incidentVertexBegin(); vit != f->incidentVertexEnd(); ++vit){
 *     Dcel::Vertex* v = *vit;
 *     // operazioni su v
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti i vertici incidenti compresi tra due dati half edge. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Face::incidentVertexBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore non garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi non è possibile
 * utilizzarlo su const Dcel::Face. Per const Dcel::Face, vedere Dcel::Face::ConstIncidentVertexIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face::IncidentVertexIterator {

        friend class Dcel::Face;

    public:
        //Constructor
        IncidentVertexIterator();

        //Public Operators
        Dcel::Vertex* operator * ()                                     const;
        bool operator == (const IncidentVertexIterator& otherIterator)  const;
        bool operator != (const IncidentVertexIterator& otherIterator)  const;

        IncidentVertexIterator operator ++ ();
        IncidentVertexIterator operator ++ (int);
        IncidentVertexIterator operator -- ();
        IncidentVertexIterator operator -- (int);

    protected:
        //Protected Attributes
        Dcel::Face*     f;      /**< \~Italian @brief Faccia su cui vengono iterati gli half edge incidenti */
        Dcel::HalfEdge* start;  /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        Dcel::HalfEdge* pos;    /**< \~Italian @brief Posizione attuale dell'iteratore */
        Dcel::HalfEdge* end;    /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        IncidentVertexIterator(Dcel::HalfEdge* start, Dcel::HalfEdge* end, Dcel::Face* f);
};

/**
 * \~Italian
 * @class Dcel::Face::ConstIncidentVertexIterator
 * @brief Iteratore che permette di ciclare sui vertici incidenti ad una faccia, garantendone l'immutabilità.
 *
 * Partendo da un half edge di start e un half edge di end (non compreso), entrambi
 * incidenti alla faccia su cui è inizializzato l'iteratore, permette di
 * ciclare su tutti i to vertex compresi tra start e end. \n
 *
 * Le operazioni di incremento eseguono un'operazione \c next() sull'half edge,
 * le operazioni di decremento eseguono un'operazione di \c prev() , e l'operazione di
 * dereferenziazione restituisce il toVertex dell'half edge.
 *
 * Per questi motivi, utilizzare questo iteratore solamente quando tutti gli half edge incidenti
 * hanno settato correttamente le seguenti relazioni:
 * - next (per le operazioni di incremento)
 * - prev (per le operazioni di decremento)
 * - to vertex (sia per operazioni di incremento che per operazioni di decremento)
 *
 * Più precisamente, è necessario che una volta effettuate diverse operazioni di next (o prev), si arrivi
 * di nuovo all'half edge di partenza (o a un eventuale halg edge finale settato all'inizializzazione).
 *
 * È possibile utilizzare l'iteratore in diversi modi. Il più semplice è ciclare in senso antiorario o orario (operatore \--)
 * su tutti gli half edge incidenti ad una data faccia \c f. In questo caso, sarà sufficiente scrivere:
 *
 * \code{.cpp}
 * for (Dcel::Face::ConstIncidentVertexIterator vit = f->incidentVertexBegin(); vit != f->incidentVertexEnd(); ++vit){
 *     const Dcel::Vertex* v = *vit;
 *     // operazioni const su v
 * }
 * \endcode
 *
 * È però possibile anche visitare tutti i vertici incidenti compresi tra due dati half edge. In questo caso sarà necessario
 * passare gli half edge estremi dell'intervallo all'inizializzatore Dcel::Face::incidentVertexBegin(). Per i dettagli su come funzionano queste
 * inizializzazioni si rimanda alla descrizione dei metodi stessi.
 *
 * Questo iteratore garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi è possibile utilizzarlo solamente
 * su const Dcel::Face. Per Dcel::Face, vedere Dcel::Face::IncidentVertexIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face::ConstIncidentVertexIterator {

        friend class Dcel::Face;

    public:
        //Constructors
        ConstIncidentVertexIterator();
        ConstIncidentVertexIterator(const Dcel::Face::IncidentVertexIterator& it);

        //Public Operators
        const Dcel::Vertex* operator * ()                                   const;
        bool operator == (const ConstIncidentVertexIterator& otherIterator) const;
        bool operator != (const ConstIncidentVertexIterator& otherIterator) const;

        ConstIncidentVertexIterator operator ++ ();
        ConstIncidentVertexIterator operator ++ (int);
        ConstIncidentVertexIterator operator -- ();
        ConstIncidentVertexIterator operator -- (int);

    protected:
        //Protected Attributes
        const Dcel::Face*     f;        /**< \~Italian @brief Faccia su cui vengono iterati gli half edge incidenti */
        const Dcel::HalfEdge* start;    /**< \~Italian @brief Half edge dal quale è partito l'iteratore */
        const Dcel::HalfEdge* pos;      /**< \~Italian @brief Posizione attuale dell'iteratore */
        const Dcel::HalfEdge* end;      /**< \~Italian @brief Half edge sul quale termina l'iteratore */

        //Protected Constructor
        ConstIncidentVertexIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Face* f);
};

/******************************************
 * Dcel::Face::ConstInnerHalfEdgeIterator *
 ******************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator::ConstInnerHalfEdgeIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un InnerHalfEdgeIterator.
 *
 * Inizializza un ConstInnerHalfEdgeIterator pari all'InnerHalfEdgeIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator::ConstInnerHalfEdgeIterator(const Dcel::Face::InnerHalfEdgeIterator& it) : iterator(it) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstInnerHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge* Dcel::Face::ConstInnerHalfEdgeIterator::operator * () const {
    return *iterator;
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra ConstInnerHalfEdgeIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Face::ConstInnerHalfEdgeIterator::operator == (const Dcel::Face::ConstInnerHalfEdgeIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra ConstInnerHalfEdgeIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Face::ConstInnerHalfEdgeIterator::operator != (const Dcel::Face::ConstInnerHalfEdgeIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del ConstInnerHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::ConstInnerHalfEdgeIterator::operator ++ () {
    return ++iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del ConstInnerHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::ConstInnerHalfEdgeIterator::operator ++ (int) {
    Dcel::Face::ConstInnerHalfEdgeIterator old = iterator;
    ++iterator;
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del ConstInnerHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::ConstInnerHalfEdgeIterator::operator -- () {
    return --iterator;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del ConstInnerHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::ConstInnerHalfEdgeIterator::operator -- (int) {
    Dcel::Face::ConstInnerHalfEdgeIterator old = iterator;
    --iterator;
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstInnerHalfEdgeIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di inner half edge della faccia.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Face nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di inner half edge della faccia
 */
inline Dcel::Face::ConstInnerHalfEdgeIterator::ConstInnerHalfEdgeIterator(const std::vector<Dcel::HalfEdge*>::const_iterator& it) : iterator(it) {
}

/****************************************
 * Dcel::Face::IncidentHalfEdgeIterator *
 ****************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Face::IncidentHalfEdgeIterator::IncidentHalfEdgeIterator() : f(nullptr), start(nullptr), pos(nullptr), end(nullptr) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'IncidentHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline Dcel::HalfEdge* Dcel::Face::IncidentHalfEdgeIterator::operator * () const {
    return pos;
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra IncidentHalfEdgeIterator.
 *
 * Due IncidentHalfEdgeIterator sono considerati uguali se:
 * - puntano allo stesso half edge;
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Face::IncidentHalfEdgeIterator::operator == (const Dcel::Face::IncidentHalfEdgeIterator& otherIterator) const {
    if (this->pos == otherIterator.pos && this->f == otherIterator.f) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra IncidentHalfEdgeIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Face::IncidentHalfEdgeIterator::operator != (const Dcel::Face::IncidentHalfEdgeIterator& otherIterator) const {
    return !(*this == otherIterator);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::IncidentHalfEdgeIterator::operator ++ () {
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::IncidentHalfEdgeIterator::operator ++ (int) {
    IncidentHalfEdgeIterator old_value = *this;
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::IncidentHalfEdgeIterator::operator -- () {
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'IncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::IncidentHalfEdgeIterator::operator -- (int) {
    IncidentHalfEdgeIterator old_value = *this;
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un IncidentHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e la faccia su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Face nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge di start
 * @param[in] end: half edge di end
 * @param[in] f: faccia su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Face::IncidentHalfEdgeIterator::IncidentHalfEdgeIterator(HalfEdge* start, HalfEdge* end, Face* f) : f(f), start(start), pos(start), end(end) {
}

/*********************************************
 * Dcel::Face::ConstIncidentHalfEdgeIterator *
 *********************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator() : f(nullptr), start(nullptr), pos(nullptr), end(nullptr) {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un IncidentHalfEdgeIterator.
 *
 * Inizializza un ConstIncidentHalfEdgeIterator pari all'IncidentHalfEdgeIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator(const Dcel::Face::IncidentHalfEdgeIterator& it) : f(it.f), start (it.start), pos(it.pos), end(it.end) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del ConstIncidentHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge *Dcel::Face::ConstIncidentHalfEdgeIterator::operator * () const {
    return pos;
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra ConstIncidentHalfEdgeIterator.
 *
 * Due ConstIncidentHalfEdgeIterator sono considerati uguali se:
 * - puntano allo stesso half edge;
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Face::ConstIncidentHalfEdgeIterator::operator == (const ConstIncidentHalfEdgeIterator& otherIterator) const {
    if (this->pos == otherIterator.pos && this->f == otherIterator.f) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra ConstIncidentHalfEdgeIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Face::ConstIncidentHalfEdgeIterator::operator !=(const ConstIncidentHalfEdgeIterator& otherIterator) const {
    return !(*this == otherIterator);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::ConstIncidentHalfEdgeIterator::operator ++ () {
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::ConstIncidentHalfEdgeIterator::operator ++ (int) {
    ConstIncidentHalfEdgeIterator old_value = *this;
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::ConstIncidentHalfEdgeIterator::operator -- () {
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del ConstIncidentHalfEdgeIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::ConstIncidentHalfEdgeIterator::operator -- (int) {
    ConstIncidentHalfEdgeIterator old_value = *this;
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstIncidentHalfEdgeIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e la faccia su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Face nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge di start
 * @param[in] end: half edge di end
 * @param[in] f: faccia su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Face::ConstIncidentHalfEdgeIterator::ConstIncidentHalfEdgeIterator(const HalfEdge* start, const HalfEdge* end, const Face* f) : f(f), start(start), pos(start), end(end) {
}

/**************************************
 * Dcel::Face::IncidentVertexIterator *
 **************************************/

//Constructor

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Face::IncidentVertexIterator::IncidentVertexIterator() : f(nullptr), start(nullptr), pos(nullptr), end(nullptr) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'IncidentVertexIterator
 * @return Il to vertex dell'half edge puntato dall'iteratore
 */
inline Dcel::Vertex* Dcel::Face::IncidentVertexIterator::operator * () const {
    return this->pos->getToVertex();
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra IncidentVertexIterator.
 *
 * Due IncidentVertexIterator sono considerati uguali se:
 * - puntano allo stesso vertice (posizione sullo stesso half edge);
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Face::IncidentVertexIterator::operator == (const Dcel::Face::IncidentVertexIterator& otherIterator) const {
    if (this->pos == otherIterator.pos && this->f == otherIterator.f) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra IncidentVertexIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return true se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Face::IncidentVertexIterator::operator != (const Dcel::Face::IncidentVertexIterator& otherIterator) const {
    return !(*this == otherIterator);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'IncidentVertexIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Face::IncidentVertexIterator Dcel::Face::IncidentVertexIterator::operator ++ () {
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'IncidentVertexIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Face::IncidentVertexIterator Dcel::Face::IncidentVertexIterator::operator ++ (int) {
    IncidentVertexIterator old_value = *this;
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'IncidentVertexIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Face::IncidentVertexIterator Dcel::Face::IncidentVertexIterator::operator -- () {
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'IncidentVertexIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Face::IncidentVertexIterator Dcel::Face::IncidentVertexIterator::operator -- (int) {
    IncidentVertexIterator old_value = *this;
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un IncidentVertexIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e la faccia su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Face nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge di start
 * @param[in] end: half edge di end
 * @param[in] f: faccia su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Face::IncidentVertexIterator::IncidentVertexIterator(HalfEdge* start, HalfEdge* end, Face* f) : f(f), start(start), pos(start), end(end) {
}

/*******************************************
 * Dcel::Face::ConstIncidentVertexIterator *
 *******************************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::Face::ConstIncidentVertexIterator::ConstIncidentVertexIterator() : f(nullptr), start(nullptr), pos(nullptr), end(nullptr) {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un IncidentVertexIterator.
 *
 * Inizializza un ConstIncidentVertexIterator pari all'IncidentVertexIterator passato in input.
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::Face::ConstIncidentVertexIterator::ConstIncidentVertexIterator(const Dcel::Face::IncidentVertexIterator& it) : f(it.f), start(it.start), pos(it.pos), end(it.end) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione dell'ConstIncidentVertexIterator
 * @return Il to vertex dell'half edge puntato dall'iteratore
 */
inline const Dcel::Vertex *Dcel::Face::ConstIncidentVertexIterator::operator * () const {
    return this->pos->getToVertex();
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra ConstIncidentVertexIterator.
 *
 * Due IncidentVertexIterator sono considerati uguali se:
 * - puntano allo stesso vertice (posizione sullo stesso half edge);
 * - iterano sulla stessa faccia.
 *
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::Face::ConstIncidentVertexIterator::operator == (const ConstIncidentVertexIterator& otherIterator) const {
    if (this->pos == otherIterator.pos && this->f == otherIterator.f) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra ConstIncidentVertexIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::Face::ConstIncidentVertexIterator::operator != (const ConstIncidentVertexIterator& otherIterator) const {
    return !(*this == otherIterator);
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso dell'ConstIncidentVertexIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::Face::ConstIncidentVertexIterator Dcel::Face::ConstIncidentVertexIterator::operator ++ () {
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso dell'ConstIncidentVertexIterator.
 *
 * Esegue un'operazione di \c next() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::Face::ConstIncidentVertexIterator Dcel::Face::ConstIncidentVertexIterator::operator ++ (int) {
    ConstIncidentVertexIterator old_value = *this;
    pos = pos->getNext();
    if (pos == end) pos = nullptr;
    return old_value;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso dell'ConstIncidentVertexIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::Face::ConstIncidentVertexIterator Dcel::Face::ConstIncidentVertexIterator::operator -- () {
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso dell'ConstIncidentVertexIterator.
 *
 * Esegue un'operazione di \c prev() sull'half edge. Se l'half edge ottenuto è uguale
 * all'half edge end, allora l'iteratore diventa uguale all'iteratore \c end().
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::Face::ConstIncidentVertexIterator Dcel::Face::ConstIncidentVertexIterator::operator --(int) {
    ConstIncidentVertexIterator old_value = *this;
    pos = pos->getPrev();
    if (pos == end) pos = nullptr;
    return old_value;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un ConstIncidentVertexIterator.
 *
 * Setta l'half edge di partenza, l'half edge di arrivo e la faccia su cui iterare.
 * L'iteratore viene inizializzato all'half edge start.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel::Face nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] start: half edge di start
 * @param[in] end: half edge di end
 * @param[in] f: faccia su cui vengono iterati gli half edge incidenti
 */
inline Dcel::Face::ConstIncidentVertexIterator::ConstIncidentVertexIterator(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end, const Dcel::Face* f) :f(f), start(start), pos(start), end(end) {
}

#endif // DCEL_FACE_ITERATORS_H
