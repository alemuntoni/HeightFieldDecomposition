/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_ITERATORS_H
#define DCEL_ITERATORS_H

//#include "dcel_struct.h"

class Dcel;

/**
 * \~Italian
 * @class Dcel::VertexIterator
 * @brief Iteratore che permette di ciclare sul vettore dei vertici della Dcel.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::VertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
 *     Dcel::Vertex* v = *vit;
 *     // operazioni su v
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::Vertex precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore non garantisce l'immutabilità della Dcel e quindi non è possibile utilizzarlo su const Dcel.
 * Per const Dcel, vedere Dcel::ConstVertexIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::VertexIterator {

        friend class Dcel;

    public:
        //Constructors
        VertexIterator();

        //Public Operators
        Dcel::Vertex* operator * () const;
        bool operator == (const VertexIterator& otherIterator) const;
        bool operator != (const VertexIterator& otherIterator) const;

        VertexIterator operator ++ ();
        VertexIterator operator ++ (int);
        VertexIterator operator -- ();
        VertexIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore dei vertici della Dcel. */
        const std::vector<Dcel::Vertex*> *vector;
        //Protected Constructor
        VertexIterator(unsigned int it, const std::vector<Dcel::Vertex*> &v);
};

/**
 * \~Italian
 * @class Dcel::ConstVertexIterator
 * @brief Iteratore che permette di ciclare sul vettore dei vertici della Dcel,
 * garantendo l'immutabilità della Dcel stessa.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::ConstVertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
 *     const Dcel::Vertex* v = *vit;
 *     // operazioni const su v
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::Vertex precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore garantisce l'immutabilità della Dcel e quindi è possibile utilizzarlo su const Dcel.
 * Se si ha necessità di modificare i vertici presenti nella Dcel, usare Dcel::VertexIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::ConstVertexIterator {

        friend class Dcel;

    public:
        //Constructors
        ConstVertexIterator();
        ConstVertexIterator(const Dcel::VertexIterator& it);

        //Public Operators
        const Dcel::Vertex* operator * () const;
        bool operator == (const ConstVertexIterator& otherIterator) const;
        bool operator != (const ConstVertexIterator& otherIterator) const;

        ConstVertexIterator operator ++ ();
        ConstVertexIterator operator ++ (int);
        ConstVertexIterator operator -- ();
        ConstVertexIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore dei vertici della Dcel. */
        const std::vector<Dcel::Vertex*> *vector;
        //Protected Constructor
        ConstVertexIterator(unsigned int it, const std::vector<Dcel::Vertex*> &v);
};

/**
 * \~Italian
 * @class Dcel::HalfEdgeIterator
 * @brief Iteratore che permette di ciclare sul vettore degli half edge della Dcel.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::HalfEdgeIterator heit = d.halfEdgeBegin(); heit != d.halfEdgeEnd(); ++heit){
 *     Dcel::HalfEdge* he = *heit;
 *     // operazioni su he
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::HalfEdge precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore non garantisce l'immutabilità della Dcel e quindi non è possibile utilizzarlo su const Dcel.
 * Per const Dcel, vedere Dcel::ConstHalfEdgeIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::HalfEdgeIterator {

        friend class Dcel;

    public:
        //Constructors
        HalfEdgeIterator();

        //Public Operators
        Dcel::HalfEdge* operator * () const;
        bool operator == (const HalfEdgeIterator& otherIterator) const;
        bool operator != (const HalfEdgeIterator& otherIterator) const;

        HalfEdgeIterator operator ++ ();
        HalfEdgeIterator operator ++ (int);
        HalfEdgeIterator operator -- ();
        HalfEdgeIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore di half edge della Dcel. */
        const std::vector<Dcel::HalfEdge*> *vector;
        //Protected Constructor
        HalfEdgeIterator(unsigned int it, const std::vector<Dcel::HalfEdge*> &v);
};

/**
 * \~Italian
 * @class Dcel::ConstHalfEdgeIterator
 * @brief Iteratore che permette di ciclare sul vettore degli half edge della Dcel,
 * garantendo l'immutabilità della Dcel stessa.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::ConstHalfEdgeIterator heit = d.halfEdgeBegin(); heit != d.halfEdgeEnd(); ++heit){
 *     const Dcel::HalfEdge* he = *heit;
 *     // operazioni const su he
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::HalfEdge precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore garantisce l'immutabilità della Dcel e quindi è possibile utilizzarlo su const Dcel.
 * Se si ha necessità di modificare gli half edge presenti nella Dcel, usare Dcel::HalfEdgeIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::ConstHalfEdgeIterator {

        friend class Dcel;

    public:
        //Constructors
        ConstHalfEdgeIterator();
        ConstHalfEdgeIterator(const Dcel::HalfEdgeIterator& it);

        //Public Operators
        const Dcel::HalfEdge* operator * () const;
        bool operator == (const ConstHalfEdgeIterator& otherIterator) const;
        bool operator != (const ConstHalfEdgeIterator& otherIterator) const;

        ConstHalfEdgeIterator operator ++ ();
        ConstHalfEdgeIterator operator ++ (int);
        ConstHalfEdgeIterator operator -- ();
        ConstHalfEdgeIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore di half edge della Dcel. */
        const std::vector<Dcel::HalfEdge*> *vector;
        //Protected Constructor
        ConstHalfEdgeIterator(unsigned int it, const std::vector<Dcel::HalfEdge*> &v);
};

/**
 * \~Italian
 * @class Dcel::FaceIterator
 * @brief Iteratore che permette di ciclare sul vettore delle facce della Dcel.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::FaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
 *     Dcel::Face* f = *fit;
 *     // operazioni su f
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::Face precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore non garantisce l'immutabilità della Dcel e quindi non è possibile utilizzarlo su const Dcel.
 * Per const Dcel, vedere Dcel::ConstFaceIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::FaceIterator {

        friend class Dcel;

    public:
        //Constructors
        FaceIterator();

        //Public Operators
        Dcel::Face* operator * () const;
        bool operator == (const FaceIterator& otherIterator) const;
        bool operator != (const FaceIterator& otherIterator) const;

        FaceIterator operator ++ ();
        FaceIterator operator ++ (int);
        FaceIterator operator -- ();
        FaceIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore delle facce della Dcel. */
        const std::vector<Dcel::Face*> *vector;
        //Protected Constructor
        FaceIterator(unsigned int it, const std::vector<Dcel::Face*> &v);
};

/**
 * \~Italian
 * @class Dcel::ConstFaceIterator
 * @brief Iteratore che permette di ciclare sul vettore delle facce della Dcel,
 * garantendo l'immutabilità della Dcel stessa.
 *
 * Come utilizzare questo iteratore data una Dcel d:
 *
 * \code{.cpp}
 * for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
 *     const Dcel::Face* f = *fit;
 *     // operazioni const su f
 * }
 * \endcode
 *
 * L'iteratore si occupa di saltare automaticamente tutti i Dcel::Face precedentemente eliminati e quindi settati a nullptr.
 * Questo iteratore garantisce l'immutabilità della Dcel e quindi è possibile utilizzarlo su const Dcel.
 * Se si ha necessità di modificare le facce presenti nella Dcel, usare Dcel::FaceIterator.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::ConstFaceIterator {

        friend class Dcel;

    public:
        //Constructors
        ConstFaceIterator();
        ConstFaceIterator(const Dcel::FaceIterator& it);

        //Public Operators
        const Dcel::Face* operator * () const;
        bool operator == (const ConstFaceIterator& otherIterator) const;
        bool operator != (const ConstFaceIterator& otherIterator) const;

        ConstFaceIterator operator ++ ();
        ConstFaceIterator operator ++ (int);
        ConstFaceIterator operator -- ();
        ConstFaceIterator operator -- (int);

    protected:
        //Protected Attributes
        unsigned int iterator; /**< \~Italian @brief Iteratore vero e proprio sul vettore delle facce della Dcel. */
        const std::vector<Dcel::Face*> *vector;
        //Protected Constructor
        ConstFaceIterator(unsigned int it, const std::vector<Dcel::Face*> &v);
};

/*****************************
 * Dcel::VertexIterator *
 *****************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::VertexIterator::VertexIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constVertexIterator
 * @return Il vertice puntato dall'iteratore
 */
inline Dcel::Vertex* Dcel::VertexIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constVertexIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::VertexIterator::operator == (const VertexIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constVertexIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::VertexIterator::operator != (const VertexIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constVertexIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::VertexIterator Dcel::VertexIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constVertexIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::VertexIterator Dcel::VertexIterator::operator ++ (int) {
    Dcel::VertexIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constVertexIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::VertexIterator Dcel::VertexIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constVertexIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::VertexIterator Dcel::VertexIterator::operator -- (int) {
    Dcel::VertexIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constVertexIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di vertici della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di vertici della Dcel
 */
inline Dcel::VertexIterator::VertexIterator(unsigned int it, const std::vector<Vertex*>& v) : iterator(it), vector(&v) {
}

/*****************************
 * Dcel::constVertexIterator *
 *****************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::ConstVertexIterator::ConstVertexIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un vertexIterator.
 *
 * Inizializza un constVertexIterator pari al vertexIterator passato in input.
 *
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::ConstVertexIterator::ConstVertexIterator(const Dcel::VertexIterator& it) : iterator(it.iterator), vector(it.vector) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constVertexIterator
 * @return Il vertice puntato dall'iteratore
 */
inline const Dcel::Vertex* Dcel::ConstVertexIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constVertexIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::ConstVertexIterator::operator == (const ConstVertexIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constVertexIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::ConstVertexIterator::operator != (const ConstVertexIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constVertexIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::ConstVertexIterator Dcel::ConstVertexIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constVertexIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::ConstVertexIterator Dcel::ConstVertexIterator::operator ++ (int) {
    Dcel::ConstVertexIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constVertexIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::ConstVertexIterator Dcel::ConstVertexIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constVertexIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::ConstVertexIterator Dcel::ConstVertexIterator::operator -- (int) {
    Dcel::ConstVertexIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constVertexIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di vertici della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di vertici della Dcel
 */
inline Dcel::ConstVertexIterator::ConstVertexIterator(unsigned int it, const std::vector<Vertex*>& v) : iterator(it), vector(&v) {
}

/*******************************
 * Dcel::HalfEdgeIterator *
 *******************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::HalfEdgeIterator::HalfEdgeIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline Dcel::HalfEdge* Dcel::HalfEdgeIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constHalfEdgeIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::HalfEdgeIterator::operator == (const HalfEdgeIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constHalfEdgeIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::HalfEdgeIterator::operator != (const HalfEdgeIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::HalfEdgeIterator Dcel::HalfEdgeIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::HalfEdgeIterator Dcel::HalfEdgeIterator::operator ++ (int) {
    Dcel::HalfEdgeIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::HalfEdgeIterator Dcel::HalfEdgeIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::HalfEdgeIterator Dcel::HalfEdgeIterator::operator -- (int) {
    Dcel::HalfEdgeIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constHalfEdgeIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di half edge della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di half edge della Dcel
 */
inline Dcel::HalfEdgeIterator::HalfEdgeIterator(unsigned int it, const std::vector<HalfEdge*>& v) : iterator(it) , vector(&v){
}

/*******************************
 * Dcel::ConstHalfEdgeIterator *
 *******************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::ConstHalfEdgeIterator::ConstHalfEdgeIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un halfEdgeIterator.
 *
 * Inizializza un constHalfEdgeIterator pari al halfEdgeIterator passato in input.
 *
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::ConstHalfEdgeIterator::ConstHalfEdgeIterator(const Dcel::HalfEdgeIterator& it) : iterator(it.iterator), vector(it.vector) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constHalfEdgeIterator
 * @return L'half edge puntato dall'iteratore
 */
inline const Dcel::HalfEdge* Dcel::ConstHalfEdgeIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constHalfEdgeIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::ConstHalfEdgeIterator::operator == (const ConstHalfEdgeIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constHalfEdgeIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::ConstHalfEdgeIterator::operator != (const ConstHalfEdgeIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constHalfEdgeIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeIterator::operator ++ (int) {
    Dcel::ConstHalfEdgeIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constHalfEdgeIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::ConstHalfEdgeIterator Dcel::ConstHalfEdgeIterator::operator -- (int) {
    Dcel::ConstHalfEdgeIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constHalfEdgeIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di half edge della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di half edge della Dcel
 */
inline Dcel::ConstHalfEdgeIterator::ConstHalfEdgeIterator(unsigned int it, const std::vector<HalfEdge*>& v) : iterator(it), vector(&v) {
}

/***************************
 * Dcel::FaceIterator *
 ***************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::FaceIterator::FaceIterator() {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constFaceIterator
 * @return Il vertice puntato dall'iteratore
 */
inline Dcel::Face* Dcel::FaceIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constFaceIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::FaceIterator::operator == (const FaceIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constFaceIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::FaceIterator::operator != (const FaceIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constFaceIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::FaceIterator Dcel::FaceIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constFaceIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::FaceIterator Dcel::FaceIterator::operator ++ (int) {
    Dcel::FaceIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constFaceIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::FaceIterator Dcel::FaceIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constFaceIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::FaceIterator Dcel::FaceIterator::operator -- (int) {
    Dcel::FaceIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constFaceIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di facce della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di facce della Dcel
 */
inline Dcel::FaceIterator::FaceIterator(unsigned int it, const std::vector<Face*>& v) : iterator(it), vector(&v) {
}

/***************************
 * Dcel::constFaceIterator *
 ***************************/

//Constructors

/**
 * \~Italian
 * @brief Costruttore vuoto.
 * Un iteratore inizializzato con questo costruttore non è utilizzabile.
 */
inline Dcel::ConstFaceIterator::ConstFaceIterator() {
}

/**
 * \~Italian
 * @brief Costruttore di copia da un faceIterator.
 *
 * Inizializza un constFaceIterator pari al faceIterator passato in input.
 *
 * @param[in] it: iteratore di cui ne verrà fatta una copia
 */
inline Dcel::ConstFaceIterator::ConstFaceIterator(const Dcel::FaceIterator& it) : iterator(it.iterator), vector(it.vector) {
}

//Public Operators

/**
 * \~Italian
 * @brief Operatore di dereferenziazione del constFaceIterator
 * @return Il vertice puntato dall'iteratore
 */
inline const Dcel::Face* Dcel::ConstFaceIterator::operator * () const {
    return (*vector)[iterator];
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra constFaceIterator.
 *
 * Normale operatore di uguaglianza tra std::vector::iterator.
 *
 * @param[in] otherIterator: iteratore con cui è verificata l'uguaglianza con this
 * @return True se gli iteratori sono uguali, false altrimenti
 */
inline bool Dcel::ConstFaceIterator::operator == (const ConstFaceIterator& otherIterator) const {
    return iterator == otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra constFaceIterator
 * @param[in] otherIterator: iteratore con cui è verificata la disuguaglianza con this
 * @return True se gli iteratori sono diversi, false altrimenti
 */
inline bool Dcel::ConstFaceIterator::operator != (const ConstFaceIterator& otherIterator) const {
    return iterator != otherIterator.iterator;
}

/**
 * \~Italian
 * @brief Operatore di incremento prefisso del constFaceIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena incrementato
 */
inline Dcel::ConstFaceIterator Dcel::ConstFaceIterator::operator ++ () {
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di incremento postfisso del constFaceIterator.
 *
 * Normale operatore di incremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere incrementato
 */
inline Dcel::ConstFaceIterator Dcel::ConstFaceIterator::operator ++ (int) {
    Dcel::ConstFaceIterator old = *this;
    do {
        ++iterator;
    } while (iterator != vector->size() && (*vector)[iterator] == nullptr);
    return old;
}

/**
 * \~Italian
 * @brief Operatore di decremento prefisso del constFaceIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore appena decrementato
 */
inline Dcel::ConstFaceIterator Dcel::ConstFaceIterator::operator -- () {
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di decremento postfisso del constFaceIterator.
 *
 * Normale operatore di decremento prefisso di un std::vector::iterator.
 *
 * @return L'iteratore prima di essere decrementato
 */
inline Dcel::ConstFaceIterator Dcel::ConstFaceIterator::operator -- (int) {
    Dcel::ConstFaceIterator old = *this;
    do {
        --iterator;
    } while (iterator != 0 && (*vector)[iterator] == nullptr);
    return old;
}

//Protected Constructor

/**
 * \~Italian
 * @brief Costruttore di un constFaceIterator.
 *
 * Inizializza l'iteratore alla posizione di partenza del vettore di facce della Dcel.
 * Per questioni di sicurezza e di robustezza del codice questo costruttore non è direttamente richiamabile
 * dal programmatore. Tuttavia, questo costruttore viene richiamato dalla friend class Dcel nei vari metodi
 * begin, che inizializzano correttamente l'iteratore e che possono essere utilizzati dal programmatore per
 * l'inizializzazione dell'iteratore.
 *
 * @param[in] it: iteratore sul vettore di facce della Dcel
 */
inline Dcel::ConstFaceIterator::ConstFaceIterator(unsigned int it, const std::vector<Dcel::Face*> &v) : iterator(it), vector(&v) {
}

#endif // DCEL_ITERATORS_H

