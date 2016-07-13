/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_HALF_EDGE_H
#define DCEL_HALF_EDGE_H

#include "dcel_struct.h"

/**
 * \~Italian
 * @class Dcel::HalfEdge
 * @brief Classe rappresentante un half edge della Dcel.
 *
 * La classe Dcel::HalfEdge implementa un half edge della Dcel.\n
 * L'half-edge è un segmento \e orientato, che parte da un vertice di origine
 * identificato come \b fromVertex e arriva ad un vertice di destinazione detto \b toVertex.
 * Ogni half edge è caratterizzato dall'avere un half-edge gemello, identificato
 * come \b twin half edge, che ha orientamento opposto rispetto all'half-edge originale.
 * L'half-edge inoltre possiede i riferimenti agli half edge \b next e \b prev, che rappresentano
 * rispettivamente l'half-edge successivo e l'half-edge precedente seguendo la direzione indicata
 * dall'half edge stesso, e il riferimento alla faccia incidente \b face.
 *
 * Per una corretta gestione della Dcel, devono essere rispettate le seguenti regole:
 *
 * - Ogni half edge \c e ha un twin \c t, e il suo twin \c t ha come twin l'half edge \c e.
 * - Ogni half edge ha from vertex uguale al to vertex del suo twin, e viceversa.
 * - Partendo da un half edge \c e, una serie di operazioni di next (3 se si ha una Dcel di triangoli) riporta all'half edge \c e, tutti questi half edge hanno la stessa faccia incidente.
 * - Partendo da un half edge \c e, una serie di operazioni di prev (3 se si ha una Dcel di triangoli) riporta all'half edge \c e, tutti questi half edge hanno la stessa faccia incidente.
 * - Il to vertex di un half edge corrisponde al from vertex del suo next;
 * - Il from vertex di un half edge corrisponde al to vertex del suo prev.
 * - Partendo da un half edge \c e, una serie di operazioni di twin e next riporta all'half edge \c e.
 * - Partendo da un half edge \c e, una serie di operazioni di prev e twin riporta all'half edge \c e.
 *
 *
 * Le altre componenti che compongono l'half edge sono:
 * - id: intero senza segno univoco all'interno della lista degli half edge della Dcel,
 *   non modificabile dall'utente. Può essere usato per identificare la faccia all'interno
 *   della Dcel (in modo meno efficiente rispetto all'utilizzo di un puntatore);\n
 * - flag: intero personalizzabile dall'utente. \n
 *
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::HalfEdge
{
        friend class Dcel;

    public:

        /***************
        * Constructors *
        ****************/

        HalfEdge();
        HalfEdge(Dcel::Vertex* from, Dcel::Vertex* to);
        HalfEdge(Dcel::Vertex* from, Dcel::Vertex* to, Dcel::HalfEdge* twin, Dcel::HalfEdge* prev, Dcel::HalfEdge* next, Dcel::Face* face);
        ~HalfEdge();

        /************************
        * Public Inline Methods *
        *************************/

        unsigned int getId()                                const;
        int getFlag()                                       const;
        const Dcel::Vertex* getFromVertex()                 const;
        const Dcel::Vertex* getToVertex()                   const;
        const Dcel::HalfEdge* getTwin()                     const;
        const Dcel::HalfEdge* getPrev()                     const;
        const Dcel::HalfEdge* getNext()                     const;
        const Dcel::Face* getFace()                         const;
        bool operator == (const HalfEdge& otherHalfEdge)    const;
        bool operator != (const HalfEdge& otherHalfEdge)    const;
        #ifdef DEBUG
        void checkFromVertex()                              const;
        void checkToVertex()                                const;
        void checkTwin()                                    const;
        void checkPrev()                                    const;
        void checkNext()                                    const;
        void checkFace()                                    const;
        #endif
        bool isConvex()                                     const;

        void setFlag();
        void setFlag(int new_flag);
        void resetFlag();
        Dcel::Vertex* getFromVertex();
        Dcel::Vertex* getToVertex();
        Dcel::HalfEdge* getTwin();
        Dcel::HalfEdge* getPrev();
        Dcel::HalfEdge* getNext();
        Dcel::Face* getFace();
        void setFromVertex(Dcel::Vertex* newFromVertex);
        void setToVertex(Dcel::Vertex* newToVertex);
        void setTwin(Dcel::HalfEdge* newTwin);
        void setPrev(Dcel::HalfEdge* newPrev);
        void setNext(Dcel::HalfEdge* newNext);
        void setFace(Dcel::Face* newFace);

        /*****************
        * Public Methods *
        ******************/

        bool isOuterComponent() const;
        float getLength()       const;
        std::string toString()  const;


    protected:

        /**************
        * Attributes *
        **************/

        Dcel::Vertex* 	fromVertex; /**< \~Italian @brief Vertice di origine dell'half edge */
        Dcel::Vertex* 	toVertex;   /**< \~Italian @brief Vertice di destinazione dell'half edge */
        Dcel::HalfEdge* twin;       /**< \~Italian @brief Half edge gemello dell'half edge */
        Dcel::HalfEdge*	prev;       /**< \~Italian @brief Half edge precendente all'half edge */
        Dcel::HalfEdge*	next;       /**< \~Italian @brief Half edge successivo all'half edge */
        Dcel::Face* 	face;       /**< \~Italian @brief Faccia incidente all'half edge */
        unsigned int    id;         /**< \~Italian @brief Id univoco, all'interno della Dcel, associato all'a faccia'half edge */
        int             flag;       /**< \~Italian @brief Flag personalizzabile, associato all'half edge */

        /***************************
        * Protected Inline Methods *
        ****************************/

        void setId(unsigned int id);
};

/*************************
 * Public Inline Methods *
 *************************/

/**
 * \~Italian
 * @brief Restituisce l'id identificativo nella Dcel dell'half edge
 * @return L'id dell'a faccia'half edge
 */
inline unsigned int Dcel::HalfEdge::getId() const {
    return id;
}

/**
 * \~Italian
 * @brief Restituisce il flag associato all'a faccia'half edge
 * @return Il flag dell'half edge
 */
inline int Dcel::HalfEdge::getFlag() const {
    return flag;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore al vertice costante di origine dell'half edge
 * @return Il from vertex dell'half edge
 */
inline const Dcel::Vertex* Dcel::HalfEdge::getFromVertex() const {
    return fromVertex;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore al vertice costante di destinazione dell'half edge
 * @return Il to vertex dell'half edge
 */
inline const Dcel::Vertex* Dcel::HalfEdge::getToVertex() const {
    return toVertex;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge costante gemello dell'half edge
 * @return Il twin dell'half edge
 */
inline const Dcel::HalfEdge* Dcel::HalfEdge::getTwin() const {
    return twin;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge costante precedente dell'half edge
 * @return Il prev dell'half edge
 */
inline const Dcel::HalfEdge* Dcel::HalfEdge::getPrev() const {
    return prev;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge costante successivo dell'half edge
 * @return Il next dell'half edge
 */
inline const Dcel::HalfEdge* Dcel::HalfEdge::getNext() const {
    return next;
}

/**
 * \~Italian
 * @brief Restituisce puntatore alla faccia costante incidente all'half edge
 * @return La faccia incidente all'half edge
 */
inline const Dcel::Face* Dcel::HalfEdge::getFace() const {
    return face;
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra half edge
 * @param otherHalfEdge[in]: half edge con cui verrà verificata l'uguaglianza con l'half edge this
 * @return True se gli half edge sono uguali, false altrimenti
 * @todo Da riscrivere
 */
inline bool Dcel::HalfEdge::operator == (const HalfEdge& otherHalfEdge) const {
    if ( otherHalfEdge.fromVertex == this->fromVertex &&
         otherHalfEdge.toVertex   == this->toVertex	)
    return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra half edge
 * @param[in] otherHalfEdge: half edge con cui verrà verificata la disuguaglianza con l'half edge this
 * @return True se gli half edge sono diversi, false altrimenti
 * @todo Da riscrivere
 */
inline bool Dcel::HalfEdge::operator!=(const HalfEdge& otherHalfEdge) const {
    if ( otherHalfEdge.fromVertex == this->fromVertex &&
         otherHalfEdge.toVertex   == this->toVertex	)
    return false;
    return true;
}

#ifdef DEBUG
/**
 * \~Italian
 * @brief Lancia un'asserzione se il vertice di origine è nullptr
 */
inline void Dcel::HalfEdge::checkFromVertex() const {
    if (fromVertex == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": from_vertex is nullptr";
        assert(! (fromVertex == nullptr));
    }
}

/**
 * \~Italian
 * @brief Lancia un'asserzione se il vertice di destinazione è nullptr
 */
inline void Dcel::HalfEdge::checkToVertex() const {
    if (toVertex == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": to_vertex is nullptr";
        assert(! (toVertex == nullptr));
    }
}

/**
 * \~Italian
 * @brief Lancia un'asserzione se l'half edge gemello è nullptr
 */
inline void Dcel::HalfEdge::checkTwin() const {
    if (twin == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": twin is nullptr";
        assert(! (twin == nullptr));
    }
}

/**
 * \~Italian
 * @brief Lancia un'asserzione se l'half edge precedente è nullptr
 */
inline void Dcel::HalfEdge::checkPrev() const {
    if (prev == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": prev is nullptr";
        assert(! (prev == nullptr));
    }
}

/**
 * \~Italian
 * @brief Lancia un'asserzione se l'half edge successivo è nullptr
 */
inline void Dcel::HalfEdge::checkNext() const {
    if (next == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": next is nullptr";
        assert(! (next == nullptr));
    }
}

/**
 * \~Italian
 * @brief Lancia un'asserzione se la faccia incidente è nullptr
 */
inline void Dcel::HalfEdge::checkFace() const {
    if (face == nullptr){
        std::cerr << "ALERT! Half Edge "<< id << ": face is nullptr";
        assert(! (face == nullptr));
    }
}
#endif

/**
 * \~Italian
 * @brief Setta il flag dell'half edge a 1
 */
inline void Dcel::HalfEdge::setFlag() {
    flag = 1;
}

/**
 * \~Italian
 * @brief Setta il flag dell'half edge
 * @param[in] new_flag: il valore del flag che verrà settato
 */
inline void Dcel::HalfEdge::setFlag(int new_flag) {
    flag = new_flag;
}

/**
 * \~Italian
 * @brief Setta il flag dell'half edge a 0
 */
inline void Dcel::HalfEdge::resetFlag() {
    flag = 0;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore al vertice di origine dell'half edge
 * @return Il from vertex dell'half edge
 */
inline Dcel::Vertex* Dcel::HalfEdge::getFromVertex() {
    return fromVertex;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore al vertice di destinazione dell'half edge
 * @return Il to vertex dell'half edge
 */
inline Dcel::Vertex* Dcel::HalfEdge::getToVertex() {
    return toVertex;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge gemello dell'half edge
 * @return Il twin dell'half edge
 */
inline Dcel::HalfEdge* Dcel::HalfEdge::getTwin() {
    return twin;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge precedente dell'half edge
 * @return Il prev dell'half edge
 */
inline Dcel::HalfEdge* Dcel::HalfEdge::getPrev() {
    return prev;
}

/**
 * \~Italian
 * @brief Restituisce puntatore all'half edge successivo dell'half edge
 * @return Il next dell'half edge
 */
inline Dcel::HalfEdge* Dcel::HalfEdge::getNext() {
    return next;
}

/**
 * \~Italian
 * @brief Restituisce puntatore alla faccia incidente all'half edge
 * @return La faccia incidente all'half edge
 */
inline Dcel::Face* Dcel::HalfEdge::getFace() {
    return face;
}

/**
 * \~Italian
 * @brief Setta il vertice di origine
 * @param[in] newFromVertex: riferimento al from vertex che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setFromVertex(Dcel::Vertex* newFromVertex)	{
    fromVertex = newFromVertex;
}

/**
 * \~Italian
 * @brief Setta il vertice di destinazione
 * @param[in] newToVertex: riferimento al to vertex che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setToVertex(Dcel::Vertex* newToVertex) {
    toVertex = newToVertex;
}

/**
 * \~Italian
 * @brief Setta l'half edge gemello
 * @param[in] newTwin: riferimento al twin che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setTwin(Dcel::HalfEdge* newTwin) {
    twin = newTwin;
}

/**
 * \~Italian
 * @brief Setta l'half edge precedente
 * @param[in] newPrev: riferimento al prev che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setPrev(Dcel::HalfEdge* newPrev) {
    prev = newPrev;
}

/**
 * \~Italian
 * @brief Setta l'half edge successivo
 * @param[in] newNext: riferimento al next che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setNext(Dcel::HalfEdge* newNext) {
    next = newNext;
}

/**
 * \~Italian
 * @brief Setta la faccia incidente
 * @param[in] newFace:riferimento alla faccia che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setFace(Dcel::Face* newFace) {
    face = newFace;
}

/**************************
 * Private Inline Methods *
 **************************/

/**
 * \~Italian
 * @brief Setta l'id dell'half edge.
 *
 * Questa funzione dovrebbe essere chiamata solamente dalla classe Dcel.
 *
 * @param[in] id: nuovo id che verrà assegnato all'half edge
 */
inline void Dcel::HalfEdge::setId(unsigned int id) {
    this->id = id;
}


#endif // DCEL_HALF_EDGE_H
