/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_VERTEX_H
#define DCEL_VERTEX_H

#include "dcel_struct.h"

/**
 * \~Italian
 * @class Dcel::Vertex
 * @brief Classe rappresentante un vertice della DCEL.
 *
 * All'interno della Dcel, le sue componenti fondamentali sono:
 * - coordinate: Pointd contenente la posizione in uno spazio 3D del vertice; \n
 * - incicentHalfEdge: uno degli half edge \b uscenti incidenti al vertice, ossia
 *   half edge che ha come from vertex il vertice stesso. \n
 *
 *
 * Per una corretta gestione della Dcel, ogni Dcel::Vertex \c v deve avere un \c incidentHalfEdge.
 * Tale halfEdge deve avere come fromVertex il vertice \c v, e il suo twin deve avere \c v come toVertex.
 * Devono poi essere rispettate le regole degli halfEdge e dei loro vertici incidenti sulle relazioni di
 * prev e next.
 *
 * Le altre componenti che compongono il vertice sono:
 * - normal: vettore 3D rappresentante la normale del vertice, che solitamente è la
 *   media delle normali delle facce incidenti;\n
 * - cardinality: numero di edge (ossia la metà del numero di half edge) incidenti sul vertice;\n
 * - id: intero senza segno univoco all'interno della lista dei vertici della Dcel,
 *   non modificabile dall'utente. Può essere usato per identificare il vertice all'interno
 *   della Dcel (in modo meno efficiente rispetto all'utilizzo di un puntatore);\n
 * - flag: intero personalizzabile dall'utente. \n
 *
 *
 * @todo color: colore associato al vertice;\n
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Vertex
{
        friend class Dcel;

    public:

        /************
        * Iterators *
        *************/

        class AdjacentVertexIterator;
        class ConstAdjacentVertexIterator;
        class OutgoingHalfEdgeIterator;
        class ConstOutgoingHalfEdgeIterator;
        class IncomingHalfEdgeIterator;
        class ConstIncomingHalfEdgeIterator;
        class IncidentHalfEdgeIterator;
        class ConstIncidentHalfEdgeIterator;
        class IncidentFaceIterator;
        class ConstIncidentFaceIterator;

        /***************
        * Constructors *
        ****************/

        Vertex();
        Vertex(const Pointd& p);
        Vertex(const Pointd& p, Dcel::HalfEdge* halfEdge);
        Vertex(const Pointd& p, Dcel::HalfEdge* halfEdge, int cardinality);
        ~Vertex();

        /************************
        * Public Inline Methods *
        *************************/

        unsigned int getId()                                const;
        int getFlag()                                       const;
        Vec3 getNormal()                                    const;
        Pointd getCoordinate()                              const;
        int getCardinality()                                const;
        const Dcel::HalfEdge* getIncidentHalfEdge()         const;
        double dist(const Dcel::Vertex* otherVertex)        const;
        bool operator == (const Dcel::Vertex& otherVertex)  const;
        bool operator != (const Dcel::Vertex& othervertex)  const;
        #ifdef DEBUG
        void checkIncidentHalfEdge()                        const;
        #endif

        void setFlag();
        void setFlag(int newFlag);
        void resetFlag();
        void setNormal(const Vec3& newNormal);
        void setCoordinate(const Pointd& newCoordinate);
        void setCardinality(int newCardinality);
        int decrementCardinality();
        int incrementCardinality();
        Dcel::HalfEdge* getIncidentHalfEdge();
        void setIncidentHalfEdge(Dcel::HalfEdge* newIncidentHalfEdge);

        /*****************
        * Public Methods *
        ******************/

        int getNumberIncidentHalfEdges()                                                                            const;
        int getNumberIncidentFaces()                                                                                const;
        int getNumberAdjacentVertices()                                                                             const;
        const Dcel::HalfEdge* findSharedHalfEdge(const Dcel::Vertex* vertex)                                        const;
        std::string toString()                                                                                      const;
        ConstAdjacentVertexIterator adjacentVertexBegin()                                                           const;
        ConstAdjacentVertexIterator adjacentVertexEnd()                                                             const;
        ConstAdjacentVertexIterator adjacentVertexBegin(const Dcel::HalfEdge* start)                                const;
        ConstAdjacentVertexIterator adjacentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end)     const;
        ConstAdjacentVertexIterator adjacentVertexBegin(const Dcel::Vertex* start)                                  const;
        ConstAdjacentVertexIterator adjacentVertexBegin(const Dcel::Vertex* start, const Dcel::Vertex* end)         const;
        ConstOutgoingHalfEdgeIterator outgoingHalfEdgeBegin()                                                       const;
        ConstOutgoingHalfEdgeIterator outgoingHalfEdgeEnd()                                                         const;
        ConstOutgoingHalfEdgeIterator outgoingHalfEdgeBegin(const Dcel::HalfEdge* start)                            const;
        ConstOutgoingHalfEdgeIterator outgoingHalfEdgeBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const;
        ConstIncomingHalfEdgeIterator incomingHalfEdgeBegin()                                                       const;
        ConstIncomingHalfEdgeIterator incomingHalfEdgeEnd()                                                         const;
        ConstIncomingHalfEdgeIterator incomingHalfEdgeBegin(const Dcel::HalfEdge* start)                            const;
        ConstIncomingHalfEdgeIterator incomingHalfEdgeBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin()                                                       const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeEnd()                                                         const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin(const Dcel::HalfEdge* start)                            const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const;
        ConstIncidentFaceIterator incidentFaceBegin()                                                               const;
        ConstIncidentFaceIterator incidentFaceEnd()                                                                 const;
        ConstIncidentFaceIterator incidentFaceBegin(const Dcel::HalfEdge* start)                                    const;
        ConstIncidentFaceIterator incidentFaceBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end)         const;

        Vec3 updateNormal();
        unsigned int updateCardinality();
        Dcel::HalfEdge* findSharedHalfEdge(const Dcel::Vertex* vertex);
        AdjacentVertexIterator adjacentVertexBegin();
        AdjacentVertexIterator adjacentVertexEnd();
        AdjacentVertexIterator adjacentVertexBegin(Dcel::HalfEdge* start);
        AdjacentVertexIterator adjacentVertexBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        AdjacentVertexIterator adjacentVertexBegin(Dcel::Vertex* start);
        AdjacentVertexIterator adjacentVertexBegin(Dcel::Vertex* start, Dcel::Vertex* end);
        OutgoingHalfEdgeIterator outgoingHalfEdgeBegin();
        OutgoingHalfEdgeIterator outgoingHalfEdgeEnd();
        OutgoingHalfEdgeIterator outgoingHalfEdgeBegin(Dcel::HalfEdge* start);
        OutgoingHalfEdgeIterator outgoingHalfEdgeBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        IncomingHalfEdgeIterator incomingHalfEdgeBegin();
        IncomingHalfEdgeIterator incomingHalfEdgeEnd();
        IncomingHalfEdgeIterator incomingHalfEdgeBegin(Dcel::HalfEdge* start);
        IncomingHalfEdgeIterator incomingHalfEdgeBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        IncidentHalfEdgeIterator incidentHalfEdgeBegin();
        IncidentHalfEdgeIterator incidentHalfEdgeEnd();
        IncidentHalfEdgeIterator incidentHalfEdgeBegin(Dcel::HalfEdge* start);
        IncidentHalfEdgeIterator incidentHalfEdgeBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        IncidentFaceIterator incidentFaceBegin();
        IncidentFaceIterator incidentFaceEnd();
        IncidentFaceIterator incidentFaceBegin(Dcel::HalfEdge* start);
        IncidentFaceIterator incidentFaceBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);

        class ConstAdjacentVertexRangeBasedIterator {
                friend class Vertex;
            public:
                ConstAdjacentVertexIterator begin() const;
                ConstAdjacentVertexIterator end() const;
            private:
                ConstAdjacentVertexRangeBasedIterator(const Vertex *v) : v(v) {}
                const Vertex *v;
        };

        class ConstOutgoingHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                ConstOutgoingHalfEdgeIterator begin() const;
                ConstOutgoingHalfEdgeIterator end() const;
            private:
                ConstOutgoingHalfEdgeRangeBasedIterator(const Vertex *v) : v(v) {}
                const Vertex *v;
        };

        class ConstIncomingHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                ConstIncomingHalfEdgeIterator begin() const;
                ConstIncomingHalfEdgeIterator end() const;
            private:
                ConstIncomingHalfEdgeRangeBasedIterator(const Vertex *v) : v(v) {}
                const Vertex *v;
        };

        class ConstIncidentHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                ConstIncidentHalfEdgeIterator begin() const;
                ConstIncidentHalfEdgeIterator end() const;
            private:
                ConstIncidentHalfEdgeRangeBasedIterator(const Vertex *v) : v(v) {}
                const Vertex *v;
        };

        class ConstIncidentFaceRangeBasedIterator {
                friend class Vertex;
            public:
                ConstIncidentFaceIterator begin() const;
                ConstIncidentFaceIterator end() const;
            private:
                ConstIncidentFaceRangeBasedIterator(const Vertex *v) : v(v) {}
                const Vertex *v;
        };

        class AdjacentVertexRangeBasedIterator {
                friend class Vertex;
            public:
                AdjacentVertexIterator begin();
                AdjacentVertexIterator end();
            private:
                AdjacentVertexRangeBasedIterator(Vertex *v) : v(v) {}
                Vertex *v;
        };

        class OutgoingHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                OutgoingHalfEdgeIterator begin();
                OutgoingHalfEdgeIterator end();
            private:
                OutgoingHalfEdgeRangeBasedIterator(Vertex *v) : v(v) {}
                Vertex *v;
        };

        class IncomingHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                IncomingHalfEdgeIterator begin();
                IncomingHalfEdgeIterator end();
            private:
                IncomingHalfEdgeRangeBasedIterator(Vertex *v) : v(v) {}
                Vertex *v;
        };

        class IncidentHalfEdgeRangeBasedIterator {
                friend class Vertex;
            public:
                IncidentHalfEdgeIterator begin();
                IncidentHalfEdgeIterator end();
            private:
                IncidentHalfEdgeRangeBasedIterator(Vertex *v) : v(v) {}
                Vertex *v;
        };

        class IncidentFaceRangeBasedIterator {
                friend class Vertex;
            public:
                IncidentFaceIterator begin();
                IncidentFaceIterator end();
            private:
                IncidentFaceRangeBasedIterator(Vertex *v) : v(v) {}
                Vertex *v;
        };

        const ConstAdjacentVertexRangeBasedIterator adjacentVertexIterator() const;
        AdjacentVertexRangeBasedIterator adjacentVertexIterator();
        const ConstOutgoingHalfEdgeRangeBasedIterator outgoingHalfEdgeIterator() const;
        OutgoingHalfEdgeRangeBasedIterator outgoingHalfEdgeIterator();
        const ConstIncomingHalfEdgeRangeBasedIterator incomingHalfEdgeIterator() const;
        IncomingHalfEdgeRangeBasedIterator incomingHalfEdgeIterator();
        const ConstIncidentHalfEdgeRangeBasedIterator incidentHalfEdgeIterator() const;
        IncidentHalfEdgeRangeBasedIterator incidentHalfEdgeIterator();
        const ConstIncidentFaceRangeBasedIterator incidentFaceIterator() const;
        IncidentFaceRangeBasedIterator incidentFaceIterator();

    protected:

        class GenericIterator;
        class ConstGenericIterator;

        /*************
        * Attributes *
        **************/

        Pointd          coordinate;         /**< \~Italian @brief Punto nello spazio 3D rappresentante la posizione del vertice */
        Dcel::HalfEdge* incidentHalfEdge;   /**< \~Italian @brief Uno degli half edge uscenti incidenti sul vertice */
        Vec3            normal;             /**< \~Italian @brief Vettore normale al vertice */
        unsigned int    cardinality;        /**< \~Italian @brief Numero di edge (metà degli half edge) incidenti sul vertice */
        unsigned int    id;                 /**< \~Italian @brief Id univoco, all'interno della Dcel, associato al vertice */
        int             flag;               /**< \~Italian @brief Flag personalizzabile, associato al vertice */

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
 * @brief Restirìtuisce l'id identificativo nella Dcel del vertice
 * @return L'id del vertice
 */
inline unsigned int Dcel::Vertex::getId() const {
    return id;
}

/**
 * \~Italian
 * @brief Restituisce il flag associato al vertice
 * @return Il flag del vertice
 */
inline int Dcel::Vertex::getFlag() const {
    return flag;
}

/**
 * \~Italian
 * @brief Restituisce il vettore normale al vertice
 * @note Non ricalcola la normale, restituisce solo l'ultima normale calcolata o settata
 * @return La normale al vertice
 */
inline Vec3 Dcel::Vertex::getNormal() const	{
    return normal;
}

/**
 * \~Italian
 * @brief Restituisce le coordinate del vertice
 * @return Pointd rappresentante la posizione nello spazio del vertice
 */
inline Pointd Dcel::Vertex::getCoordinate() const {
    return coordinate;
}

/**
 * \~Italian
 * @brief Restituisce il numero di edge incidenti sul vertice
 * @note Non ricalcola la cardinalità, restituisce solo l'ultima cardinalità calcolata o settata
 * @return La cardinalità del vertice
 */
inline int  Dcel::Vertex::getCardinality() const {
    return cardinality;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore l'half edge costante incidente sul vertice
 * @return L'HalfEdge incidente sul vertice
 */
inline const Dcel::HalfEdge* Dcel::Vertex::getIncidentHalfEdge() const {
    return incidentHalfEdge;
}

/**
 * \~Italian
 * @brief Calcola e restituisce la distanza tra il vertice this e il vertice in input
 * @param[in] otherVertex: vertice con cui verrà calcolata la distanza dal vertice this
 * @return La distanza tra il vertice this e otherVertex
 */
inline double Dcel::Vertex::dist(const Vertex* otherVertex) const {
    return this->coordinate.dist(otherVertex->coordinate);
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra vertici
 * @param[in] otherVertex: vertice con cui verrà verificata l'uguaglianza con la faccia this
 * @return True se i vertici sono uguali, false altrimenti
 * @todo Da riscrivere
 */
inline bool Dcel::Vertex::operator == (const Vertex& otherVertex) const {
    if (otherVertex.coordinate == this->coordinate ) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra vertici
 * @param[in] otherVertex: vertice con cui verrà verificata la disuguaglianza con la faccia this
 * @return True se i vertici sono diversi, false altrimenti
 */
inline bool Dcel::Vertex::operator != (const Vertex& otherVertex) const {
    if (otherVertex.coordinate == this->coordinate ) return false;
    return true;
}

#ifdef DEBUG
/**
 * \~Italian
 * @brief Lancia un'asserzione se l'incidentHalfEdge è nullptr
 */
inline void Dcel::Vertex::checkIncidentHalfEdge() const {
    if (incidentHalfEdge == nullptr){
        std::cerr << "ALERT! Vertex "<< id <<": incident_half_edge is nullptr";
        assert(! (incidentHalfEdge == nullptr));
    }
}
#endif

/**
 * \~Italian
 * @brief Setta il flag del vertice a 1
 */
inline void Dcel::Vertex::setFlag() {
    flag = 1;
}

/**
 * \~Italian
 *
 * @brief Setta il flag del vertice
 * @param[in] newFlag: il valore del flag che verrà settato
 */
inline void Dcel::Vertex::setFlag(int newFlag) {
    flag = newFlag;
}

/**
 * \~Italian
 * @brief Setta il flag della faccia a 0
 */
inline void Dcel::Vertex::resetFlag() {
    flag = 0;
}

/**
 * \~Italian
 * @brief Setta il vettore normale al vertice
 * @param[in] newNormal: il vettore normale che verrà settato
 */
inline void Dcel::Vertex::setNormal(const Vec3& newNormal) {
    normal = newNormal;
}

/**
 * \~Italian
 * @brief Setta le coordinate del vertice
 * @param[in] newCoordinate: il punto che verrà settato
 */
inline void Dcel::Vertex::setCoordinate(const Pointd& newCoordinate) {
    coordinate = newCoordinate;
}

/**
 * \~Italian
 * @brief Setta la cardinalità del vertice
 * @param[in] newCardinality: la cardinalitù che verrà settata
 */
inline void Dcel::Vertex::setCardinality( int newCardinality ) {
    cardinality = newCardinality;
}

/**
 * \~Italian
 * @brief Decrementa la cardinalità del vertice (se essa è maggiore di 0)
 * @return La cardinalità decrementata
 */
inline int Dcel::Vertex::decrementCardinality() {
    if (cardinality > 0) --cardinality;
    return cardinality;
}

/**
 * \~Italian
 * @brief Incrementa la cardinalità del vertice
 * @return La cardinalità incrementata
 */
inline int Dcel::Vertex::incrementCardinality() {
    return ++cardinality;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore all'half edge incidente sul vertice
 * @return L'HalfEdge incidente sul vertice
 */
inline Dcel::HalfEdge* Dcel::Vertex::getIncidentHalfEdge() {
    return incidentHalfEdge;
}

/**
 * \~Italian
 * @brief Assegna un nuovo half edge incidente al vertice
 * @param[in] newIncidentHalfEdge: puntatore all'half edge incidente assegnato al vertice
 */
inline void Dcel::Vertex::setIncidentHalfEdge(HalfEdge* newIncidentHalfEdge ) {
    incidentHalfEdge = newIncidentHalfEdge;
}

inline const Dcel::Vertex::ConstAdjacentVertexRangeBasedIterator Dcel::Vertex::adjacentVertexIterator() const {
    return std::move(ConstAdjacentVertexRangeBasedIterator(this));
}

inline Dcel::Vertex::AdjacentVertexRangeBasedIterator Dcel::Vertex::adjacentVertexIterator() {
    return std::move(AdjacentVertexRangeBasedIterator(this));
}

inline const Dcel::Vertex::ConstOutgoingHalfEdgeRangeBasedIterator Dcel::Vertex::outgoingHalfEdgeIterator() const {
    return std::move(ConstOutgoingHalfEdgeRangeBasedIterator(this));
}

inline Dcel::Vertex::OutgoingHalfEdgeRangeBasedIterator Dcel::Vertex::outgoingHalfEdgeIterator() {
    return std::move(OutgoingHalfEdgeRangeBasedIterator(this));
}

inline const Dcel::Vertex::ConstIncomingHalfEdgeRangeBasedIterator Dcel::Vertex::incomingHalfEdgeIterator() const {
    return std::move(ConstIncomingHalfEdgeRangeBasedIterator(this));
}

inline Dcel::Vertex::IncomingHalfEdgeRangeBasedIterator Dcel::Vertex::incomingHalfEdgeIterator() {
    return std::move(IncomingHalfEdgeRangeBasedIterator(this));
}

inline const Dcel::Vertex::ConstIncidentHalfEdgeRangeBasedIterator Dcel::Vertex::incidentHalfEdgeIterator() const {
    return std::move(ConstIncidentHalfEdgeRangeBasedIterator(this));
}

inline Dcel::Vertex::IncidentHalfEdgeRangeBasedIterator Dcel::Vertex::incidentHalfEdgeIterator() {
    return std::move(IncidentHalfEdgeRangeBasedIterator(this));
}

inline const Dcel::Vertex::ConstIncidentFaceRangeBasedIterator Dcel::Vertex::incidentFaceIterator() const {
    return std::move(ConstIncidentFaceRangeBasedIterator(this));
}

inline Dcel::Vertex::IncidentFaceRangeBasedIterator Dcel::Vertex::incidentFaceIterator() {
    return std::move(IncidentFaceRangeBasedIterator(this));
}

/****************************
 * Protected Inline Methods *
 ****************************/

/**
 * \~Italian
 * @brief Setta l'id del vertice.
 *
 * Questa funzione dovrebbe essere chiamata solamente dalla classe Dcel.
 *
 * @param[in] id: nuovo id che verrà assegnato al vertice
 */
inline void Dcel::Vertex::setId(unsigned int id)
{
    this->id = id;
}

#endif // DCEL_VERTEX_H
