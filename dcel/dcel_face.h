/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_FACE_H
#define DCEL_FACE_H

#include "dcel_struct.h"
#include <QColor>

/**
 * \~Italian
 * @class Dcel::Face
 * @brief Classe rappresentante una faccia della Dcel.
 *
 * All'interno della Dcel, le sue componenti fondamentali sono:
 * - outerHalfEdge: un half edge incidente esterno, ossia che si trova sul bordo esterno della faccia (tutti gli altri half edge dello stesso tipo saranno individuabili
 *   mediante operazioni di next e prev dell'half edge);\n
 * - innerHalfEdges: lista di half edge incidenti interni, ossia sul bordo di un buco interno alla faccia, un inner half edge per ogni buco della faccia (tutti gli altri
 *   half edge dello stesso tipo saranno individuabili mediante operazioni di next e prev dell'half edge).\n
 *
 *
 * Per una gestione corretta della Dcel, ogni faccia \c f deve avere un \c outerHalfEdge e 0 o più \c innerHalfEdge.
 * Ognuno di questi questi half edge, e tutti quelli raggiungibili mediante operazioni di next e prev, devono avere come incidentFace \c f.
 *
 * Le altre componenti che compongono la faccia sono:
 * - normal: vettore 3D rappresentante la normale alla faccia;\n
 * - area: superficie della faccia;\n
 * - color: colore associato alla faccia;\n
 * - id: intero senza segno univoco all'interno della lista delle facce della Dcel, non modificabile dall'utente. Può essere usato per identificare la faccia all'interno
 *   della Dcel (in modo meno efficiente rispetto all'utilizzo di un puntatore);\n
 * - flag: intero personalizzabile dall'utente. \n
 *
 *
 * Sono messi a disposizione i seguenti iteratori per navigare sugli inner half edge (uno per ogni buco):
 * - Dcel::Face::InnerHalfEdgeIterator
 * - Dcel::Face::ConstInnerHalfEdgeIterator
 *
 *
 * Sono messi a disposizione anche i seguenti iteratori per navigare sulle componenti incidenti alla faccia:
 * - Dcel::Face::IncidentHalfEdgeIterator
 * - Dcel::Face::IncidentVertexIterator
 * - Dcel::Face::ConstIncidentHalfEdgeIterator
 * - Dcel::Face::ConstIncidentVertexIterator
 *
 * Per navigare su tutti gli half edge di un buco adiacenti alla faccia, è possibile usare l'iteratore sugli half edge incidenti:
 *
 * \code{.cpp}
 * Dcel::HalfEdge* firstInnerHalfEdge;
 * if (f->getNumberInnerHalfEdges() > 0){
 *     firstInnerHalfEdge = *(f->innerHalfEdgeBegin()); //primo inner half edge della faccia
 *     for (Dcel::Face::IncidentHalfEdgeIterator heit = f->incidentHalfEdgeBegin(firstInnerHalfEdge); heit!=f->incidentHalfEdgeEnd(); ++heit){
 *         Dcel::HalfEdge* inner = *heit;
 *         // operazioni su inner
 *     }
 * }
 * \endcode
 *
 * Il controllo sull' \c if verifica che la faccia \c f abbia almeno un inner half edge (ossia abbia almeno un buco).
 * A questo punto, \c firstInnerHalfEdge viene fatto puntare ad un inner half edge, e viene fatto ciclare un iteratore sugli incident half edge di \c f
 * inizializzato con \c firstInnerHalfEdge.
 *
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
class Dcel::Face {

        friend class Dcel;

    public:

        /************
        * Iterators *
        *************/

        /**
         * \~Italian
         * @class InnerHalfEdgeIterator
         *
         * @brief Iteratore che permette di ciclare sul vettore degli inner half edge associati alla faccia.
         * Ogni inner half edge è associato ad un buco presente all'interno della faccia.
         * Una volta che si ha a disposizione un inner half edge, è possibile ciclare sul bordo del buco
         * mediante delle semplici operazioni di next, oppure utilizzando un IncidentHalfEdgeIterator o
         * ConstIncidentHalfEdgeIterator opportunamente inizializzato.
         *
         * È possibile utilizzare l'iteratore esattamente come si utilizza un iteratore su un std::vector.
         * Per esempio, data una faccia Dcel::Face* f:
         *
         * \code{.cpp}
         * for (Dcel::Face::InnerHalfEdgeIterator heit = f->innerHalfEgeBegin(); heit != f->innerHalfEdgeEnd(); ++heit){
         *     Dcel::HalfEdge* he = *heit;
         *     // operazioni su he
         * }
         * \endcode
         *
         * Questo iteratore non garantisce l'immutabilità della faccia e quindi della Dcel a cui appartiene, e quindi non è possibile
         * utilizzarlo su const Dcel::Face. Per const Dcel::Face, vedere Dcel::Face::ConstInnerHalfEdgeIterator.
         */
        typedef std::vector<Dcel::HalfEdge*>::iterator InnerHalfEdgeIterator;
        class ConstInnerHalfEdgeIterator;
        class IncidentHalfEdgeIterator;
        class ConstIncidentHalfEdgeIterator;
        class IncidentVertexIterator;
        class ConstIncidentVertexIterator;

        /***************
        * Constructors *
        ****************/

        Face();
        Face(Dcel::HalfEdge* outer);
        Face(Dcel::HalfEdge* outer, const Vec3& normal);
        ~Face();

        /************************
        * Public Inline Methods *
        *************************/

        unsigned int getId()                            const;
        int getFlag()                                   const;
        Vec3 getNormal()                                const;
        double getArea()                                const;
        QColor getColor()                               const;
        const Dcel::HalfEdge* getOuterHalfEdge()        const;
        int getNumberInnerHalfEdges()                   const;
        bool hasHoles()                                 const;
        bool operator == (const Face& otherFace)        const;
        bool operator != (const Face& otherFace)        const;
        #ifdef DEBUG
        void checkOuterHalfEdge()                       const;
        #endif


        void setFlag();
        void setFlag(int newFlag);
        void resetFlag();
        void setNormal(const Vec3& newNormal);
        void setArea(double newArea);
        void setColor(const QColor& newColor);
        Dcel::HalfEdge* getOuterHalfEdge();
        void setOuterHalfEdge(Dcel::HalfEdge* newOuterHalfEdge);
        void addInnerHalfEdge(Dcel::HalfEdge* newInnerHalfEdge);


        /*****************
        * Public Methods *
        ******************/

        const Dcel::Vertex* getVertex1()                                                                            const;
        const Dcel::Vertex* getVertex2()                                                                            const;
        const Dcel::Vertex* getVertex3()                                                                            const;
        bool isTriangle()                                                                                           const;
        int getNumberIncidentVertices()                                                                             const;
        int getNumberIncidentHalfEdges()                                                                            const;
        Pointd getBarycentre()                                                                                      const;
        std::string toString()                                                                                      const;
        ConstInnerHalfEdgeIterator innerHalfEdgeBegin()                                                             const;
        ConstInnerHalfEdgeIterator innerHalfEdgeEnd()                                                               const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin()                                                       const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeEnd()                                                         const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin(const Dcel::HalfEdge* start)                            const;
        ConstIncidentHalfEdgeIterator incidentHalfEdgeBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const;
        ConstIncidentVertexIterator incidentVertexBegin()                                                           const;
        ConstIncidentVertexIterator incidentVertexEnd()                                                             const;
        ConstIncidentVertexIterator incidentVertexBegin(const Dcel::HalfEdge* start)                                const;
        ConstIncidentVertexIterator incidentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end)     const;
        ConstIncidentVertexIterator incidentVertexBegin(const Dcel::Vertex* start)                                  const;
        ConstIncidentVertexIterator incidentVertexBegin(const Dcel::Vertex* start, const Dcel::Vertex* end)         const;

        Dcel::Vertex* getVertex1();
        Dcel::Vertex* getVertex2();
        Dcel::Vertex* getVertex3();
        Vec3 updateNormal();
        float updateArea();
        void removeInnerHalfEdge(const InnerHalfEdgeIterator& iterator);
        bool removeInnerHalfEdge(const Dcel::HalfEdge* halfEdge);
        void removeAllInnerHalfEdges();
        InnerHalfEdgeIterator innerHalfEdgeBegin();
        InnerHalfEdgeIterator innerHalfEdgeEnd();
        IncidentHalfEdgeIterator incidentHalfEdgeBegin();
        IncidentHalfEdgeIterator incidentHalfEdgeEnd();
        IncidentHalfEdgeIterator incidentHalfEdgeBegin(Dcel::HalfEdge* start);
        IncidentHalfEdgeIterator incidentHalfEdgeBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        IncidentVertexIterator incidentVertexBegin();
        IncidentVertexIterator incidentVertexEnd();
        IncidentVertexIterator incidentVertexBegin(Dcel::HalfEdge* start);
        IncidentVertexIterator incidentVertexBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end);
        IncidentVertexIterator incidentVertexBegin(Dcel::Vertex* start);
        IncidentVertexIterator incidentVertexBegin(Dcel::Vertex* start, Dcel::Vertex* end);

        #ifdef CGAL_DEFINED
        void getTriangulation(std::vector<std::tuple<const Vertex*, const Vertex*, const Vertex*> >& triangles) const;
        #endif

    protected:

        /*************
        * Attributes *
        **************/

        Dcel::HalfEdge*                 outerHalfEdge;  /**< \~Italian @brief Uno degli half edge sul bordo della faccia */
        std::vector<Dcel::HalfEdge*>    innerHalfEdges; /**< \~Italian @brief Lista degli half edge sul bordo di eventuali buchi della faccia, uno per ogni buco */
        Vec3                            normal;         /**< \~Italian @brief Vettore normale alla faccia */
        double                          area;           /**< \~Italian @brief Superficie della faccia */
        QColor                          color;          /**< \~Italian @brief Colore associato alla faccia */
        unsigned int                    id;             /**< \~Italian @brief Id univoco, all'interno della Dcel, associato alla faccia */
        int                             flag;           /**< \~Italian @brief Flag personalizzabile, associato alla faccia */

        /***************************
        * Protected Inline Methods *
        ****************************/

        void setId(unsigned int id);

        /********************
        * Protected Methods *
        *********************/

        std::string innerComponentsToString() const;
};

/*************************
 * Public Inline Methods *
 *************************/
/**
 * \~Italian
 * @brief Restituisce l'id identificativo nella Dcel della faccia
 * @return L'id della faccia
 */
inline unsigned int Dcel::Face::getId() const {
    return id;
}

/**
 * \~Italian
 * @brief Restituisce il flag associato alla faccia
 * @return Il flag della faccia
 */
inline int Dcel::Face::getFlag() const {
    return flag;
}

/**
 * \~Italian
 * @brief Restituisce il vettore normale alla faccia
 * @note Non ricalcola la normale, restituisce solo l'ultima normale calcolata o settata
 * @return La normale della faccia
 */
inline Vec3 Dcel::Face::getNormal() const {
    return normal;
}

/**
 * \~Italian
 * @brief Restituisce la superficie della faccia
 * @note Non ricalcola l'area, restituisce solo l'ultima area calcolata o settata
 * @return L'area della faccia
 */
inline double Dcel::Face::getArea() const {
    return area;
}

/**
 * \~Italian
 * @brief Restituisce il colore associato alla faccia
 * @return Il colore della faccia
 */
inline QColor Dcel::Face::getColor() const {
    return color;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore all'half edge costante di bordo esterno della faccia
 * @return L'HalfEdge di bordo della faccia
 */
inline const Dcel::HalfEdge* Dcel::Face::getOuterHalfEdge() const {
    return outerHalfEdge;
}

/**
 * \~Italian
 * @brief Restituisce il numero di inner half edges contenuti nella faccia, ossia il numero di buchi
 * @return Il numero di HalfEdge interni della faccia
 */
inline int Dcel::Face::getNumberInnerHalfEdges() const {
    return innerHalfEdges.size();
}

/**
 * \~Italian
 * @brief Restituisce true se la faccia contiene buchi
 * @return True se la faccia contiene buchi, false altrimenti
 */
inline bool Dcel::Face::hasHoles() const {
    return (innerHalfEdges.size() != 0);
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra facce
 * @param[in] otherFace: faccia con cui verrà verificata l'uguaglianza con la faccia this
 * @return True se le facce sono uguali, false altrimenti
 * @todo Da riscrivere
 */
inline bool Dcel::Face::operator == (const Face& otherFace) const {
    if (otherFace.id == id) return true;
    else return false;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra facce
 * @param[in] otherFace: faccia con cui verrà verificata la disuguaglianza con la faccia this
 * @return True se le facce sono diverse, false altrimenti
 */
inline bool Dcel::Face::operator != (const Face& otherFace) const {
    return !(*this == otherFace);
}

#ifdef DEBUG
/**
 * \~Italian
 * @brief Lancia un'asserzione se l'outerHalfEdge è nullptr
 */
inline void Dcel::Face::checkOuterHalfEdge() const {
    if (outerHalfEdge == nullptr) {
        std::cerr << "ALERT! Face "<< id << ": outer_half_edge is nullptr";
        assert(! (outerHalfEdge == nullptr));
    }
}
#endif

/**
 * \~Italian
 * @brief Setta il flag della faccia a 1
 */
inline void Dcel::Face::setFlag() {
    flag = 1;
}

/**
 * \~Italian
 * @brief Setta il flag della faccia
 * @param[in] newFlag: il valore del flag che verrà settato
 */
inline void Dcel::Face::setFlag(int newFlag) {
    flag = newFlag;
}

/**
 * \~Italian
 * @brief Setta il flag della faccia a 0
 */
inline void Dcel::Face::resetFlag() {
    flag = 0;
}

/**
 * \~Italian
 * @brief Setta il vettore normale della faccia
 * @param[in] newNormal: il vettore normale che verrà settato
 */
inline void Dcel::Face::setNormal(const Vec3& newNormal) {
    normal = newNormal;
}

/**
 * \~Italian
 * @brief Setta la superficie della faccia
 * @param[in] newArea: il valore dell'area che verrà settato
 */
inline void Dcel::Face::setArea(double newArea) {
    area = newArea;
}

/**
 * \~Italian
 * @brief Assegna un nuovo colore alla faccia
 * @param[in] newColor: il nuovo colore che verrà assegnato alla faccia
 */
inline void Dcel::Face::setColor(const QColor& newColor) {
    color = newColor;
}

/**
 * \~Italian
 * @brief Restituisce il puntatore all'half edge di bordo esterno della faccia
 * @return L'HalfEdge di bordo della faccia
 */
inline Dcel::HalfEdge* Dcel::Face::getOuterHalfEdge() {
    return outerHalfEdge;
}

/**
 * \~Italian
 * @brief Assegna un nuovo half edge di bordo esterno alla faccia
 * @param[in] newOuterHalfEdge: puntatore all'half edge di bordo esterno assegnato alla faccia
 */
inline void Dcel::Face::setOuterHalfEdge(Dcel::HalfEdge* newOuterHalfEdge) {
    outerHalfEdge = newOuterHalfEdge;
}

/**
 * \~Italian
 * @brief Aggiunge un nuovo half edge di bordo interno (ossia un buco) alla faccia
 * @param[in] newInnerHalfEdge: nuovo half edge di bordo interno aggiunto alla faccia
 */
inline void Dcel::Face::addInnerHalfEdge(Dcel::HalfEdge* newInnerHalfEdge) {
    innerHalfEdges.push_back(newInnerHalfEdge);
}

/**************************
 * Private Inline Methods *
 **************************/

/**
 * \~Italian
 * @brief Setta l'id della faccia.
 *
 * Questa funzione dovrebbe essere chiamata solamente dalla classe Dcel.
 *
 * @param[in] id: nuovo id che verrà assegnato alla faccia
 */
inline void Dcel::Face::setId(unsigned int id) {
    this->id = id;
}

#endif // DCEL_FACE_H
