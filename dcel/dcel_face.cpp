/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "dcel_face_iterators.h"
#include "dcel_vertex_iterators.h"
#include "../common/common.h"

/****************
 * Constructors *
 ****************/
/**
 * \~Italian
 * @brief Costruttore di default.
 *
 * Crea una faccia con:
 * - outer half edge settato a nullptr;
 * - nessun inner half edge;
 * - normale pari a (0, 0, 0);
 * - area pari a 0;
 * - id pari a 0;
 * - flag pari a 0.
 */
Dcel::Face::Face() : outerHalfEdge(nullptr), area(0), id(0), flag(0){
    innerHalfEdges.clear();
}

/**
 * \~Italian
 * @brief Costruttore.
 *
 * Crea una faccia con:
 * - outer half edge pari al parametro in input outer;
 * - nessun inner half edge;
 * - normale pari a (0, 0, 0);
 * - area pari a 0;
 * - id pari a 0;
 * - flag pari a 0.
 *
 *
 * @param[in] outer: puntatore all'outer half edge settato alla faccia
 */
Dcel::Face::Face(Dcel::HalfEdge* outer) : outerHalfEdge(outer), area(0), id(0), flag(0) {
    innerHalfEdges.clear();
}

/**
 * \~Italian
 * @brief Costruttore.
 *
 * Crea una faccia con:
 * - outer half edge pari al parametro in input outer;
 * - nessun inner half edge;
 * - normale pari al parametro in input normal;
 * - area pari a 0;
 * - id pari a 0;
 * - flag pari a 0;
 *
 *
 * @param[in] outer: puntatore all'outer half edge settato alla faccia
 * @param[in] normal: vettore normale assegnato alla faccia
 */
Dcel::Face::Face(Dcel::HalfEdge* outer, const Vec3& normal) : outerHalfEdge(outer), normal(normal), area(0), id(0), flag(0) {
    innerHalfEdges.clear();
}

/**
 * \~Italian
 * @brief Distruttore vuoto.
 *
 * La classe Dcel dovrà occuparsi di eliminare tutti i riferimenti in essa contenuti (e quindi contenuti di conseguenza anche nella classe Dcel::Face).
 */
Dcel::Face::~Face(void) {}


/******************
 * Public Methods *
 ******************/

const Dcel::Vertex*Dcel::Face::getVertex1() const {
    return outerHalfEdge->getFromVertex();
}

const Dcel::Vertex*Dcel::Face::getVertex2() const {
    return outerHalfEdge->getToVertex();
}

const Dcel::Vertex*Dcel::Face::getVertex3() const {
    return outerHalfEdge->getNext()->getToVertex();
}

/**
 * \~Italian
 * @brief Funzione che verifica se la faccia è un triangolo
 * @warning utilizza Dcel::Face::ConstIncidentVertexIterator
 * @return True se la faccia è un triangolo, false altrimenti
 */
bool Dcel::Face::isTriangle() const {
    Dcel::Face::ConstIncidentVertexIterator v = incidentVertexBegin();
    ++v; ++v; ++v;
    return (v == incidentVertexEnd());
}

/**
 * \~Italian
 * @brief Restituisce il numero di vertici incidenti alla faccia
 * @warning Utilizza Dcel::Face::ConstIncidentVertexIterator
 * @return Il numero di vertici incidenti alla faccia
 */
int Dcel::Face::getNumberIncidentVertices() const {
    ConstIncidentVertexIterator vi;
    int n = 0;
    for (vi = incidentVertexBegin(); vi != incidentVertexEnd(); ++vi) n++;
    return n;
}

/**
 * \~Italian
 * @brief Restituisce il numero di half edge incidenti alla faccia
 * @warning Utilizza Dcel::Face::ConstIncidentHalfEdgeIterator
 * @return Il numero di half edge incidenti alla faccia
 */
int Dcel::Face::getNumberIncidentHalfEdges() const {
    ConstIncidentHalfEdgeIterator hei;
    int n = 0;
    for (hei = incidentHalfEdgeBegin(); hei != incidentHalfEdgeEnd(); ++hei) n++;
    return n;
}

/**
 * \~Italian
 * @brief Calcola e restituisce il baricentro della faccia.
 *
 * Il baricentro è calcolato come media dei vertici incidenti alla faccia.
 *
 * @warning Utilizza Dcel::Face::ConstIncidentVertexIterator
 * @return Il baricentro della faccia.
 */
Pointd Dcel::Face::getBarycentre() const {
    int n = 0;
    Pointd p;
    for (ConstIncidentVertexIterator vit = incidentVertexBegin(); vit != incidentVertexEnd(); ++vit){
        p += (*vit)->getCoordinate();
        n++;
    }
    p /= n;
    return p;
}

/**
 * \~Italian
 * @brief Funzione toString di una faccia
 * @return Una stringa rappresentativa della faccia
 * @todo Da aggiornare
 */
std::string Dcel::Face::toString() const {
    std::stringstream ss;

    ss << "ID: " << id << "; Normal: " << normal.toString() << "; Outer Component: ";
    if (outerHalfEdge != nullptr) ss << outerHalfEdge->getId();
    else ss << "nullptr";
    ss << "; N Inner Components: " << innerHalfEdges.size() << "; Inner Components: "
       << innerComponentsToString() << ".";
    std::string s1 = ss.str();
    return s1;
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstInnerHalfEdgeIterator
 * @return Un iteratore che punta al primo inner half edge della faccia
 */
Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::innerHalfEdgeBegin() const {
    return innerHalfEdges.begin();
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::ConstInnerHalfEdgeIterator
 * @return Un iteratore che punta all'ultimo inner half edge della faccia
 */
Dcel::Face::ConstInnerHalfEdgeIterator Dcel::Face::innerHalfEdgeEnd() const {
    return innerHalfEdges.end();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'outer half edge.
 *
 * @return Un iteratore che punta all'outer half edge della faccia
 */
Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin() const {
    return ConstIncidentHalfEdgeIterator(outerHalfEdge, outerHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::ConstIncidentHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeEnd() const {
    return ConstIncidentHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta essere incidente alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin(const Dcel::HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return ConstIncidentHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Face::ConstIncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    if (end->getFace() != this){
        std::cerr << "ERROR: end half edge " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return ConstIncidentHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'outer half edge.
 *
 * @return Un iteratore che punta al toVertex dell'outer half edge della faccia
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexBegin() const {
    return ConstIncidentVertexIterator(outerHalfEdge, outerHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::ConstIncidentVertexIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexEnd() const {
    return ConstIncidentVertexIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta essere incidente alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexBegin(const Dcel::HalfEdge* start) const {
    #ifdef DEBUG
    if (start->getFace() != this) {
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return ConstIncidentVertexIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'half edge start e fino al toVertex dell'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, il cui toVertex \b non \b è \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end) const {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    if (end->getFace() != this){
        std::cerr << "ERROR: end half edge " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return ConstIncidentVertexIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal vertice start. \n
 * È meno efficiente rispetto a Dcel::Face::constIncidentVertexBegin(const Dcel::HalfEdge* start).
 *
 * @param[in] start: vertice di partenza
 * @warning Se start non risulta essere incidente alla faccia (ossia non possiede un half edge incidente alla faccia),
 *          viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexBegin(const Dcel::Vertex* start) const {
    for (Dcel::Vertex::ConstIncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
         heit!= start->incomingHalfEdgeEnd();
         ++heit)
    {
        if ((*heit)->getFace() == this) return ConstIncidentVertexIterator(*heit, *heit, this);
    }
    std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
    assert(0);
    return ConstIncidentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::ConstIncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal vertice start e fino al vertice end. \n
 * È meno efficiente rispetto a Dcel::Face::constIncidentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end).
 *
 * @param[in] start: vertice di partenza
 * @param[in] end: vertice di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia (ossia non possiedono un half edge incidente alla faccia),
 *          viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 */
Dcel::Face::ConstIncidentVertexIterator Dcel::Face::incidentVertexBegin(const Dcel::Vertex* start, const Dcel::Vertex* end) const {
    Vertex::ConstIncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
    while (heit!= start->incomingHalfEdgeEnd() && ((*heit)->getFace() != this)) ++heit;
    #ifdef DEBUG
    if ((*heit)->getFace() != this) {
        std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    const HalfEdge* s = *heit;
    for (heit= end->incomingHalfEdgeBegin(); heit!= end->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFace() == this) return ConstIncidentVertexIterator(s, *heit, this);
    }
    std::cerr << "ERROR: end vertex " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
    assert(0);
    return ConstIncidentVertexIterator();
}

Dcel::Vertex*Dcel::Face::getVertex1() {
    return outerHalfEdge->getFromVertex();
}

Dcel::Vertex*Dcel::Face::getVertex2() {
    return outerHalfEdge->getToVertex();
}

Dcel::Vertex*Dcel::Face::getVertex3() {
    return outerHalfEdge->getNext()->getToVertex();
}

/**
 * \~Italian
 * @brief Funzione che aggiorna la normale alla faccia
 * @warning Funziona se e solo se la faccia è un triangolo
 * @warning Utilizza Dcel::Face::ConstIncidentVertexIterator
 * @return La normale alla faccia aggiornata
 */
Vec3 Dcel::Face::updateNormal() {
    Vertex* a = outerHalfEdge->getFromVertex();
    Vertex* b = outerHalfEdge->getToVertex();
    Vertex* c = outerHalfEdge->getNext()->getToVertex();
    normal = (b->getCoordinate() - a->getCoordinate()).cross(c->getCoordinate() - a->getCoordinate());
    normal.normalize();
    if (outerHalfEdge->getNext()->getNext()->getToVertex() != a){
        Vec3 zAxis(0,0,1);
        Vec3 v = -(normal.cross(zAxis));
        v.normalize();
        double dot = normal.dot(zAxis);
        double angle = acos(dot);

        double r[3][3] = {0};
        if (normal != zAxis){
            if (normal == -zAxis){
                v = Vec3(1,0,0);
            }
            getRotationMatrix(v, angle, r);
        }
        else {
            r[0][0] = r[1][1] = r[2][2] = 1;
        }

        std::vector<Pointd> points;
        for (ConstIncidentVertexIterator vit = incidentVertexBegin(); vit != incidentVertexEnd(); ++vit){
            Pointd p = (*vit)->getCoordinate();
            Pointd pr(p.x() * r[0][0] + p.y() * r[1][0] +p.z() * r[2][0], p.x() * r[0][1] + p.y() * r[1][1] +p.z() * r[2][1], p.x() * r[0][2] + p.y() * r[1][2] +p.z() * r[2][2]);
            points.push_back(pr);
        }
        double sum = 0;
        for (unsigned int i = 0; i < points.size(); i++){
            Pointd p1 = points[i];
            Pointd p2 = points[(i+1)%points.size()];
            sum += (p2.x() - p1.x()) * (p2.y()+p1.y());
        }
        if (sum > 0)
            normal = -normal;
    }
    return normal;
}

/**
 * \~Italian
 * @brief Funzione che aggiorna l'area della faccia
 * @warning Funziona se e solo se la faccia è un triangolo
 * @warning Utilizza Dcel::Face::ConstIncidentVertexIterator
 * @return L'area della faccia aggiornata
 */
float Dcel::Face::updateArea() {
    updateNormal();
    if (isTriangle()) {
        Pointd v1 = outerHalfEdge->getFromVertex()->getCoordinate();
        Pointd v2 = outerHalfEdge->getToVertex()->getCoordinate();
        Pointd v3 = outerHalfEdge->getPrev()->getFromVertex()->getCoordinate();
        area = (((v3 - v1).cross(v2 - v1)).getLength() / 2);
    }
    #ifdef CGAL_DEFINED
    else {
        area = 0;
        std::vector<std::tuple<const Dcel::Vertex*, const Dcel::Vertex*, const Dcel::Vertex*> > t;

        getTriangulation(t);
        for (unsigned int i = 0; i <t.size(); ++i){
            std::tuple<const Dcel::Vertex*, const Dcel::Vertex*, const Dcel::Vertex*> tr =  t[i];
            Pointd v1 = std::get<0>(tr)->getCoordinate();
            Pointd v2 = std::get<1>(tr)->getCoordinate();
            Pointd v3 = std::get<2>(tr)->getCoordinate();
            area += (((v3 - v1).cross(v2 - v1)).getLength() / 2);
        }
    }
    #endif
    return area;
}

/**
 * \~Italian
 * @brief Funzione che rimuove un inner half edge dalla faccia
 * @param[in] iterator: iteratore che punta all'inner half edge da eliminare
 */
void Dcel::Face::removeInnerHalfEdge(const Face::InnerHalfEdgeIterator& iterator) {
    innerHalfEdges.erase(iterator);
}

/**
 * \~Italian
 * @brief Funzione che rimuove un inner half edge dalla faccia
 *
 * È meno efficiente rispetto a Dcel::Face::removeInnerHalfEdge(const Face::innerHalfEdgeIterator &ihe).
 *
 * @param[in] halfEdge: inner half edge da eliminare
 * @return True se la rimozione è andata a buon fine, false altrimenti.
 */
bool Dcel::Face::removeInnerHalfEdge(const Dcel::HalfEdge* halfEdge) {
    InnerHalfEdgeIterator i = std::find(innerHalfEdges.begin(), innerHalfEdges.end(), halfEdge);
    if (i != innerHalfEdges.end()){
        innerHalfEdges.erase(i);
        return true;
    }
    return false;
}

void Dcel::Face::removeAllInnerHalfEdges() {
    innerHalfEdges.clear();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::InnerHalfEdgeIterator
 * @return Un iteratore che punta al primo inner half edge della faccia
 */
Dcel::Face::InnerHalfEdgeIterator Dcel::Face::innerHalfEdgeBegin() {
    return innerHalfEdges.begin();
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::InnerHalfEdgeIterator
 * @return Un iteratore che punta all'ultimo inner half edge della faccia
 */
Dcel::Face::InnerHalfEdgeIterator Dcel::Face::innerHalfEdgeEnd() {
    return innerHalfEdges.end();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'outer half edge.
 *
 * @return Un iteratore che punta all'outer half edge della faccia
 */
Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin() {
    return IncidentHalfEdgeIterator(outerHalfEdge, outerHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::IncidentHalfEdgeIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeEnd() {
    return IncidentHalfEdgeIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta essere incidente alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin(Dcel::HalfEdge* start) {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return IncidentHalfEdgeIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentHalfEdgeIterator.
 *
 * Permette di ciclare sugli half edge incidenti alla faccia, partendo dall'half edge start e fino all'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta all'half edge start
 */
Dcel::Face::IncidentHalfEdgeIterator Dcel::Face::incidentHalfEdgeBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end) {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    if (end->getFace() != this){
        std::cerr << "ERROR: end half edge " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return IncidentHalfEdgeIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'outer half edge.
 *
 * @return Un iteratore che punta al toVertex dell'outer half edge della faccia
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexBegin() {
    return IncidentVertexIterator(outerHalfEdge, outerHalfEdge, this);
}

/**
 * \~Italian
 * @brief Funzione di finalizzazione di Dcel::Face::IncidentVertexIterator
 * @warning L'iteratore restituito non è un iteratore valido, e su di esso le operazioni di
 *          incremento e decremento sono inutili. Questa funzione è da utilizzarsi solamente per
 *          il \b confronto \b con \b un \b altro \b iteratore \b valido
 * @return Un iteratore di finalizzazione
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexEnd() {
    return IncidentVertexIterator(nullptr, nullptr, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'half edge start.
 *
 * @param[in] start: half edge di partenza
 * @warning Se start non risulta essere incidente alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexBegin(Dcel::HalfEdge* start) {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return IncidentVertexIterator(start, start, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal toVertex dell'half edge start e fino al toVertex dell'half edge end.
 *
 * @param[in] start: half edge di partenza
 * @param[in] end: half edge di arrivo, il cui toVertex \b non \b è \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia, viene lanciata un'asserzione e il programma termina
 * @return Un iteratore che punta al toVertex dell'half edge start
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexBegin(Dcel::HalfEdge* start, Dcel::HalfEdge* end) {
    #ifdef DEBUG
    if (start->getFace() != this){
        std::cerr << "ERROR: start half edge " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    if (end->getFace() != this){
        std::cerr << "ERROR: end half edge " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    return IncidentVertexIterator(start, end, this);
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal vertice start. \n
 * È meno efficiente rispetto a Dcel::Face::incidentVertexBegin(const Dcel::HalfEdge* start).
 *
 * @param[in] start: vertice di partenza
 * @warning Se start non risulta essere incidente alla faccia (ossia non possiede un half edge incidente alla faccia),
 *          viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexBegin(Dcel::Vertex* start) {
    for (Dcel::Vertex::IncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
         heit!= start->incomingHalfEdgeEnd();
         ++heit)
    {
        if ((*heit)->getFace() == this) return IncidentVertexIterator(*heit, *heit, this);
    }
    std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
    assert(0);
    return IncidentVertexIterator();
}

/**
 * \~Italian
 * @brief Funzione di inizializzazione di Dcel::Face::IncidentVertexIterator.
 *
 * Permette di ciclare sui vertici incidenti alla faccia, partendo dal vertice start e fino al vertice end. \n
 * È meno efficiente rispetto a Dcel::Face::incidentVertexBegin(const Dcel::HalfEdge* start, const Dcel::HalfEdge* end).
 *
 * @param[in] start: vertice di partenza
 * @param[in] end: vertice di arrivo, \b non \b compreso
 * @warning Se start e end non risultano essere incidenti alla faccia (ossia non possiedono un half edge incidente alla faccia),
 *          viene lanciata un'asserzione e il programma termina
 * @warning Utilizza Dcel::Vertex::ConstIncomingHalfEdgeIterator
 * @return Un iteratore che punta al vertice start
 */
Dcel::Face::IncidentVertexIterator Dcel::Face::incidentVertexBegin(Dcel::Vertex* start, Dcel::Vertex* end) {
    Vertex::IncomingHalfEdgeIterator heit = start->incomingHalfEdgeBegin();
    while (heit!= start->incomingHalfEdgeEnd() && ((*heit)->getFace() != this)) ++heit;
    #ifdef DEBUG
    if ((*heit)->getFace() != this) {
        std::cerr << "ERROR: start vertex " << start->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
        assert(0);
    }
    #endif
    HalfEdge* s = *heit;
    for (heit= end->incomingHalfEdgeBegin(); heit!= end->incomingHalfEdgeEnd(); ++heit){
        if ((*heit)->getFace() == this) return IncidentVertexIterator(s, *heit, this);
    }
    std::cerr << "ERROR: end vertex " << end->getId() << " hasn't this face " << this->getId() << " as incident face.\n";
    assert(0);
    return IncidentVertexIterator();
}

#ifdef CGAL_DEFINED
/**
 * \~Italian
 * @brief Dcel::Face::getTriangulation
 * @param triangles
 */
void Dcel::Face::getTriangulation(std::vector<std::tuple<const Dcel::Vertex*, const Dcel::Vertex*, const Dcel::Vertex*> > &triangles) const {
    // Taking all the coordinates on vectors
    std::vector<const Dcel::Vertex*> borderCoordinates;
    std::vector< std::vector<const Dcel::Vertex*> > innerBorderCoordinates;
    std::map<CGALPoint, const Dcel::Vertex*> pointsVerticesMap;
    for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = incidentHalfEdgeBegin(); heit != incidentHalfEdgeEnd(); ++heit){
        borderCoordinates.push_back((*heit)->getFromVertex());
        std::pair<const Dcel::Vertex*, const Dcel::Vertex*> pp;
        pp.first = (*heit)->getFromVertex();
        pp.second = (*heit)->getToVertex();
    }

    if (hasHoles()){
        innerBorderCoordinates.reserve(getNumberInnerHalfEdges());
        int i = 0;
        for (Dcel::Face::ConstInnerHalfEdgeIterator ihe = innerHalfEdgeBegin(); ihe != innerHalfEdgeEnd(); ++ihe, ++i){
            const Dcel::HalfEdge* he = *ihe;
            std::vector<const Dcel::Vertex*> inner;
            for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = incidentHalfEdgeBegin(he); heit != incidentHalfEdgeEnd(); ++heit){
                inner.push_back((*heit)->getFromVertex());
                std::pair<const Dcel::Vertex*, const Dcel::Vertex*> pp;
                pp.first = (*heit)->getFromVertex();
                pp.second = (*heit)->getToVertex();
            }
            innerBorderCoordinates.push_back(inner);
        }
    }

    //Rotation of the coordinates
    Vec3 faceNormal = normal;
    Vec3 zAxis(0,0,1);
    Vec3 v = -(faceNormal.cross(zAxis));
    v.normalize();
    double dot = faceNormal.dot(zAxis);
    double angle = acos(dot);

    double r[3][3] = {0};
    if (faceNormal != zAxis){
        if (faceNormal == -zAxis){
            v = Vec3(1,0,0);
        }
        getRotationMatrix(v, angle, r);
    }
    else {
        r[0][0] = r[1][1] = r[2][2] = 1;
    }

    //rotate points and make 2D polygon
    Polygon_2 polygon1;
    std::vector<Polygon_2> innerPolygons;
    for (unsigned int i = 0; i < borderCoordinates.size(); ++i){
        Pointd a = borderCoordinates[i]->getCoordinate();
        Pointd p1(a.x() * r[0][0] + a.y() * r[1][0] +a.z() * r[2][0], a.x() * r[0][1] + a.y() * r[1][1] +a.z() * r[2][1], a.x() * r[0][2] + a.y() * r[1][2] +a.z() * r[2][2]);
        CGALPoint p(p1.x(), p1.y());
        polygon1.push_back(p);
        pointsVerticesMap[p] = borderCoordinates[i];
    }
    if (hasHoles()){
        for (unsigned int i = 0; i < innerBorderCoordinates.size(); ++i) {
            Polygon_2 innerPolygon;
            for (unsigned j = 0; j < innerBorderCoordinates[i].size(); ++j) {
                Pointd a = innerBorderCoordinates[i][j]->getCoordinate();
                Pointd p1(a.x() * r[0][0] + a.y() * r[1][0] + a.z() * r[2][0],
                          a.x() * r[0][1] + a.y() * r[1][1] + a.z() * r[2][1],
                          a.x() * r[0][2] + a.y() * r[1][2] + a.z() * r[2][2]);
                CGALPoint p(p1.x(), p1.y());
                innerPolygon.push_back(p);
                pointsVerticesMap[p] = innerBorderCoordinates[i][j];
            }
            innerPolygons.push_back(innerPolygon);
        }
    }

    ///TRIANGULATION

    CDT cdt;
    cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
    for (unsigned int i = 0; i < innerPolygons.size(); ++i)
        cdt.insert_constraint(innerPolygons[i].vertices_begin(), innerPolygons[i].vertices_end(), true);
    markDomains(cdt);

    triangles.clear();
    for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end();++fit) {
        if ( fit->info().in_domain() ) {
            Triangle triangle = *fit;
            CDT::Vertex_handle v = triangle.vertex(0);
            const CGALPoint p1 = v->point();
            v = triangle.vertex(1);
            const CGALPoint p2 = v->point();
            v = triangle.vertex(2);
            const CGALPoint p3 = v->point();
            const Dcel::Vertex* a = pointsVerticesMap[p1];
            const Dcel::Vertex* b = pointsVerticesMap[p2];
            const Dcel::Vertex* c = pointsVerticesMap[p3];
            std::tuple<const Dcel::Vertex*, const Dcel::Vertex*, const Dcel::Vertex*> tuple(a, b, c);
            triangles.push_back(tuple);
        }
    }
}
#endif


/*********************
 * Protected Methods *
 *********************/

/**
 * \~Italian
 * @brief Funzione che restituisce una stringa degli inner half edge
 * @return Una stringa rappresentativa degli inner half edge della faccia
 */
std::string Dcel::Face::innerComponentsToString() const {
    std::stringstream ss;
    ss << "(";
    for (unsigned int i=0; i<innerHalfEdges.size(); i++){
        if (innerHalfEdges[i] != nullptr) {
            if (i != 0) ss << "; ";
            ss << innerHalfEdges[i]->getId();
        }
        else ss << "nullptr";
    }
    ss << ")";
    std::string s1 = ss.str();
    return s1;
}
