#include "chart.h"
#include "segmentation_iterators.h"
#include "common.h"

using namespace cg3;

Segmentation::Segmentation(const Dcel &d, const std::function<bool(const Dcel::Face*, const Dcel::Face*)>& f) {
    id = 0;
    fun = f;
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        if (s.find(*fit) == s.end()){
            Chart* p = new Chart(id++, *fit, s, fun);
            charts.insert(p);
        }
    }
    boundingBox = d.getBoundingBox();
}

Segmentation::Segmentation(const Segmentation &s) {
    for (ConstChartIterator pit = s.chartBegin(); pit != s.chartEnd(); ++pit){
        const Dcel::Face* f = *((*pit)->faceBegin());
        Chart * p = new Chart((*pit)->getId(), f, this->s, s.fun);
        charts.insert(p);
    }
    id = s.id;
    boundingBox = s.boundingBox;
    fun = s.fun;
}

Segmentation &Segmentation::operator=(const Segmentation &s) {
    std::set<Chart*>::iterator it;
    int i = 0;
    for (it = charts.begin(); it != charts.end(); ++it){
        Chart* p = *it;
        delete p;
        i++;
    }
    charts.clear();
    this->s.clear();
    for (ConstChartIterator pit = s.chartBegin(); pit != s.chartEnd(); ++pit){
        const Dcel::Face* f = *((*pit)->faceBegin());
        Chart * p = new Chart((*pit)->getId(), f, this->s, s.fun);
        charts.insert(p);
    }
    id = s.id;
    boundingBox = s.boundingBox;
    fun = s.fun;
    return *this;
}

Segmentation::~Segmentation() {
    std::set<Chart*>::iterator it;
    for (it = charts.begin(); it != charts.end(); ++it){
        Chart* p = *it;
        delete p;
    }

}

unsigned int Segmentation::getNumFaces() const {
    return s.size();
}

Segmentation::Chart* Segmentation::addChart(const Dcel::Face *seed) {
    Segmentation::Chart* p = new Segmentation::Chart(id++, seed, s, fun);
    charts.insert(p);
    return p;
}

void Segmentation::addFaceOnChart(const Dcel::Face *f, Segmentation::Chart *p) {
    p->addFace(f);
    s[f] = p;
}

bool Segmentation::isFaceOnBorder(const Dcel::Face* f) const {
    const Chart* p = getChart(f);
    for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = f->incidentHalfEdgeBegin(); heit != f->incidentHalfEdgeEnd(); ++heit){
        if (!(p->faceExists((*heit)->getTwin()->getFace()))) return true;
    }
    for (Dcel::Face::ConstIncidentVertexIterator vit = f->incidentVertexBegin(); vit != f->incidentVertexEnd(); ++vit) {
        const Dcel::Vertex* v = *vit;
        for (Dcel::Vertex::ConstIncidentFaceIterator fit = v->incidentFaceBegin(); fit != v->incidentFaceEnd(); ++fit) {
            if (!(p->faceExists(*fit))) return true;
        }
    }
    return false;
}

bool Segmentation::isCornerVertex(const Dcel::Vertex* v) const {
    std::set<const Chart*> incidentPatches;
    for (Dcel::Vertex::ConstIncidentFaceIterator fit = v->incidentFaceBegin(); fit!=v->incidentFaceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        incidentPatches.insert(getChart(f));
    }
    return (incidentPatches.size() > 2);

}

void Segmentation::deleteChart(Chart* p) {
    for (Chart::ConstFaceIterator fit = p->faceBegin(); fit != p->faceEnd(); ++fit){
        s.erase(*fit);
    }
    charts.erase(p);
    delete p;
}

bool Segmentation::deleteFace(const Dcel::Face *f) {
    std::map<const Dcel::Face*, Chart*>::iterator it = s.find(f);
    if (it != s.end()){
        Chart* p = it->second;
        p->deleteFace(f);
        s.erase(f);
        if (p->getNumFaces() == 0) {
            charts.erase(p);
            delete p;
        }
        return true;
    }
    else return false;
}

int Segmentation::reflood(Segmentation::Chart *p) {
    const Dcel::Face* f = *(p->faceBegin());
    std::set<const Dcel::Face*> set = p->reflood(f);
    if (set.size() > 0){
        int i = 0;
        for (std::set<const Dcel::Face*>::iterator it = set.begin(); it != set.end(); ++it){
            s.erase(*it);
        }
        while (set.size() > 0){
            const Dcel::Face* seed = *(set.begin());
            Segmentation::Chart* p = new Segmentation::Chart(id++, seed, s, fun);
            charts.insert(p);
            for (Segmentation::Chart::ConstFaceIterator fit = p->faceBegin(); fit != p->faceEnd(); ++fit){
                set.erase(*fit);
            }
            i++;
        }
        return i;
    }
    return 0;
}

void Segmentation::mergeCharts(Segmentation::Chart *p1, Segmentation::Chart *p2) {
    for (Segmentation::Chart::ConstFaceIterator fit = p2->faceBegin(); fit != p2->faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        p1->addFace(f);
        s[f] = p1;
    }
    charts.erase(p2);
    delete p2;
}

void Segmentation::clear() {
    s.clear();
    for (std::set<Chart*>::iterator it = charts.begin(); it != charts.end(); ++it){
        Chart* p = *it;
        delete p;
    }
    charts.clear();
    id = 0;
}

std::vector<Segmentation::Chart*> Segmentation::getChartVector() {
    return std::vector<Chart*>(charts.begin(), charts.end());
}

float Segmentation::getAverageArea() const {
    float area = 0;
    for (ConstChartIterator pit = chartBegin(); pit != chartEnd(); ++pit) area += (*pit)->getArea();
    area /= getNumCharts();
    return area;
}

Dcel Segmentation::getDcelFromSegmentation() const {
    std::map<const Dcel::Vertex*, Dcel::Vertex*> verticesDcelToSeg;
    std::map<std::pair<Pointd, Pointd>, Dcel::HalfEdge*> halfEdgesMap;
    Dcel d;
    for (ConstChartIterator pit = chartBegin(); pit != chartEnd(); ++pit){
        getFaceFromChart(*pit, d, verticesDcelToSeg, halfEdgesMap);
    }
    std::cerr << halfEdgesMap.size();
    for (Dcel::HalfEdgeIterator heit = d.halfEdgeBegin(); heit != d.halfEdgeEnd(); ++heit){
        Dcel::HalfEdge* he = *heit;

        if (he->getTwin() == nullptr)
            he->getFace()->setColor(Color(10,10,10));
    }
    d.updateBoundingBox();
    d.saveOnObjFile("SegNotW.obj");
    //assert(halfEdgesMap.size() == 0);

    return d;
}


Segmentation::ChartIterator Segmentation::chartBegin() {
    return ChartIterator(charts.begin());
}

Segmentation::ChartIterator Segmentation::chartEnd() {
    return ChartIterator(charts.end());
}

Segmentation::ConstChartIterator Segmentation::chartBegin() const {
    return ConstChartIterator(charts.begin());
}

Segmentation::ConstChartIterator Segmentation::chartEnd() const {
    return ConstChartIterator(charts.end());
}

Segmentation::ConstFaceIterator Segmentation::faceBegin() const {
    return ConstFaceIterator(s.begin());
}

Segmentation::ConstFaceIterator Segmentation::faceEnd() const {
    return ConstFaceIterator(s.end());
}

const Dcel::HalfEdge*Segmentation::nextHalfEdgeOnBorder(const Dcel::HalfEdge* he) const {
    const Dcel::Vertex* v = he->getToVertex();
    const Chart* p = getChart(he->getFace());
    const Dcel::HalfEdge* nextHalfEdge = nullptr;
    bool finded = false;
    for (Dcel::Vertex::ConstOutgoingHalfEdgeIterator heit = v->outgoingHalfEdgeBegin(he->getNext()); heit != v->outgoingHalfEdgeEnd() && !finded; ++heit) {
        nextHalfEdge = *heit;
        if (p->faceExists(nextHalfEdge->getFace()) && !(p->faceExists(nextHalfEdge->getTwin()->getFace())))
            finded = true;
    }
    return nextHalfEdge;
}

void Segmentation::getFaceFromChart(const Segmentation::Chart* p, Dcel& d, std::map<const Dcel::Vertex*, Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<Pointd, Pointd>, Dcel::HalfEdge*>& halfEdgesMap) const {
    Dcel::Face* actualFace = d.addFace();
    std::set<const Dcel::HalfEdge*> borderHalfEdges;
    const Dcel::HalfEdge* firstHalfEdgeOnborder = nullptr;

    // Ricerca half edge e faccia sul bordo
    for (Segmentation::Chart::ConstFaceIterator fit = p->faceBegin(); fit != p->faceEnd(); ++fit) {
        const Dcel::Face* f = *fit;
        const Chart* p = getChart(f);
        for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = f->incidentHalfEdgeBegin(); heit != f->incidentHalfEdgeEnd(); ++heit){
            if (!(p->faceExists((*heit)->getTwin()->getFace()))) {
                borderHalfEdges.insert(*heit);
            }
        }
    }

    firstHalfEdgeOnborder = *(borderHalfEdges.begin());
    while (!isCornerVertex(firstHalfEdgeOnborder->getFromVertex()))
        firstHalfEdgeOnborder = nextHalfEdgeOnBorder(firstHalfEdgeOnborder);

    BoundingBox bb;
    Dcel::HalfEdge* firstHalfEdge = makeBorder(d, firstHalfEdgeOnborder, actualFace, verticesDcelToSeg, halfEdgesMap, borderHalfEdges, bb);
    if (borderHalfEdges.size() == 0)
        actualFace->setOuterHalfEdge(firstHalfEdge);
    else { // buchi!
        std::vector<Dcel::HalfEdge*> holesHalfEdgesBorder;
        std::vector<double> diagBoundingBoxesBorder;
        holesHalfEdgesBorder.push_back(firstHalfEdge);
        diagBoundingBoxesBorder.push_back(bb.diag());
        while (borderHalfEdges.size() != 0){
            bb.reset();

            firstHalfEdgeOnborder = *(borderHalfEdges.begin());
            while (!isCornerVertex(firstHalfEdgeOnborder->getFromVertex()))
                firstHalfEdgeOnborder = nextHalfEdgeOnBorder(firstHalfEdgeOnborder);

            firstHalfEdge = makeBorder(d, firstHalfEdgeOnborder, actualFace, verticesDcelToSeg, halfEdgesMap, borderHalfEdges, bb);
            holesHalfEdgesBorder.push_back(firstHalfEdge);
            diagBoundingBoxesBorder.push_back(bb.diag());
        }

        //nell'array holesHalfEdgesBorder ho gli half edge iniziali di bordo. Ora devo capire qual è il bordo esterno e quali i bordi interni
        double max = 0;
        unsigned int j = 0;
        for (unsigned int i = 0; i < diagBoundingBoxesBorder.size(); i++){
            if (diagBoundingBoxesBorder[i] > max){
                max = diagBoundingBoxesBorder[i];
                j = i;
            }
        }
        actualFace->setOuterHalfEdge(holesHalfEdgesBorder[j]);
        for (unsigned int i = 0; i < holesHalfEdgesBorder.size(); i++){
            if (i != j){
                actualFace->addInnerHalfEdge(holesHalfEdgesBorder[i]);
            }
        }
    }

    actualFace->setNormal(p->getNormal());
    actualFace->setColor(colorOfNormal(p->getNormal()));
    actualFace->setArea(p->getArea());

}

Dcel::HalfEdge* Segmentation::makeBorder(Dcel& d, const Dcel::HalfEdge* firstHalfEdgeOnborder, Dcel::Face* actualFace, std::map<const Dcel::Vertex*, Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<Pointd, Pointd>, Dcel::HalfEdge*>& halfEdgesMap, std::set<const Dcel::HalfEdge*> &borderHalfEdges, BoundingBox &bb) const{
    const Dcel::HalfEdge* nextHalfEdge = firstHalfEdgeOnborder;
    Dcel::Vertex* lastCorner = nullptr;
    Dcel::Vertex* firstCorner = nullptr;
    Dcel::HalfEdge* lastHalfEdge = nullptr;
    Dcel::HalfEdge* firstHalfEdge = nullptr;
    Dcel::HalfEdge* actualHalfEdge = nullptr;

    // se il vertice sta già nella mappa dei vertici
    bb.setMin(bb.getMin().min(firstHalfEdgeOnborder->getFromVertex()->getCoordinate()));
    bb.setMax(bb.getMax().max(firstHalfEdgeOnborder->getFromVertex()->getCoordinate()));
    if (verticesDcelToSeg.find(firstHalfEdgeOnborder->getFromVertex()) == verticesDcelToSeg.end()){
        firstCorner = d.addVertex(firstHalfEdgeOnborder->getFromVertex()->getCoordinate());
        verticesDcelToSeg[firstHalfEdgeOnborder->getFromVertex()] = firstCorner;
    }
    else
        firstCorner = verticesDcelToSeg[firstHalfEdgeOnborder->getFromVertex()];
    lastCorner = firstCorner;

    do {
        borderHalfEdges.erase(nextHalfEdge);
        const Dcel::Vertex* v = nextHalfEdge->getToVertex();
        bb.setMin(bb.getMin().min(v->getCoordinate()));
        bb.setMax(bb.getMax().max(v->getCoordinate()));
        if (isCornerVertex(v)){
            Dcel::Vertex* actualCorner;

            if (verticesDcelToSeg.find(v) == verticesDcelToSeg.end()){
                actualCorner = d.addVertex(v->getCoordinate());
                verticesDcelToSeg[v] = actualCorner;
            }
            else
                actualCorner = verticesDcelToSeg[v];


            // Half Edge da lastCorner a actualCorner
            actualHalfEdge = d.addHalfEdge();
            std::pair<Pointd, Pointd> pair;
            pair.first = actualCorner->getCoordinate();
            pair.second = lastCorner->getCoordinate();
            if (halfEdgesMap.find(pair) != halfEdgesMap.end()){
                Dcel::HalfEdge* twin = halfEdgesMap[pair];
                actualHalfEdge->setTwin(twin);
                twin->setTwin(actualHalfEdge);
                halfEdgesMap.erase(pair);
            }
            else {
                pair.first = lastCorner->getCoordinate();
                pair.second = actualCorner->getCoordinate();
                halfEdgesMap[pair] = actualHalfEdge;
            }
            actualHalfEdge->setFromVertex(lastCorner);
            actualHalfEdge->setToVertex(actualCorner);
            lastCorner->setIncidentHalfEdge(actualHalfEdge);
            actualHalfEdge->setFace(actualFace);
            if (firstHalfEdge == nullptr) {
                firstHalfEdge = actualHalfEdge;
            }
            if (lastHalfEdge != nullptr){
                lastHalfEdge->setNext(actualHalfEdge);
                actualHalfEdge->setPrev(lastHalfEdge);
            }
            lastHalfEdge = actualHalfEdge;

            lastCorner = actualCorner;
        }
        nextHalfEdge = nextHalfEdgeOnBorder(nextHalfEdge);
    } while (nextHalfEdge != firstHalfEdgeOnborder);
    actualHalfEdge->setNext(firstHalfEdge);
    firstHalfEdge->setPrev(actualHalfEdge);

    return firstHalfEdge;

}

bool Segmentation::sameChartOperator(const Dcel::Face* f1, const Dcel::Face* f2) {
    return epsilonEqual(f1->getNormal(), f2->getNormal());
}
