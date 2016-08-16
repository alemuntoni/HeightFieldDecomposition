#ifndef SEGMENTATION_STRUCT_H
#define SEGMENTATION_STRUCT_H

#include "dcel/dcel.h"
#include "common/comparators.h"
#include <set>
#include <functional>

#define N_LABELS 18

class Segmentation {
    public:

        class Chart;

        Segmentation();
        Segmentation(const Dcel &d, const std::function<bool(const Dcel::Face*, const Dcel::Face*)> &f = sameChartOperator);
        Segmentation(const Segmentation &s);
        Segmentation& operator=(const Segmentation &s);
        ~Segmentation();

        int getNumCharts() const;
        unsigned int getNumFaces() const;
        Chart* getChart(const Dcel::Face *f);
        const Chart* getChart(const Dcel::Face *f) const;
        void deleteChart(Chart* p);
        bool deleteFace(const Dcel::Face* f);
        Chart* addChart(const Dcel::Face* seed);
        void addFaceOnChart(const Dcel::Face* f, Chart* p);
        bool faceExists(const Dcel::Face* f) const;
        bool isFaceOnBorder(const Dcel::Face* f) const;
        bool isCornerVertex(const Dcel::Vertex* v) const;
        int reflood(Chart* p);
        void mergeCharts(Chart* p1, Chart* p2);
        void clear();
        std::vector<Chart*> getChartVector();
        float getAverageArea() const;
        Dcel getDcelFromSegmentation() const;

        //class patch_iterator;

        typedef std::set<Chart*>::iterator ChartIterator;

        Segmentation::ChartIterator chartBegin();
        Segmentation::ChartIterator chartEnd();

        class ConstChartIterator;

        Segmentation::ConstChartIterator chartBegin() const;
        Segmentation::ConstChartIterator chartEnd() const;

        class ConstFaceIterator;

        Segmentation::ConstFaceIterator faceBegin() const;
        Segmentation::ConstFaceIterator faceEnd() const;

    protected:
        std::map<const Dcel::Face*, Chart*> s;
        std::set<Chart*> charts;
        BoundingBox boundingBox;
        int id = 0;
        std::function<bool(const Dcel::Face*, const Dcel::Face*)> fun;

        const Dcel::HalfEdge* nextHalfEdgeOnBorder(const Dcel::HalfEdge* he) const;
        void getFaceFromChart(const Segmentation::Chart* p, Dcel& d, std::map<const Dcel::Vertex*, Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<Pointd, Pointd>, Dcel::HalfEdge* >& halfEdgesMap) const;
        Dcel::HalfEdge* makeBorder(Dcel& d, const Dcel::HalfEdge* firstHalfEdgeOnborder, Dcel::Face* actualFace, std::map<const Dcel::Vertex*, Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<Pointd, Pointd>, Dcel::HalfEdge*>& halfEdgesMap, std::set<const Dcel::HalfEdge*>& borderHalfEdges, BoundingBox& bb) const;
        static bool sameChartOperator(const Dcel::Face* f1, const Dcel::Face* f2);
};

inline Segmentation::Segmentation() {
}

inline int Segmentation::getNumCharts() const {
    return charts.size();
}

inline Segmentation::Chart* Segmentation::getChart(const Dcel::Face *f) {
    if (s.find(f) != s.end()) return s.at(f);
    else return nullptr;
}

inline const Segmentation::Chart* Segmentation::getChart(const Dcel::Face *f) const {
    if (s.find(f) != s.end()) return s.at(f);
    else return nullptr;
}

inline bool Segmentation::faceExists(const Dcel::Face *f) const {
    if (s.find(f) == s.end()) return false;
    else return true;
}
#endif // SEGMENTATION_STRUCT_H
