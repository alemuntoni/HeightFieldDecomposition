#ifndef SEGMENTATION_STRUCT_H
#define SEGMENTATION_STRUCT_H

#include "cg3/meshes/dcel/dcel.h"
#include "cg3/utilities/comparators.h"
#include <set>
#include <functional>

#define N_LABELS 18

class Segmentation {
    public:

        class Chart;

        Segmentation();
        Segmentation(const cg3::Dcel &d, const std::function<bool(const cg3::Dcel::Face*, const cg3::Dcel::Face*)> &f = sameChartOperator);
        Segmentation(const Segmentation &s);
        Segmentation& operator=(const Segmentation &s);
        ~Segmentation();

        int getNumCharts() const;
        unsigned int getNumFaces() const;
        Chart* getChart(const cg3::Dcel::Face *f);
        const Chart* getChart(const cg3::Dcel::Face *f) const;
        void deleteChart(Chart* p);
        bool deleteFace(const cg3::Dcel::Face* f);
        Chart* addChart(const cg3::Dcel::Face* seed);
        void addFaceOnChart(const cg3::Dcel::Face* f, Chart* p);
        bool faceExists(const cg3::Dcel::Face* f) const;
        bool isFaceOnBorder(const cg3::Dcel::Face* f) const;
        bool isCornerVertex(const cg3::Dcel::Vertex* v) const;
        int reflood(Chart* p);
        void mergeCharts(Chart* p1, Chart* p2);
        void clear();
        std::vector<Chart*> getChartVector();
        float getAverageArea() const;
        cg3::Dcel getDcelFromSegmentation() const;

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
        std::map<const cg3::Dcel::Face*, Chart*> s;
        std::set<Chart*> charts;
        cg3::BoundingBox boundingBox;
        int id = 0;
        std::function<bool(const cg3::Dcel::Face*, const cg3::Dcel::Face*)> fun;

        const cg3::Dcel::HalfEdge* nextHalfEdgeOnBorder(const cg3::Dcel::HalfEdge* he) const;
        void getFaceFromChart(const Segmentation::Chart* p, cg3::Dcel& d, std::map<const cg3::Dcel::Vertex*, cg3::Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<cg3::Pointd, cg3::Pointd>, cg3::Dcel::HalfEdge* >& halfEdgesMap) const;
        cg3::Dcel::HalfEdge* makeBorder(cg3::Dcel& d, const cg3::Dcel::HalfEdge* firstHalfEdgeOnborder, cg3::Dcel::Face* actualFace, std::map<const cg3::Dcel::Vertex*, cg3::Dcel::Vertex*> &verticesDcelToSeg, std::map<std::pair<cg3::Pointd, cg3::Pointd>, cg3::Dcel::HalfEdge*>& halfEdgesMap, std::set<const cg3::Dcel::HalfEdge*>& borderHalfEdges, cg3::BoundingBox& bb) const;
        static bool sameChartOperator(const cg3::Dcel::Face* f1, const cg3::Dcel::Face* f2);
};

inline Segmentation::Segmentation() {
}

inline int Segmentation::getNumCharts() const {
    return charts.size();
}

inline Segmentation::Chart* Segmentation::getChart(const cg3::Dcel::Face *f) {
    if (s.find(f) != s.end()) return s.at(f);
    else return nullptr;
}

inline const Segmentation::Chart* Segmentation::getChart(const cg3::Dcel::Face *f) const {
    if (s.find(f) != s.end()) return s.at(f);
    else return nullptr;
}

inline bool Segmentation::faceExists(const cg3::Dcel::Face *f) const {
    if (s.find(f) == s.end()) return false;
    else return true;
}
#endif // SEGMENTATION_STRUCT_H
