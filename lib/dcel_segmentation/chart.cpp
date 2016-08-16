#include "chart.h"

Segmentation::Chart::Chart(){
}

Segmentation::Chart::Chart(int id, const Dcel::Face *seed, std::map<const Dcel::Face *, Chart *> &s, const std::function<bool(const Dcel::Face*, const Dcel::Face*)>& f) : id(id), fun(f) {
    std::vector<const Dcel::Face *> stack_faces; // only triangles with same label of the patch will stay on the stack
    normal = seed->getNormal();

    s[seed] = this;
    faces.insert(seed);

    // adding neighbor triangles (if they have same label) to the stack
    for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = seed->incidentHalfEdgeBegin(); heit != seed->incidentHalfEdgeEnd(); ++heit){
        if ((*heit)->getTwin() != nullptr) {
            const Dcel::Face* adjacent = (*heit)->getTwin()->getFace();
            if (fun(seed, adjacent)) stack_faces.push_back(adjacent);
        }
    }

    // while there aren't other triangles on the stack
    while (stack_faces.size() > 0) {
        const Dcel::Face* fi = stack_faces[stack_faces.size()-1];
        stack_faces.pop_back(); //pop
        faces.insert(fi);
        s[fi] = this;
        for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = fi->incidentHalfEdgeBegin(); heit != fi->incidentHalfEdgeEnd(); ++heit) {
            if ((*heit)->getTwin() != nullptr) {
                const Dcel::Face* adjacent = (*heit)->getTwin()->getFace();
                if (fun(seed, adjacent)) {
                    if (faces.find(adjacent) == faces.end())
                        stack_faces.push_back(adjacent);
                }
            }
        }
    }

    area = 0;
    for (std::set<const Dcel::Face *>::const_iterator fit = faces.cbegin(); fit != faces.cend(); ++fit){
        const Dcel::Face* f = *fit;
        area += f->getArea();
    }
}

std::set<const Dcel::Face *> Segmentation::Chart::reflood(const Dcel::Face *seed) {
    std::set<const Dcel::Face *> old_faces = faces;
    faces.clear();
    std::vector<const Dcel::Face *> stack_faces; // only triangles with same label of the patch will stay on the stack
    faces.insert(seed);
    old_faces.erase(seed);
    for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = seed->incidentHalfEdgeBegin(); heit != seed->incidentHalfEdgeEnd(); ++heit){
        if ((*heit)->getTwin() != nullptr) {
            const Dcel::Face* adjacent = (*heit)->getTwin()->getFace();
            if (adjacent->getNormal() ==  normal) stack_faces.push_back(adjacent);
        }
    }
    while (stack_faces.size() > 0) {
        const Dcel::Face* fi = stack_faces[stack_faces.size()-1];
        stack_faces.pop_back(); //pop
        faces.insert(fi);
        old_faces.erase(fi);
        for (Dcel::Face::ConstIncidentHalfEdgeIterator heit = fi->incidentHalfEdgeBegin(); heit != fi->incidentHalfEdgeEnd(); ++heit) {
            if ((*heit)->getTwin() != nullptr) {
                const Dcel::Face* adjacent = (*heit)->getTwin()->getFace();
                if (adjacent->getNormal() == normal) {
                    if (faces.find(adjacent) == faces.end())
                        stack_faces.push_back(adjacent);
                }
            }
        }
    }
    area = 0;
    for (std::set<const Dcel::Face *>::const_iterator fit = faces.cbegin(); fit != faces.cend(); ++fit){
        const Dcel::Face* f = *fit;
        area += f->getArea();
    }
    return old_faces;

}

Segmentation::Chart::ConstFaceIterator Segmentation::Chart::faceBegin() const {
    return ConstFaceIterator(faces.begin());
}

Segmentation::Chart::ConstFaceIterator Segmentation::Chart::faceEnd() const {
    return ConstFaceIterator(faces.end());
}
