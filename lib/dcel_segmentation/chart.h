#ifndef PATCH_H
#define PATCH_H

#include <set>
#include "cg3/meshes/dcel/dcel.h"
#include "segmentation_struct.h"

class Segmentation::Chart {
        friend class Segmentation;
    public:
        Chart();
        ~Chart();
        float getArea() const;
        int getId() const;
        int getNumFaces() const;
        cg3::Vec3 getNormal() const;
        bool faceExists(const cg3::Dcel::Face* f) const;
        const cg3::Dcel::Face* getSeed() const;

        typedef std::set<const cg3::Dcel::Face *>::iterator ConstFaceIterator;

        ConstFaceIterator faceBegin() const;
        ConstFaceIterator faceEnd() const;

    protected:
        Chart(int id, const cg3::Dcel::Face *seed, std::map<const cg3::Dcel::Face *, Chart *> &s, const std::function<bool(const cg3::Dcel::Face*, const cg3::Dcel::Face*)>& f);
        std::set<const cg3::Dcel::Face *> reflood(const cg3::Dcel::Face *seed);
        void deleteFace(const cg3::Dcel::Face* f);
        void addFace(const cg3::Dcel::Face* f);

        int id;
        std::set<const cg3::Dcel::Face *> faces;
        cg3::Vec3 normal;
        float area;
        std::function<bool(const cg3::Dcel::Face*, const cg3::Dcel::Face*)> fun;
};

inline Segmentation::Chart::~Chart(){
}

inline float Segmentation::Chart::getArea() const {
    return area;
}

inline int Segmentation::Chart::getId() const {
    return id;
}

inline int Segmentation::Chart::getNumFaces() const {
    return faces.size();
}

inline cg3::Vec3 Segmentation::Chart::getNormal() const {
    return normal;
}

inline bool Segmentation::Chart::faceExists(const cg3::Dcel::Face* f) const {
    if (faces.find(f) == faces.end()) return false;
    else return true;
}

inline const cg3::Dcel::Face*Segmentation::Chart::getSeed() const {
    assert(faces.size() > 0);
    return *(faces.begin());
}

inline void Segmentation::Chart::deleteFace(const cg3::Dcel::Face *f) {
    area -= f->getArea();
    faces.erase(f);
}

inline void Segmentation::Chart::addFace(const cg3::Dcel::Face *f) {
    area += f->getArea();
    faces.insert(f);
}

#endif // PATCH_H
