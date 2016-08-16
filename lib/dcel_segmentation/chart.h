#ifndef PATCH_H
#define PATCH_H

#include <set>
#include "dcel/dcel.h"
#include "segmentation_struct.h"

class Segmentation::Chart {
        friend class Segmentation;
    public:
        Chart();
        ~Chart();
        float getArea() const;
        int getId() const;
        int getNumFaces() const;
        Vec3 getNormal() const;
        bool faceExists(const Dcel::Face* f) const;
        const Dcel::Face* getSeed() const;

        typedef std::set<const Dcel::Face *>::iterator ConstFaceIterator;

        ConstFaceIterator faceBegin() const;
        ConstFaceIterator faceEnd() const;

    protected:
        Chart(int id, const Dcel::Face *seed, std::map<const Dcel::Face *, Chart *> &s, const std::function<bool(const Dcel::Face*, const Dcel::Face*)>& f);
        std::set<const Dcel::Face *> reflood(const Dcel::Face *seed);
        void deleteFace(const Dcel::Face* f);
        void addFace(const Dcel::Face* f);

        int id;
        std::set<const Dcel::Face *> faces;
        Vec3 normal;
        float area;
        std::function<bool(const Dcel::Face*, const Dcel::Face*)> fun;
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

inline Vec3 Segmentation::Chart::getNormal() const {
    return normal;
}

inline bool Segmentation::Chart::faceExists(const Dcel::Face* f) const {
    if (faces.find(f) == faces.end()) return false;
    else return true;
}

inline const Dcel::Face*Segmentation::Chart::getSeed() const {
    assert(faces.size() > 0);
    return *(faces.begin());
}

inline void Segmentation::Chart::deleteFace(const Dcel::Face *f) {
    area -= f->getArea();
    faces.erase(f);
}

inline void Segmentation::Chart::addFace(const Dcel::Face *f) {
    area += f->getArea();
    faces.insert(f);
}

#endif // PATCH_H
