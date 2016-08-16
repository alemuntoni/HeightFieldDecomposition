#ifndef SEGMENTATION_ITERATORS_H
#define SEGMENTATION_ITERATORS_H

#include "segmentation_struct.h"

class Segmentation::ConstChartIterator {
        friend class Segmentation;

    public:
        ConstChartIterator();
        ConstChartIterator(const Segmentation::ChartIterator &it);
        const Segmentation::Chart* operator *() const;
        ConstChartIterator operator ++();
        ConstChartIterator operator ++(int);
        ConstChartIterator operator --();
        ConstChartIterator operator --(int);
        bool operator ==(const ConstChartIterator &right) const;
        bool operator !=(const ConstChartIterator &right) const;

    protected:
        std::set<Segmentation::Chart*>::const_iterator it;
};

class Segmentation::ConstFaceIterator {
        friend class Segmentation;

    public:
        ConstFaceIterator();
        const Dcel::Face* operator *() const;
        ConstFaceIterator operator ++();
        ConstFaceIterator operator ++(int);
        ConstFaceIterator operator --();
        ConstFaceIterator operator --(int);
        bool operator ==(const ConstFaceIterator &right) const;
        bool operator !=(const ConstFaceIterator &right) const;
        const Segmentation::Chart* get_patch_id() const;

    private:
        std::map<const Dcel::Face*, Segmentation::Chart*>::const_iterator it;
        ConstFaceIterator(const std::map<const Dcel::Face*, Segmentation::Chart*>::const_iterator &it);
};

/**
 * @brief Segmentation::const_patch_iterator
 */
inline Segmentation::ConstChartIterator::ConstChartIterator(){
}

inline Segmentation::ConstChartIterator::ConstChartIterator(const Segmentation::ChartIterator &it) {
    this->it = it;
}

inline const Segmentation::Chart* Segmentation::ConstChartIterator::operator *() const {
    return *it;
}

inline Segmentation::ConstChartIterator Segmentation::ConstChartIterator::operator ++() {
    ++it;
    return *this;
}

inline Segmentation::ConstChartIterator Segmentation::ConstChartIterator::operator ++(int) {
    ConstChartIterator old_value = *this;
    ++it;
    return old_value;
}

inline Segmentation::ConstChartIterator Segmentation::ConstChartIterator::operator --() {
    --it;
    return *this;
}

inline Segmentation::ConstChartIterator Segmentation::ConstChartIterator::operator --(int) {
    ConstChartIterator old_value = *this;
    --it;
    return old_value;
}

inline bool Segmentation::ConstChartIterator::operator ==(const ConstChartIterator &right) const {
    return this->it == right.it;
}

inline bool Segmentation::ConstChartIterator::operator !=(const ConstChartIterator &right) const {
    return !(*this == right);
}

/**
 * @brief Segmentation::const_face_iterator
 */
inline Segmentation::ConstFaceIterator::ConstFaceIterator() {
}

inline Segmentation::ConstFaceIterator::ConstFaceIterator(const std::map<const Dcel::Face*, Chart*>::const_iterator &it) {
    this->it = it;
}

inline const Dcel::Face* Segmentation::ConstFaceIterator::operator *() const {
    return (it->first);
}

inline Segmentation::ConstFaceIterator Segmentation::ConstFaceIterator::operator ++() {
    ++it;
    return *this;
}

inline Segmentation::ConstFaceIterator Segmentation::ConstFaceIterator::operator ++(int) {
    ConstFaceIterator old_value = *this;
    ++it;
    return old_value;
}

inline Segmentation::ConstFaceIterator Segmentation::ConstFaceIterator::operator --() {
    --it;
    return *this;
}

inline Segmentation::ConstFaceIterator Segmentation::ConstFaceIterator::operator --(int) {
    ConstFaceIterator old_value = *this;
    --it;
    return old_value;
}

inline bool Segmentation::ConstFaceIterator::operator ==(const Segmentation::ConstFaceIterator &right) const {
    return this->it == right.it;
}

inline bool Segmentation::ConstFaceIterator::operator !=(const ConstFaceIterator &right) const {
    return !(*this == right);
}

inline const Segmentation::Chart* Segmentation::ConstFaceIterator::get_patch_id() const {
    return it->second;
}
#endif // SEGMENTATION_ITERATORS_H
