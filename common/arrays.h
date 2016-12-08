#ifndef ARRAY3D_H
#define ARRAY3D_H

#include <vector>
#include <assert.h>
#include <iomanip>
#include "serialize.h"

template <class T> class Array2D : public SerializableObject{

    public:
        Array2D();
        Array2D(size_t sizeX, size_t sizeY);
        Array2D(size_t sizeX, size_t sizeY, const T& value);
        T& operator () (size_t i, size_t j);
        T operator () (size_t i, size_t j) const;
        const T* operator () (size_t i) const;

        size_t getSizeX() const;
        size_t getSizeY() const;

        void setConstant(const T& c);

        void resize (size_t x, size_t y);
        void resize (size_t x, size_t y, const T& value);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        bool deserialize(std::ifstream& binaryFile);

    private:
        size_t getIndex(size_t i, size_t j) const;

        size_t sizeX, sizeY;
        std::vector<T> v;
};

template <class T>
std::ostream& operator<< (std::ostream& inputStream, const Array2D<T>& a);

template <class T> class Array3D : public SerializableObject{

    public:
        Array3D();
        Array3D(size_t sizeX, size_t sizeY, size_t sizeZ);
        Array3D(size_t sizeX, size_t sizeY, size_t sizeZ, const T& value);
        T& operator () (size_t i, size_t j, size_t k);
        T operator () (size_t i, size_t j, size_t k) const;
        const T* operator () (size_t i, size_t j) const;

        size_t getSizeX() const;
        size_t getSizeY() const;
        size_t getSizeZ() const;

        void setConstant(const T& c);

        void resize (size_t x, size_t y, size_t z);
        void resize (size_t x, size_t y, size_t z, const T& value);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        bool deserialize(std::ifstream& binaryFile);

    private:
        size_t getIndex(size_t i, size_t j, size_t k) const;

        size_t sizeX, sizeY, sizeZ;
        std::vector<T> v;
};

template <class T> class Array4D : public SerializableObject{

    public:
        Array4D();
        Array4D(size_t sizeX, size_t sizeY, size_t sizeZ, size_t sizeW);
        Array4D(size_t sizeX, size_t sizeY, size_t sizeZ, size_t sizeW, const T& value);
        T& operator () (size_t i, size_t j, size_t k, size_t l);
        T operator () (size_t i, size_t j, size_t k, size_t l) const;
        const T* operator () (size_t i, size_t j, size_t k) const;

        size_t getSizeX() const;
        size_t getSizeY() const;
        size_t getSizeZ() const;
        size_t getSizeW() const;

        void setConstant(const T& c);

        void resize (size_t x, size_t y, size_t z, size_t w);
        void resize (size_t x, size_t y, size_t z, size_t w, const T& value);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        bool deserialize(std::ifstream& binaryFile);

    private:
        size_t getIndex(size_t i, size_t j, size_t k, size_t l);

        size_t sizeX, sizeY, sizeZ, sizeW;
        std::vector<T> v;
};

///
/// Array2D
///

template <class T>
inline Array2D<T>::Array2D() : sizeX(0), sizeY(0) {
    v.resize(0);
}

template <class T>
inline Array2D<T>::Array2D(size_t sizeX, size_t sizeY) : sizeX(sizeX), sizeY(sizeY) {
    v.resize(sizeX*sizeY);
}

template <class T>
inline Array2D<T>::Array2D(size_t sizeX, size_t sizeY, const T& value) : sizeX(sizeX), sizeY(sizeY) {
    v.resize(sizeX*sizeY, value);
}

template <class T>
inline T& Array2D<T>::operator ()(size_t i, size_t j) {
    return v[getIndex(i,j)];
}

template <class T>
inline T Array2D<T>::operator ()(size_t i, size_t j) const {
    return v[getIndex(i,j)];
}

template <class T>
inline const T* Array2D<T>::operator ()(size_t i) const{
    assert (i < sizeX);
    return &(v[sizeY*i]);
}

template <class T>
inline size_t Array2D<T>::getSizeX() const {
    return sizeX;
}

template <class T>
inline size_t Array2D<T>::getSizeY() const{
    return sizeY;
}

template <class T>
inline void Array2D<T>::setConstant(const T& c) {
    std::fill(v.begin(), v.end(), c);
}

template <class T>
inline void Array2D<T>::resize(size_t x, size_t y) {
    v.resize(x*y);
    sizeX = x;
    sizeY = y;
}

template <class T>
inline void Array2D<T>::resize(size_t x, size_t y, const T& value) {
    v.resize(x*y, value);
    sizeX = x;
    sizeY = y;
}

template <class T>
inline void Array2D<T>::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(sizeX, binaryFile);
    Serializer::serialize(sizeY, binaryFile);
    for (unsigned int i = 0; i < v.size(); ++i)
        Serializer::serialize(v[i], binaryFile);
}

template <class T>
inline bool Array2D<T>::deserialize(std::ifstream& binaryFile) {
    Array2D<T> tmp;
    if (Serializer::deserialize(tmp.sizeX, binaryFile) &&
            Serializer::deserialize(tmp.sizeY, binaryFile)) {
        tmp.v.resize(tmp.sizeX*tmp.sizeY);
        for (unsigned int i = 0; i < tmp.v.size(); ++i){
            if (! Serializer::deserialize(tmp.v[i], binaryFile))
                return false;
        }
        *this = std::move(tmp);
        return true;
    }
    else
        return false;
}

template <class T>
inline size_t Array2D<T>::getIndex(size_t i, size_t j) const {
    assert (i < sizeX);
    assert (j < sizeY);
    return j + sizeY*i;
}

template <class T>
std::ostream& operator<< (std::ostream& inputStream, const Array2D<T>& a) {
    for (unsigned int i = 0; i < a.getSizeX(); i++){
        for (unsigned int j = 0; j < a.getSizeY(); j++){
            inputStream << std::setw(4) <<a(i,j) << " ";
        }
        inputStream << "\n";
    }
    return inputStream;
}

///
/// Array3D
///

template <class T>
inline Array3D<T>::Array3D() : sizeX(0), sizeY(0), sizeZ(0) {
    v.resize(0);
}

template <class T>
inline Array3D<T>::Array3D(size_t sizeX, size_t sizeY, size_t sizeZ) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ) {
    v.resize(sizeX*sizeY*sizeZ);
}

template <class T>
inline Array3D<T>::Array3D(size_t sizeX, size_t sizeY, size_t sizeZ, const T& value) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ) {
    v.resize(sizeX*sizeY*sizeZ, value);
}

template <class T>
inline T& Array3D<T>::operator ()(size_t i, size_t j, size_t k) {
    return v[getIndex(i,j,k)];
}

template <class T>
inline T Array3D<T>::operator ()(size_t i, size_t j, size_t k) const {
    return v[getIndex(i,j,k)];
}

template <class T>
inline const T* Array3D<T>::operator ()(size_t i, size_t j) const{
    assert (i < sizeX);
    assert (j < sizeY);
    return &(v[sizeZ*(j + sizeY*i)]);
}

template <class T>
inline size_t Array3D<T>::getSizeX() const {
    return sizeX;
}

template <class T>
inline size_t Array3D<T>::getSizeY() const{
    return sizeY;
}

template <class T>
inline size_t Array3D<T>::getSizeZ() const {
    return sizeZ;
}

template <class T>
inline void Array3D<T>::setConstant(const T& c) {
    std::fill(v.begin(), v.end(), c);
}

template <class T>
inline void Array3D<T>::resize(size_t x, size_t y, size_t z) {
    v.resize(x*y*z);
    sizeX = x;
    sizeY = y;
    sizeZ = z;
}

template <class T>
inline void Array3D<T>::resize(size_t x, size_t y, size_t z, const T& value) {
    v.resize(x*y*z, value);
    sizeX = x;
    sizeY = y;
    sizeZ = z;
}

template <class T>
inline void Array3D<T>::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(sizeX, binaryFile);
    Serializer::serialize(sizeY, binaryFile);
    Serializer::serialize(sizeZ, binaryFile);
    for (unsigned int i = 0; i < v.size(); ++i)
        Serializer::serialize(v[i], binaryFile);
}

template <class T>
inline bool Array3D<T>::deserialize(std::ifstream& binaryFile) {
    Array3D<T> tmp;
    if (Serializer::deserialize(tmp.sizeX, binaryFile) &&
            Serializer::deserialize(tmp.sizeY, binaryFile) &&
            Serializer::deserialize(tmp.sizeZ, binaryFile)) {
        tmp.v.resize(tmp.sizeX*tmp.sizeY*tmp.sizeZ);
        for (unsigned int i = 0; i < tmp.v.size(); ++i){
            if (! Serializer::deserialize(tmp.v[i], binaryFile))
                return false;
        }
        *this = std::move(tmp);
        return true;
    }
    else
        return false;
}

template <class T>
inline size_t Array3D<T>::getIndex(size_t i, size_t j, size_t k) const {
    assert (i < sizeX);
    assert (j < sizeY);
    assert (k < sizeZ);
    return k+sizeZ*(j + sizeY*i);
}

///
/// Array4D
///

template <class T>
inline Array4D<T>::Array4D() : sizeX(0), sizeY(0), sizeZ(0), sizeW(0) {
    v.resize(0);
}

template <class T>
inline Array4D<T>::Array4D(size_t sizeX, size_t sizeY, size_t sizeZ, size_t sizeW) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ), sizeW(sizeW) {
    v.resize(sizeX*sizeY*sizeZ*sizeW);
}

template <class T>
inline Array4D<T>::Array4D(size_t sizeX, size_t sizeY, size_t sizeZ, size_t sizeW, const T& value) : sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ), sizeW(sizeW) {
    v.resize(sizeX*sizeY*sizeZ*sizeW, value);
}

template <class T>
inline T& Array4D<T>::operator ()(size_t i, size_t j, size_t k, size_t l) {
    return v[getIndex(i,j,k,l)];
}

template <class T>
inline T Array4D<T>::operator ()(size_t i, size_t j, size_t k, size_t l) const {
    return v[getIndex(i,j,k,l)];
}

template <class T>
inline const T* Array4D<T>::operator ()(size_t i, size_t j, size_t k) const {
    assert (i < sizeX);
    assert (j < sizeY);
    assert (k < sizeZ);
    return &(v[sizeW*(k + sizeZ*(j + sizeY*i))]);
}

template <class T>
inline size_t Array4D<T>::getSizeX() const {
    return sizeX;
}

template <class T>
inline size_t Array4D<T>::getSizeY() const{
    return sizeY;
}

template <class T>
inline size_t Array4D<T>::getSizeZ() const {
    return sizeZ;
}

template <class T>
inline size_t Array4D<T>::getSizeW() const {
    return sizeW;
}

template <class T>
inline void Array4D<T>::setConstant(const T& c) {
    std::fill(v.begin(), v.end(), c);
}

template <class T>
inline void Array4D<T>::resize(size_t x, size_t y, size_t z, size_t w) {
    v.resize(x*y*z*w);
    sizeX = x;
    sizeY = y;
    sizeZ = z;
    sizeW = w;
}

template <class T>
inline void Array4D<T>::resize(size_t x, size_t y, size_t z, size_t w, const T& value) {
    v.resize(x*y*z*w, value);
    sizeX = x;
    sizeY = y;
    sizeZ = z;
    sizeW = w;
}

template <class T>
inline void Array4D<T>::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(sizeX, binaryFile);
    Serializer::serialize(sizeY, binaryFile);
    Serializer::serialize(sizeZ, binaryFile);
    Serializer::serialize(sizeW, binaryFile);
    for (unsigned int i = 0; i < v.size(); ++i)
        Serializer::serialize(v[i], binaryFile);
}

template <class T>
inline bool Array4D<T>::deserialize(std::ifstream& binaryFile) {
    Array4D<T> tmp;
    if (Serializer::deserialize(tmp.sizeX, binaryFile) &&
            Serializer::deserialize(tmp.sizeY, binaryFile) &&
            Serializer::deserialize(tmp.sizeZ, binaryFile) &&
            Serializer::deserialize(tmp.sizeW, binaryFile)) {
        tmp.v.resize(tmp.sizeX*tmp.sizeY*tmp.sizeZ*tmp.sizeW);
        for (unsigned int i = 0; i < tmp.v.size(); ++i){
            if (! Serializer::deserialize(tmp.v[i], binaryFile))
                return false;
        }
        *this = std::move(tmp);
        return true;
    }
    else
        return false;
}

template <class T>
inline size_t Array4D<T>::getIndex(size_t i, size_t j, size_t k, size_t l) {
    assert (i < sizeX);
    assert (j < sizeY);
    assert (k < sizeZ);
    assert (l < sizeW);
    return l + sizeW*(k + sizeZ*(j + sizeY*i));
}

#endif // ARRAY3D_H
