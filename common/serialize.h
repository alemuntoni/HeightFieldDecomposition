/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef SERIALIZE_H
#define SERIALIZE_H

#include "serializable_object.h"

#include <string>
#include <set>
#include <vector>
#include <list>
#include <map>
#ifdef QT_CORE_LIB
#include <QColor>
#endif //QT_CORE_LIB
#include <typeinfo>
#ifdef COMMON_WITH_EIGEN
#include <Eigen/Core>
#endif //COMMON_WITH_EIGEN
#include <array>

#include <type_traits> // To use 'std::integral_constant'.

/**
 * \~English
 * @namespace Serializer
 *
 * @brief Please, if you can, add serialize/deserialize methods for all types you need that don't work
 * with the standard "serialize"/"deserialize" methods!
 *
 * \~Italian
 * @namespace Serializer
 * @brief Supporta la serializzazione/deserializzazione di tutti i tipi primitivi più:
 * - QColor
 * - std::string
 * - std::set<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::vector<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::list<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::map<T1,T2,...> dove:
 *         T1 è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 *         T2 è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::array<T,size_t,...> dove:
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 *         size_t è la dimensione dell'array
 */
namespace Serializer {

    template <typename T> void serialize(const T& obj, std::ofstream& binaryFile);

    template <typename T> bool deserialize(T& obj, std::ifstream& binaryFile);

    #ifdef QT_CORE_LIB
    void serialize(const QColor& obj, std::ofstream& binaryFile);

    bool deserialize(QColor& obj, std::ifstream& binaryFile);
    #endif

    void serialize(const std::string& str, std::ofstream& binaryFile);

    bool deserialize(std::string& str, std::ifstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::set<T, A...> &s, std::ofstream& binaryFile);

    template <typename T, typename ...A> bool deserialize(std::set<T, A...> &s, std::ifstream& binaryFile);

    template <typename ...A> void serialize(const std::vector<bool, A...> &v, std::ofstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::vector<T, A...> &v, std::ofstream& binaryFile);

    template <typename ...A> bool deserialize(std::vector<bool, A...> &v, std::ifstream& binaryFile);

    template <typename T, typename ...A> bool deserialize(std::vector<T, A...> &v, std::ifstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::list<T, A...> &l, std::ofstream& binaryFile);

    template <typename T, typename ...A> bool deserialize(std::list<T, A...> &l, std::ifstream& binaryFile);

    template <typename T1, typename T2, typename ...A> void serialize(const std::map<T1, T2, A...> &m, std::ofstream& binaryFile);

    template <typename T1, typename T2, typename ...A> bool deserialize(const std::map<T1, T2, A...> &m, std::ifstream& binaryFile);

    #ifdef COMMON_WITH_EIGEN
    template <typename T, int ...A> void serialize(const Eigen::Matrix<T, A...> &m, std::ofstream& binaryFile);

    template <typename T, int ...A> bool deserialize(Eigen::Matrix<T, A...> &m, std::ifstream& binaryFile);
    #endif //COMMON_WITH_EIGEN

    template <typename T, size_t ...A> void serialize(const std::array<T, A...> &a, std::ofstream& binaryFile);

    template <typename T, size_t ...A> bool deserialize(std::array<T, A...> &a, std::ifstream& binaryFile);
}

/**
 * \~Italian
 * @brief
 *
 * \~English
 * @brief Serializer::serialize
 *
 * This function allows to serialize on a std::ofstream opened in binary mode:
 *
 * - All primitive types;
 * - All classes that have correctly implemented the abstract class SerializableObject
 *
 * This method will be called if there is not a specialized "serialize" method for the type of object
 * that you are passing as first parameter (see specialized methods below).
 *
 * @param[in] obj: object which we want serialize
 * @param[in] binaryFile: std::ofstream opened in binary mode on the file where we want to serialize
 */
template <typename T>
inline void Serializer::serialize(const T& obj, std::ofstream& binaryFile){
    if (std::is_base_of<SerializableObject, T>::value){
        SerializableObject* o =(SerializableObject*) &obj;
        o->serialize(binaryFile);
    }
    else
        binaryFile.write(reinterpret_cast<const char*>(&obj), sizeof(T));
}

/**
 * \~English
 * @brief Serializer::deserialize
 *
 * This function allows to deserialize on a std::ifstream opened in binary mode:
 *
 * - All primitive types
 * - All classes that have correctly implemented the abstract class SerializableObject
 *
 * All you have to do is to call all the deserialize methods in the same order of the methods
 * serialize.
 *
 * This method will be called if there is not a specialized "deserialize" method for the type of object
 * that you are passing as first parameter (see specialized methods below).
 *
 * @param[out] obj: the object that we want to load
 * @param[in] binaryFile: std::ifstream opened in binary mode on the file we want to deserialize
 */
template <typename T>
inline bool Serializer::deserialize(T& obj, std::ifstream& binaryFile){
    if (std::is_base_of<SerializableObject, T>::value){
        SerializableObject* o =(SerializableObject*) &obj;
        return o->deserialize(binaryFile);
    }
    else{
        if (binaryFile.read(reinterpret_cast<char*>(&obj), sizeof(T)))
            return true;
        else
            return false;
    }
}

#ifdef QT_CORE_LIB
/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] obj: QColor
 * @param binaryFile
 */
inline void Serializer::serialize(const QColor& obj, std::ofstream& binaryFile){
    int r = obj.red(), g = obj.green(), b = obj.blue(), a = obj.alpha();
    Serializer::serialize(r, binaryFile);
    Serializer::serialize(g, binaryFile);
    Serializer::serialize(b, binaryFile);
    Serializer::serialize(a, binaryFile);
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @param[out] obj: QColor
 * @param binaryFile
 */
inline bool Serializer::deserialize(QColor& obj, std::ifstream& binaryFile){
    int r, g, b, a;
    if (Serializer::deserialize(r, binaryFile) &&
        Serializer::deserialize(g, binaryFile) &&
        Serializer::deserialize(b, binaryFile) &&
        Serializer::deserialize(a, binaryFile)) {
            obj.setRgb(r,g,b,a);
            return true;
    }
    else
        return false;
}
#endif

inline void Serializer::serialize(const std::string& str, std::ofstream& binaryFile){
    size_t size=str.size();
    Serializer::serialize(size, binaryFile);
    binaryFile.write(&str[0],size);
}

inline bool Serializer::deserialize(std::string& str, std::ifstream& binaryFile){
    size_t size;
    std::string tmp;
    if (Serializer::deserialize(size, binaryFile)){
        tmp.resize(size);
        if (binaryFile.read(&tmp[0], size)){
            str = std::move(tmp);
            return true;
        }
    }
    else
        return false;
}

/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] s: std::set
 * @param binaryFile
 */
template <typename T, typename ...A>
inline void Serializer::serialize(const std::set<T, A...> &s, std::ofstream& binaryFile){
    size_t size = s.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::set<T, A...>::const_iterator it = s.begin(); it != s.end(); ++it)
        Serializer::serialize((*it), binaryFile);
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input set is an empty set.
 * @param[out] s: std::set
 * @param binaryFile
 */
template <typename T, typename ...A>
inline bool Serializer::deserialize(std::set<T, A...> &s, std::ifstream& binaryFile){
    std::set<T, A...> tmp;
    size_t size;
    if (Serializer::deserialize(size, binaryFile)){
        for (unsigned int it = 0; it < size; ++it){
            T obj;
            if(Serializer::deserialize(obj, binaryFile))
                tmp.insert(obj);
            else
                return false;
        }
        s = std::move(tmp);
        return true;
    }
    else
        return false;
}

/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] v: std::vector of booleans
 * @param binaryFile
 */
template <typename ...A>
inline void Serializer::serialize(const std::vector<bool, A...> &v, std::ofstream& binaryFile){
    bool tmp;
    size_t size = v.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::vector<bool, A...>::const_iterator it = v.begin(); it != v.end(); ++it){
        if (*it) tmp = 1;
        else tmp = 0;
        Serializer::serialize(tmp, binaryFile);
    }
}

/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] v: std::vector
 * @param binaryFile
 */
template <typename T, typename ...A>
inline void Serializer::serialize(const std::vector<T, A...> &v, std::ofstream& binaryFile){
    size_t size = v.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::vector<T, A...>::const_iterator it = v.begin(); it != v.end(); ++it)
        Serializer::serialize((*it), binaryFile);
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input vector is an empty vector (it will be overwritten)
 * @param[out] v: std::vector
 * @param binaryFile
 */
template <typename ...A>
inline bool Serializer::deserialize(std::vector<bool, A...> &v, std::ifstream& binaryFile){
    size_t size;
    std::vector<bool, A...> tmpv;
    if (Serializer::deserialize(size, binaryFile)){
        tmpv.resize(size);
        bool tmp;
        for (unsigned int it = 0; it < size; ++it){
            if (Serializer::deserialize(tmp, binaryFile))
                tmpv[it] = tmp;
            else
                return false;
        }
        v = std::move(tmpv);
        return true;
    }
    else
        return false;
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input vector is an empty vector (it will be overwritten)
 * @param[out] v: std::vector
 * @param binaryFile
 */
template <typename T, typename ...A>
inline bool Serializer::deserialize(std::vector<T, A...> &v, std::ifstream& binaryFile){
    size_t size;
    std::vector<T, A...> tmpv;
    if (Serializer::deserialize(size, binaryFile)){
        tmpv.resize(size);
        for (unsigned int it = 0; it < size; ++it){
            if (! Serializer::deserialize(tmpv[it], binaryFile))
                return false;
        }
        v = std::move(tmpv);
        return true;
    }
    else
        return false;
}

/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] l: std::list
 * @param binaryFile
 */
template <typename T, typename ...A>
inline void Serializer::serialize(const std::list<T, A...> &l, std::ofstream& binaryFile){
    size_t size = l.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::list<T, A...>::const_iterator it = l.begin(); it != l.end(); ++it)
        Serializer::serialize((*it), binaryFile);
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input list is an empty list
 * @param[out] l: std::list
 * @param binaryFile
 */
template <typename T, typename ...A>
inline bool Serializer::deserialize(std::list<T, A...> &l, std::ifstream& binaryFile){
    size_t size;
    std::list<T, A...> tmp;

    if (Serializer::deserialize(size, binaryFile)){
        for (unsigned int it = 0; it < size; ++it){
            T obj;
            if (Serializer::deserialize(obj, binaryFile))
                tmp.push_back(obj);
            else
                return false;
        }
        l = std::move(tmp);
        return true;
    }
    else
        return false;
}

/**
 * \~English
 * @brief Serializer::serialize
 * @param[out] m: std::map
 * @param binaryFile
 */
template <typename T1, typename T2, typename ...A>
inline void Serializer::serialize(const std::map<T1, T2, A...> &m, std::ofstream& binaryFile){
    size_t size = m.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::map<T1, T2, A...>::const_iterator it = m.begin(); it != m.end(); ++it){
        Serializer::serialize((it->first), binaryFile);
        Serializer::serialize((it->second), binaryFile);
    }
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input map is an empty map
 * @param[in] m: std::map
 * @param binaryFile
 */
template <typename T1, typename T2, typename ...A>
inline bool Serializer::deserialize(const std::map<T1, T2, A...> &m, std::ifstream& binaryFile){
    size_t size;
    std::map<T1, T2, A...> tmp;

    if (Serializer::deserialize(size, binaryFile)) {

        for (unsigned int it = 0; it < size; ++it){
            T1 o1;
            T2 o2;

            if (Serializer::deserialize(o1, binaryFile) &&
                Serializer::deserialize(o2, binaryFile))
                    tmp[std::move(o1)] = std::move(o2);
            else
                return false;
        }
        m = std::move(tmp);
        return true;
    }
    else
        return false;
}

#ifdef COMMON_WITH_EIGEN
/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] m: Eigen::Matrix
 * @param binaryFile
 */
template <typename T, int ...A>
inline void Serializer::serialize(const Eigen::Matrix<T, A...> &m, std::ofstream& binaryFile){
    size_t row = m.rows(), col = m.cols();
    Serializer::serialize(row, binaryFile);
    Serializer::serialize(col, binaryFile);
    for (unsigned int i = 0; i < row; i++){
        for (unsigned int j = 0; j < col; ++j){
            Serializer::serialize(m(i,j), binaryFile);
        }
    }
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input set is an empty set.
 * @param[out] m: Eigen::Matrix
 * @param binaryFile
 */
template <typename T, int ...A>
inline bool Serializer::deserialize(Eigen::Matrix<T, A...> &m, std::ifstream& binaryFile){
    size_t row, col;
    Eigen::Matrix<T, A...> tmp;
    if (Serializer::deserialize(row, binaryFile) && Serializer::deserialize(col, binaryFile)) {
        tmp.resize(row, col);

        for (unsigned int i = 0; i < row; i++){
            for (unsigned int j = 0; j < col; ++j){
                if (! Serializer::deserialize(tmp(i,j), binaryFile))
                    return false;
            }
        }
        m = std::move(tmp);
        return true;
    }
    else
        return false;
}
#endif //COMMON_WITH_EIGEN

/**
 * \~English
 * @brief Serializer::serialize
 * @param[in] m: Eigen::Matrix
 * @param binaryFile
 */
template <typename T, size_t ...A>
inline void Serializer::serialize(const std::array<T, A...> &a, std::ofstream& binaryFile){
    size_t size = a.size();
    Serializer::serialize(size, binaryFile);
    for (typename std::array<T, A...>::iterator it = a.begin(); it != a.end(); ++it)
        Serializer::serialize((*it), binaryFile);
}

/**
 * \~English
 * @brief Serializer::deserialize
 * @warning please make sure that the input set is an empty set.
 * @param[out] m: Eigen::Matrix
 * @param binaryFile
 */
template <typename T, size_t ...A>
inline bool Serializer::deserialize(std::array<T, A...> &a, std::ifstream& binaryFile){
    size_t size;
    if (Serializer::deserialize(size, binaryFile) && size == a.size()){
        std::vector<T> tmp(size);
        for (unsigned int it = 0; it < size; ++it){
            if (! Serializer::deserialize(tmp[it], binaryFile)){
                return false;
            }
        }
        std::copy_n(tmp.begin(), size, a.begin());
        return true;
    }
    else
        return false;
}

#endif // SERIALIZE_H
