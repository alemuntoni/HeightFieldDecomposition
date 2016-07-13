/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef SERIALIZE_H
#define SERIALIZE_H

#include <string>
#include <fstream>
#include <set>
#include <vector>
#include <list>
#include <map>
#include <QColor>
#include <typeinfo>
#include <Eigen/Core>
#include <array>

#include <type_traits> // To use 'std::integral_constant'.

/**
 * \~English
 * @interface SerializableObject
 *
 * @brief The SerializableObject interface
 *
 *
 * If an class implements this interface, it becames "Serializable".
 * In that case, rules for serialization and deserialization of tha class are given on
 * SerializableObject::serialize() and SerializableObject::deserialize() methods.
 */
class SerializableObject
{
    public:
        SerializableObject() {}
        virtual ~SerializableObject() {}

        /**
         * \~English
         * @brief serialize
         *
         * How to use:
         *
         * std::ofstream myfile;\n
         * //overwrite: \n
         * myfile.open ("fileName", std::ios::out | std::ios::binary);
         * //or append:\n
         * myfile.open ("fileName", std::ios::out | std::ios::binary | std::ios::app);
         *
         * if (myfile.is_open()) {\n
         *     myobj.serialize(myfile);\n
         * }\n
         * myfile.close();
         *
         * @param[in] binaryFile : ofstream when we want to serialize the object
         */
        virtual void serialize(std::ofstream& binaryFile) const = 0;

        /**
         * \~English
         * @brief deserialize
         *
         * How to use:
         *
         * std::ifstream myfile;\n
         * myfile.open ("filename", std::ios::in | std::ios::binary);\n
         * if (myfile.is_open()) {\n
         *     myobj.deserialize(myfile);\n
         * }\n
         * myfile.close();\n
         *
         * @param[in] binaryFile : ifstream when we want to read the object
         */
        virtual void deserialize(std::ifstream& binaryFile) = 0;
};

/**
 * \~English
 * @namespace Serializer
 *
 * @brief Please, if you can, add serialize/deserialize method for all types you need that don't work
 * with the standard "serialize"/"deserialize" methods!
 *
 * \~Italian
 * @namespace Serializer
 * @brief Supporta la serializzazione/deserializzazione di tutti i tipi primitivi più:
 * - QColor
 * - std::set<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::vector<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::list<T,...> dove :
 *         T è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 * - std::map<T1,T2,...> dove:
 *         T1 è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 *         T2 è un tipo primitivo o un SerializableObject (NON puntatore a SerializableObject)
 */
namespace Serializer {

    template <typename T> void serialize(const T& obj, std::ofstream& binaryFile);

    template <typename T> void deserialize(T& obj, std::ifstream& binaryFile);

    void serialize(const QColor& obj, std::ofstream& binaryFile);

    void deserialize(QColor& obj, std::ifstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::set<T, A...> &s, std::ofstream& binaryFile);

    template <typename T, typename ...A> void deserialize(std::set<T, A...> &s, std::ifstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::vector<T, A...> &v, std::ofstream& binaryFile);

    template <typename T, typename ...A> void deserialize(std::vector<T, A...> &v, std::ifstream& binaryFile);

    template <typename T, typename ...A> void serialize(const std::list<T, A...> &l, std::ofstream& binaryFile);

    template <typename T, typename ...A> void deserialize(std::list<T, A...> &l, std::ifstream& binaryFile);

    template <typename T1, typename T2, typename ...A> void serialize(const std::map<T1, T2, A...> &m, std::ofstream& binaryFile);

    template <typename T1, typename T2, typename ...A> void deserialize(const std::map<T1, T2, A...> &m, std::ifstream& binaryFile);

    template <typename T, int ...A> void serialize(const Eigen::Matrix<T, A...> &m, std::ofstream& binaryFile);

    template <typename T, int ...A> void deserialize(Eigen::Matrix<T, A...> &m, std::ifstream& binaryFile);

    template <typename T, size_t ...A> void serialize(const std::array<T, A...> &a, std::ofstream& binaryFile);

    template <typename T, size_t ...A> void deserialize(std::array<T, A...> &a, std::ifstream& binaryFile);
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
inline void Serializer::deserialize(T& obj, std::ifstream& binaryFile){
    if (std::is_base_of<SerializableObject, T>::value){
        SerializableObject* o =(SerializableObject*) &obj;
        o->deserialize(binaryFile);
    }
    else
        binaryFile.read(reinterpret_cast<char*>(&obj), sizeof(T));
}

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
inline void Serializer::deserialize(QColor& obj, std::ifstream& binaryFile){
    int r, g, b, a;
    Serializer::deserialize(r, binaryFile);
    Serializer::deserialize(g, binaryFile);
    Serializer::deserialize(b, binaryFile);
    Serializer::deserialize(a, binaryFile);
    obj.setRgb(r,g,b,a);
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
inline void Serializer::deserialize(std::set<T, A...> &s, std::ifstream& binaryFile){
    size_t size;
    Serializer::deserialize(size, binaryFile);
    for (unsigned int it = 0; it < size; ++it){
        T obj;
        Serializer::deserialize(obj, binaryFile);
        s.insert(obj);
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
template <typename T, typename ...A>
inline void Serializer::deserialize(std::vector<T, A...> &v, std::ifstream& binaryFile){
    size_t size;
    Serializer::deserialize(size, binaryFile);
    v.resize(size);
    for (unsigned int it = 0; it < size; ++it){
        Serializer::deserialize(v[it], binaryFile);
    }
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
inline void Serializer::deserialize(std::list<T, A...> &l, std::ifstream& binaryFile){
    size_t size;
    Serializer::deserialize(size, binaryFile);
    for (unsigned int it = 0; it < size; ++it){
        T obj;
        Serializer::deserialize(obj, binaryFile);
        l.push_back(obj);
    }
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
inline void Serializer::deserialize(const std::map<T1, T2, A...> &m, std::ifstream& binaryFile){
    size_t size;
    Serializer::deserialize(size, binaryFile);

    for (unsigned int it = 0; it < size; ++it){
        T1 o1;
        T2 o2;

        Serializer::deserialize(o1, binaryFile);
        Serializer::deserialize(o2, binaryFile);
        m[o1] = o2;
    }
}

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
inline void Serializer::deserialize(Eigen::Matrix<T, A...> &m, std::ifstream& binaryFile){
    size_t row, col;
    Serializer::deserialize(row, binaryFile);
    Serializer::deserialize(col, binaryFile);
    m.resize(row, col);

    for (unsigned int i = 0; i < row; i++){
        for (unsigned int j = 0; j < col; ++j){
            Serializer::deserialize(m(i,j), binaryFile);
        }
    }
}

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
inline void Serializer::deserialize(std::array<T, A...> &a, std::ifstream& binaryFile){
    size_t size;
    Serializer::deserialize(size, binaryFile);
    assert(size == a.size());
    for (unsigned int it = 0; it < size; ++it){
        Serializer::deserialize(a[it], binaryFile);
    }
}

#endif // SERIALIZE_H
