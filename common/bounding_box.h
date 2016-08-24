/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#pragma once

#include <float.h>
#include "point.h"

/**
 * \~English
 * @class BoundingBox
 * @brief The BoundingBox class models a Bounding Box which contains a 3D mesh.
 *
 *        It is composed of two 3D points representing the minimum and maximum coordinates that generates the bounding box.
 *        The class provides methods like for obtaining the center or the diagonal of the bounding box.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author Marco Livesu (marco.livesu@gmail.com)
 *
 * \~Italian
 * @class BoundingBox
 * @brief La classe BoundingBox modella un Bounding Box contenente una mesh 3D.
 *
 *        Essa è caratterizzata da due punti contenenti le tre coordinate minime e massime che generano il bounding box.
 *        La classe fornisce metodi come il calcolo del centro e il calcolo della diagonale.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author Marco Livesu (marco.livesu@gmail.com)
 */
class BoundingBox : public SerializableObject{
    public:

        /****************
         * Constructors *
         ****************/

        BoundingBox();
        BoundingBox(const Pointd& min, const Pointd& max);

        /************************
        * Public Inline Methods *
        *************************/

        const Pointd& getMin()     const;
        const Pointd& getMax()     const;
        const double& getMinX()    const;
        const double& getMinY()    const;
        const double& getMinZ()    const;
        const double& getMaxX()    const;
        const double& getMaxY()    const;
        const double& getMaxZ()    const;
        double getLengthX()        const;
        double getLengthY()        const;
        double getLengthZ()        const;

        Pointd  center()    const;
        double diag()       const;
        bool isStrictlyIntern(const Pointd& p)     const;
        bool isStrictlyIntern(double px, double py, double pz)     const;
        bool isIntern(const Pointd& p)     const;
        bool isIntern(double px, double py, double pz)     const;

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;


        void setMin(const Pointd& min);
        void setMax(const Pointd& max);
        void setMin(double x, double y, double z);
        void setMax(double x, double y, double z);
        void setMinX(double x);
        void setMinY(double y);
        void setMinZ(double z);
        void setMaxX(double x);
        void setMaxY(double y);
        void setMaxZ(double z);
        void reset();

        // SerializableObject interface
        void deserialize(std::ifstream& binaryFile);

    protected:

        /**************
        * Attributes *
        **************/

        Pointd min; /**< \~English @brief Minimum point of the bounding box
                         \~Italian @brief Punto minimo del bounding box*/
        Pointd max; /**< \~English @brief Maximum point of the bounding box
                         \~Italian @brief Punto massimo del bounding box*/
};

/****************
 * Constructors *
 ****************/

/**
 * \~English
 * @brief Constructor, creates a bounding box and calls BoundingBox::reset() method
 *
 * \~Italian
 * @brief Costruttore, crea un bounding box e chiama il metodo BoundingBox::reset()
 */
inline BoundingBox::BoundingBox() {
    reset();
}

/**
 * \~English
 * @brief Constructor, creates a bounding box whith with minimum equal to the parameter \c min and maximum equal to the parameter \c max
 * @param[in] min: point setted as minimum point of the bounding box
 * @param[in] max: point setted as maximum point of the bounding box
 *
 * \~Italian
 * @brief Costruttore, crea un bounding box con limite minimo pari a \c min e limite massimo pari a \c max
 * @param[in] min: punto minimo che verrà settato
 * @param[in] max: punto massimo che verrà settato
 */
inline BoundingBox::BoundingBox(const Pointd& min, const Pointd& max) : min(min), max(max) {
}

/*************************
* Public Inline Methods *
*************************/

/**
 * \~English
 * @brief Returns the minimum point of the bounding box
 * @return The minimum point
 *
 * \~Italian
 * @brief Restituisce il punto minimo del bounding box
 * @return Il punto minimo
 */
inline const Pointd& BoundingBox::getMin() const {
    return min;
}

/**
 * \~English
 * @brief Returns the maximum point of the bounding box
 * @return The maximum point
 *
 * \~Italian
 * @brief Restituisce il punto massimo del bounding box
 * @return Il punto massimo
 */
inline const Pointd& BoundingBox::getMax() const {
    return max;
}

/**
 * \~English
 * @brief Returns the x component of the minimum point of the bounding box
 * @return X component of the minimum point
 *
 * \~Italian
 * @brief Restituisce la componente x del punto minimo del bounding box
 * @return La componente x del punto minimo
 */
inline const double& BoundingBox::getMinX() const {
    return min.x();
}

/**
 * \~English
 * @brief Returns the y component of the minimum point of the bounding box
 * @return Y component of the minimum point
 *
 * \~Italian
 * @brief Restituisce la componente y del punto minimo del bounding box
 * @return La componente y del punto minimo
 */
inline const double& BoundingBox::getMinY() const {
    return min.y();
}

/**
 * \~English
 * @brief Returns the z component of the minimum point of the bounding box
 * @return Z component of the minimum point
 *
 * \~Italian
 * @brief Restituisce la componente z del punto minimo del bounding box
 * @return La componente z del punto minimo
 */
inline const double& BoundingBox::getMinZ() const {
    return min.z();
}

/**
 * \~English
 * @brief Returns the x component of the maximum point of the bounding box
 * @return X component of the maximum point
 *
 * \~Italian
 * @brief Restituisce la componente x del punto massimo del bounding box
 * @return La componente x del punto massimo
 */
inline const double& BoundingBox::getMaxX() const {
    return max.x();
}

/**
 * \~English
 * @brief Returns the y component of the maximum point of the bounding box
 * @return Y component of the maximum point
 *
 * \~Italian
 * @brief Restituisce la componente y del punto massimo del bounding box
 * @return La componente y del punto massimo
 */
inline const double& BoundingBox::getMaxY() const {
    return max.y();
}

/**
 * \~English
 * @brief Returns the Z component of the maximum point of the bounding box
 * @return Z component of the maximum point
 *
 * \~Italian
 * @brief Restituisce la componente z del punto massimo del bounding box
 * @return La componente z del punto massimo
 */
inline const double& BoundingBox::getMaxZ() const {
    return max.z();
}

inline double BoundingBox::getLengthX() const {
    return max.x() - min.x();
}

inline double BoundingBox::getLengthY() const {
    return max.y() - min.y();
}

inline double BoundingBox::getLengthZ() const {
    return max.z() - min.z();
}

/**
 * \~English
 * @brief Calculates the center of the bounding box
 * @return The point centered in the bounding box
 *
 * \~Italian
 * @brief Calcola il centro del bounding box
 * @return Il punto al centro del bounding box
 */
inline Pointd BoundingBox::center() const {
    return (min + max) * 0.5;
}

/**
 * \~English
 * @brief Calculates the length of the diagonal of the bounding box
 * @return The length of the diagonal of the bounding box
 *
 * \~Italian
 * @brief Calcola la lunghezza della diagonale del bounding box
 * @return La lunghezza della diagonale del bounding box
 */
inline double BoundingBox::diag() const {
    return (min - max).getLength();
}

inline bool BoundingBox::isStrictlyIntern(const Pointd &p) const {
    return (p.x() > min.x() && p.y() > min.y() && p.z() > min.z() &&
            p.x() < max.x() && p.y() < max.y() && p.z() < max.z());
}

inline bool BoundingBox::isStrictlyIntern(double px, double py, double pz) const {
    return (px > min.x() && py > min.y() && pz > min.z() &&
            px < max.x() && py < max.y() && pz < max.z());
}

inline bool BoundingBox::isIntern(const Pointd &p) const {
    return (p.x() >= min.x() && p.y() >= min.y() && p.z() >= min.z() &&
            p.x() <= max.x() && p.y() <= max.y() && p.z() <= max.z());
}

inline bool BoundingBox::isIntern(double px, double py, double pz) const {
    return (px >= min.x() && py >= min.y() && pz >= min.z() &&
            px <= max.x() && py <= max.y() && pz <= max.z());
}

/**
 * \~English
 * @brief Serializes in a std::ofstream opened in binary mode the bounding box
 * @param an std::ofstream opened in binary mode
 *
 * \~Italian
 * @brief Serializza in un std::ofstream aperto in modalità binaria il bounding box
 * @param Un std::ofstream aperto in modalità binaria
 */
inline void BoundingBox::serialize(std::ofstream& binaryFile) const {
    min.serialize(binaryFile);
    max.serialize(binaryFile);
}

/**
 * \~English
 * @brief Modifies the minimum point of the bounding box
 * @param[in] min: point setted as minimum point
 *
 * \~Italian
 * @brief Modifica il punto minimo del bounding box
 * @param[in] min: punto settato come punto minimo
 */
inline void BoundingBox::setMin(const Pointd& min) {
    this->min = min;
}

/**
 * \~English
 * @brief Modifies the maximum point of the bounding box
 * @param[in] max: point setted as maximum point
 *
 * \~Italian
 * @brief Modifica il punto massimo del bounding box
 * @param[in] max: punto settato come punto massimo
 */
inline void BoundingBox::setMax(const Pointd& max) {
    this->max = max;
}

/**
 * \~English
 * @brief Modifies the minimum point of the bounding box
 * @param[in] x: value setted as \c x component of the minimum point
 * @param[in] y: value setted as \c y component of the minimum point
 * @param[in] z: value setted as \c z component of the minimum point
 *
 * \~Italian
 * @brief Modifica il punto minimo del bounding box
 * @param[in] x: valore settato come componente \c x del punto minimo
 * @param[in] y: valore settato come componente \c y del punto minimo
 * @param[in] z: valore settato come componente \c z del punto minimo
 */
inline void BoundingBox::setMin(double x, double y, double z) {
    min.set(x, y, z);
}

/**
 * \~English
 * @brief Modifies the maximum point of the bounding box
 * @param[in] x: value setted as \c x component of the maximum point
 * @param[in] y: value setted as \c y component of the maximum point
 * @param[in] z: value setted as \c z component of the maximum point
 *
 * \~Italian
 * @brief Modifica il punto massimo del bounding box
 * @param[in] x: valore settato come componente \c x del punto massimo
 * @param[in] y: valore settato come componente \c y del punto massimo
 * @param[in] z: valore settato come componente \c z del punto massimo
 */
inline void BoundingBox::setMax(double x, double y, double z) {
    max.set(x, y, z);
}

/**
 * \~English
 * @brief Modifies the \c x component of the minimum point of the bounding box
 * @param[in] x: value that will be setted as \c x component of the minimum point
 *
 * \~Italian
 * @brief Modifica la componente \c x del punto minimo del bounding box
 * @param[in] x: valore settato come componente \c x del punto minimo
 */
inline void BoundingBox::setMinX(double x) {
    min.setX(x);
}

/**
 * \~English
 * @brief Modifies the \c y component of the minimum point of the bounding box
 * @param[in] y: value that will be setted as \c y component of the minimum point
 *
 * \~Italian
 * @brief Modifica la componente \c y del punto minimo del bounding box
 * @param[in] y: valore settato come componente \c y del punto minimo
 */
inline void BoundingBox::setMinY(double y) {
    min.setY(y);
}

/**
 * \~English
 * @brief Modifies the \c z component of the minimum point of the bounding box
 * @param[in] z: value that will be setted as \c z component of the minimum point
 *
 * \~Italian
 * @brief Modifica la componente \c z del punto minimo del bounding box
 * @param[in] z: valore settato come componente \c z del punto minimo
 */
inline void BoundingBox::setMinZ(double z) {
    min.setZ(z);
}

/**
 * \~English
 * @brief Modifies the \c x component of the maximum point of the bounding box
 * @param[in] x: value that will be setted as \c x component of the maximum point
 *
 * \~Italian
 * @brief Modifica la componente \c x del punto massimo del bounding box
 * @param[in] x: valore settato come componente \c x del punto massimo
 */
inline void BoundingBox::setMaxX(double x) {
    max.setX(x);
}

/**
 * \~English
 * @brief Modifies the \c y component of the maximum point of the bounding box
 * @param[in] y: value that will be setted as \c y component of the maximum point
 *
 * \~Italian
 * @brief Modifica la componente \c y del punto massimo del bounding box
 * @param[in] y: valore settato come componente \c y del punto massimo
 */
inline void BoundingBox::setMaxY(double y) {
    max.setY(y);
}

/**
 * \~English
 * @brief Modifies the \c z component of the maximum point of the bounding box
 * @param[in] z: value that will be setted as \c z component of the maximum point
 *
 * \~Italian
 * @brief Modifica la componente \c z del punto massimo del bounding box
 * @param[in] z: valore settato come componente \c z del punto massimo
 */
inline void BoundingBox::setMaxZ(double z) {
    max.setZ(z);
}

/**
 * \~English
 * @brief Resets the bounding box.
 *
 * Sets the three components of minimum to +std::numeric_limits<double>::max() and the three components of maximum to -std::numeric_limits<double>::max().
 * In this way, it is faster to create a new bounding box (every number is greater than -std::numeric_limits<double>::max() and every number is lower
 * than std::numeric_limits<double>::max()).
 *
 * \~Italian
 * @brief Resetta il bounding box.
 *
 * Setta le tre componenti del punto minimo a std::numeric_limits<double>::max() e del punto massimo a -std::numeric_limits<double>::max().
 * In questo modo, è più veloce creare un nuovo bounding box (ogni numero è maggiore di -std::numeric_limits<double>::max() e ogni numero è minore di
 * std::numeric_limits<double>::max()).
 */
inline void BoundingBox::reset() {
    min = Pointd( std::numeric_limits<double>::max(),  std::numeric_limits<double>::max(),  std::numeric_limits<double>::max());
    max = Pointd(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
}

/**
 * \~English
 * @brief Deserializes (reads) a bounding box from a std::ifstream opened in binary mode
 * @param[in] binaryFile: std::ifstream opened in binary mode, having the cursor on a position where is stored a bounding box
 *
 * \~Italian
 * @brief Deserializza (legge) un bounding box da un std::ifstream aperto in modalità binaria
 * @param[in] binaryFile: std::ifstream aperto in modalità binaria, avente il cursore in una posizione dove è salvato un bounding box
 */
inline void BoundingBox::deserialize(std::ifstream& binaryFile) {
    min.deserialize(binaryFile);
    max.deserialize(binaryFile);
}

#endif // BOUNDING_BOX_H
