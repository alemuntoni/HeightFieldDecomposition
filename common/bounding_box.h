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
        const Pointd& min()        const;
        const Pointd& getMax()     const;
        const Pointd& max()        const;
        const double& getMinX()    const;
        const double& minX()       const;
        const double& getMinY()    const;
        const double& minY()       const;
        const double& getMinZ()    const;
        const double& minZ()       const;
        const double& getMaxX()    const;
        const double& maxX()       const;
        const double& getMaxY()    const;
        const double& maxY()       const;
        const double& getMaxZ()    const;
        const double& maxZ()       const;
        double getLengthX()        const;
        double getLengthY()        const;
        double getLengthZ()        const;

        Pointd  center()    const;
        double diag()       const;
        bool isStrictlyIntern(const Pointd& p)     const;
        bool isStrictlyIntern(double px, double py, double pz)     const;
        bool isIntern(const Pointd& p)     const;
        bool isIntern(double px, double py, double pz)     const;
        bool isEpsilonIntern(const Pointd& p, double epsilon = 1e-6)     const;
        bool isEpsilonIntern(double px, double py, double pz, double epsilon = 1e-6)     const;
        void getExtremes(std::vector<Pointd> &extremes) const;
        std::vector<Pointd> getExtremes() const;

        //Operators
        const double& operator[](unsigned int i)                 const;
        const double& operator()(unsigned int i)                 const;

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;


        void setMin(const Pointd& min);
        Pointd& min();
        void setMax(const Pointd& max);
        Pointd& max();
        void setMin(double x, double y, double z);
        void setMax(double x, double y, double z);
        void setMinX(double x);
        double& minX();
        void setMinY(double y);
        double& minY();
        void setMinZ(double z);
        double& minZ();
        void setMaxX(double x);
        double& maxX();
        void setMaxY(double y);
        double& maxY();
        void setMaxZ(double z);
        double& maxZ();
        void reset();

        //Operators
        double& operator[](unsigned int i);
        double& operator()(unsigned int i);

        // SerializableObject interface
        bool deserialize(std::ifstream& binaryFile);

    protected:

        /**************
        * Attributes *
        **************/

        Pointd minCoord; /**< \~English @brief Minimum point of the bounding box
                         \~Italian @brief Punto minimo del bounding box*/
        Pointd maxCoord; /**< \~English @brief Maximum point of the bounding box
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
inline BoundingBox::BoundingBox(const Pointd& min, const Pointd& max) : minCoord(min), maxCoord(max) {
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
    return minCoord;
}

inline const Pointd& BoundingBox::min() const {
    return minCoord;
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
    return maxCoord;
}

inline const Pointd& BoundingBox::max() const {
    return maxCoord;
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
    return minCoord.x();
}

inline const double& BoundingBox::minX() const {
    return minCoord.x();
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
    return minCoord.y();
}

inline const double& BoundingBox::minY() const {
    return minCoord.y();
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
    return minCoord.z();
}

inline const double& BoundingBox::minZ() const {
    return minCoord.z();
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
    return maxCoord.x();
}

inline const double& BoundingBox::maxX() const {
    return maxCoord.x();
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
    return maxCoord.y();
}

inline const double& BoundingBox::maxY() const {
    return maxCoord.y();
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
    return maxCoord.z();
}

inline const double& BoundingBox::maxZ() const {
    return maxCoord.z();
}

inline double BoundingBox::getLengthX() const {
    return maxCoord.x() - minCoord.x();
}

inline double BoundingBox::getLengthY() const {
    return maxCoord.y() - minCoord.y();
}

inline double BoundingBox::getLengthZ() const {
    return maxCoord.z() - minCoord.z();
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
    return (minCoord + maxCoord) * 0.5;
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
    return (minCoord - maxCoord).getLength();
}

inline bool BoundingBox::isStrictlyIntern(const Pointd &p) const {
    return (p.x() > minCoord.x() && p.y() > minCoord.y() && p.z() > minCoord.z() &&
            p.x() < maxCoord.x() && p.y() < maxCoord.y() && p.z() < maxCoord.z());
}

inline bool BoundingBox::isStrictlyIntern(double px, double py, double pz) const {
    return (px > minCoord.x() && py > minCoord.y() && pz > minCoord.z() &&
            px < maxCoord.x() && py < maxCoord.y() && pz < maxCoord.z());
}

inline bool BoundingBox::isIntern(const Pointd &p) const {
    return (p.x() >= minCoord.x() && p.y() >= minCoord.y() && p.z() >= minCoord.z() &&
            p.x() <= maxCoord.x() && p.y() <= maxCoord.y() && p.z() <= maxCoord.z());
}

inline bool BoundingBox::isIntern(double px, double py, double pz) const {
    return (px >= minCoord.x() && py >= minCoord.y() && pz >= minCoord.z() &&
            px <= maxCoord.x() && py <= maxCoord.y() && pz <= maxCoord.z());
}

inline bool BoundingBox::isEpsilonIntern(const Pointd& p, double epsilon) const {
    return (p.x() >= minCoord.x()-epsilon && p.y() >= minCoord.y()-epsilon && p.z() >= minCoord.z()-epsilon &&
            p.x() <= maxCoord.x()+epsilon && p.y() <= maxCoord.y()+epsilon && p.z() <= maxCoord.z()+epsilon);
}

inline bool BoundingBox::isEpsilonIntern(double px, double py, double pz, double epsilon) const {
    return (px >= minCoord.x()-epsilon && py >= minCoord.y()-epsilon && pz >= minCoord.z()-epsilon &&
            px <= maxCoord.x()+epsilon && py <= maxCoord.y()+epsilon && pz <= maxCoord.z()+epsilon);
}

inline void BoundingBox::getExtremes(std::vector<Pointd>& extremes) const {
    extremes.resize(8);
    extremes[0] = minCoord;
    extremes[1].set(maxCoord.x(), minCoord.y(), minCoord.z());
    extremes[2].set(maxCoord.x(), minCoord.y(), maxCoord.z());
    extremes[3].set(minCoord.x(), minCoord.y(), maxCoord.z());
    extremes[4].set(minCoord.x(), maxCoord.y(), minCoord.z());
    extremes[5].set(maxCoord.x(), maxCoord.y(), minCoord.z());
    extremes[6] = maxCoord;
    extremes[7].set(minCoord.x(), maxCoord.y(), maxCoord.z());
}

inline std::vector<Pointd> BoundingBox::getExtremes() const {
    std::vector<Pointd> extremes;
    getExtremes(extremes);
    return extremes;
}

inline const double& BoundingBox::operator[](unsigned int i) const{
    assert(i < 6);
    switch (i%6){
        case 0: return minCoord.x();
        case 1: return minCoord.y();
        case 2: return minCoord.z();
        case 3: return maxCoord.x();
        case 4: return maxCoord.y();
        case 5: return maxCoord.z();
    }
    return minCoord.x();
}

inline const double& BoundingBox::operator()(unsigned int i) const {
    assert(i < 6);
    switch (i%6){
        case 0: return minCoord.x();
        case 1: return minCoord.y();
        case 2: return minCoord.z();
        case 3: return maxCoord.x();
        case 4: return maxCoord.y();
        case 5: return maxCoord.z();
    }
    return minCoord.x();
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
    minCoord.serialize(binaryFile);
    maxCoord.serialize(binaryFile);
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
    this->minCoord = min;
}

inline Pointd&BoundingBox::min() {
    return minCoord;
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
    this->maxCoord = max;
}

inline Pointd&BoundingBox::max() {
    return maxCoord;
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
    minCoord.set(x, y, z);
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
    maxCoord.set(x, y, z);
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
    minCoord.setX(x);
}

inline double& BoundingBox::minX() {
    return minCoord.x();
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
    minCoord.setY(y);
}

inline double& BoundingBox::minY() {
    return minCoord.y();
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
    minCoord.setZ(z);
}

inline double& BoundingBox::minZ() {
    return minCoord.z();
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
    maxCoord.setX(x);
}

inline double&BoundingBox::maxX() {
    return maxCoord.x();
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
    maxCoord.setY(y);
}

inline double&BoundingBox::maxY() {
    return maxCoord.y();
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
    maxCoord.setZ(z);
}

inline double& BoundingBox::maxZ() {
    return maxCoord.z();
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
    minCoord = Pointd( std::numeric_limits<double>::max(),  std::numeric_limits<double>::max(),  std::numeric_limits<double>::max());
    maxCoord = Pointd(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
}

inline double& BoundingBox::operator[](unsigned int i) {
    assert(i < 6);
    switch (i%6){
        case 0: return minCoord.x();
        case 1: return minCoord.y();
        case 2: return minCoord.z();
        case 3: return maxCoord.x();
        case 4: return maxCoord.y();
        case 5: return maxCoord.z();
    }
    return minCoord.x();
}

inline double& BoundingBox::operator()(unsigned int i) {
    assert(i < 6);
    switch (i%6){
        case 0: return minCoord.x();
        case 1: return minCoord.y();
        case 2: return minCoord.z();
        case 3: return maxCoord.x();
        case 4: return maxCoord.y();
        case 5: return maxCoord.z();
    }
    return minCoord.x();
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
inline bool BoundingBox::deserialize(std::ifstream& binaryFile) {
    Pointd tmp;
    if (tmp.deserialize(binaryFile) && maxCoord.deserialize(binaryFile)){
        minCoord = std::move(tmp);
        return true;
    }
    else
        return false;

}

#endif // BOUNDING_BOX_H
