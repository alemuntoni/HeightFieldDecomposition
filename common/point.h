/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef DCEL_POINT_H
#define DCEL_POINT_H

#include <sstream>
#include <assert.h>
#include <string>
#include <iostream>
#include <math.h>

#include "serialize.h"

/**
 * \~English
 * @class Point
 * @brief The Point class models a point or a vector on a 3D space.
 *
 * Represents a 3D point or vector, with the precision given by the template type T.
 * In particular, it is possible to have vectros/points with integer, float or double precision.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 *
 * \~Italian
 * @class Point
 * @brief La classe Point modella un punto o un vettore nell spazio.
 *
 * Rappresenta un punto o vettore in 3 dimensioni, con precisione dati variabile definita dal tipo T.
 * In particolare, è possibile avere punti/vettori con precisione intera, float o double.
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
template <class T> class Point : SerializableObject {

    public:

        /****************
         * Constructors *
         ****************/

        Point(T xCoord = 0.0, T yCoord = 0.0, T zCoord = 0.0);
        virtual ~Point(void){}

        /*************************
        * Public Inline Methods *
        *************************/

        const T& x()                                        const;
        const T& y()                                        const;
        const T& z()                                        const;
        double dist(const Point<T>& otherPoint)             const;
        T dot(const Point<T>& otherVector)                  const;
        Point<T> cross(const Point<T>& otherVector)         const;
        double getLength()                                  const;
        double getLengthSquared()                           const;
        Point<T> min(const Point<T>& otherPoint)            const;
        Point<T> max(const Point<T>& otherPoint)            const;
        // SerializableObject interface
        void serialize(std::ofstream &myfile)               const;

        // Operators
        bool operator == (const Point<T>& otherPoint)       const;
        bool operator != (const Point<T>& otherPoint)       const;
        bool operator < (const Point<T>& otherPoint)        const;
        Point<T> operator - ()                              const;
        Point<T> operator + (const T& scalar)               const;
        Point<T> operator + (const Point<T>& otherPoint)    const;
        Point<T> operator - (const T& scalar)               const;
        Point<T> operator - (const Point<T>& otherPoint)    const;
        Point<T> operator * (const T& scalar)               const;
        Point<T> operator * (const Point<T>& otherPoint)    const;
        Point<T> operator / (const T& scalar )              const;
        Point<T> operator / (const Point<T>& otherPoint)    const;


        T& x();
        T& y();
        T& z();
        void setX(const T& x);
        void setY(const T& y);
        void setZ(const T& z);
        void set(const T& x, const T& y, const T& z);
        double normalize();
        #ifdef COMMON_WITH_EIGEN
        void rotate(const Eigen::Matrix3d &matrix, const Point<T>& centroid = Point<T>());
        #endif //COMMON_WITH_EIGEN
        void rotate(double matrix[3][3], const Point<T>& centroid = Point<T>());

        // SerializableObject interface
        void deserialize(std::ifstream &myfile);

        // Operators
        Point<T> operator += (const Point<T>& otherPoint);
        Point<T> operator -= (const Point<T>& otherPoint);
        Point<T> operator *= (const T& scalar);
        Point<T> operator *= (const Point<T>& otherPoint);
        Point<T> operator /= (const T& scalar );
        Point<T> operator /= (const Point<T>& otherPoint);

        /*****************
        * Public Methods *
        ******************/

        std::string toString() const;

    protected:

        /**************
        * Attributes *
        **************/

        T xCoord; /**< \~English @brief The \c x component of the point/vector
                       \~Italian @brief La componente \c x del punto/vettore */
        T yCoord; /**< \~English @brief The \c y component of the point/vector
                       \~Italian @brief La componente \c y del punto/vettore */
        T zCoord; /**< \~English @brief The \c z component of the point/vector
                       \~Italian @brief La componente \c z del punto/vettore */

};

/****************
* Other Methods *
*****************/

template <class T>
Point<T> operator * (const T& scalar, const Point<T>& point);

template <class T>
Point<T> mul(const T m[][3], const Point<T>& point);

#ifdef COMMON_WITH_EIGEN
template <class T>
Point<T> mul(const Eigen::Matrix3d &m, const Point<T>& point);
#endif //COMMON_WITH_EIGEN

template <class T>
std::ostream& operator<< (std::ostream& inputStream, const Point<T>& p);

/**************
* Other Types *
***************/

typedef Point<float>  Pointf; /**< \~English @brief Point composed of float components
                                   \~Italian @brief Point composto da componenti float */
typedef Point<double> Pointd; /**< \~English @brief Point composed of double components
                                   \~Italian @brief Point composto da componenti double */
typedef Point<int>    Pointi; /**< \~English @brief Point composed of integer components
                                   \~Italian @brief Point composto da componenti int */
typedef Point<double> Vec3;   /**< \~English @brief Point composed of double components, sinctactic sugar for discriminate points from vectors
                                   \~Italian @brief Point composto da componenti double, zucchero sintattico per distinguere punti da vettori */
template<typename T>
using Point3D = Point<T>;

/****************
 * Constructors *
 ****************/

/**
 * \~English
 * @brief Constructor, initializes the point with the input values
 * @param[in] x: value of \c x component, default 0
 * @param[in] y: value of \c y component, default 0
 * @param[in] z: value of \c z component, default 0
 *
 * \~Italian
 * @brief Costruttore, inizializza il punto con i valori in input
 * @param[in] x: valore componente \c x, default 0
 * @param[in] y: valore componente \c y, default 0
 * @param[in] z: valore componente \c z, default 0
 */
template <class T>
inline Point<T>::Point(T x, T y, T z) : xCoord(x), yCoord(y), zCoord(z) {
}

/*************************
* Public Inline Methods *
*************************/

/**
 * \~English
 * @brief Returns the \c x component of the point/vector
 * @return \c x component
 *
 * \~Italian
 * @brief Restituisce la componente \c x del punto/vettore
 * @return La componente \c x
 */
template <class T>
inline const T& Point<T>::x() const {
    return this->xCoord;
}

/**
 * \~English
 * @brief Returns the \c y component of the point/vector
 * @return \c y component
 *
 * \~Italian
 * @brief Restituisce la componente \c y del punto/vettore
 * @return La componente \c y
 */
template <class T>
inline const T& Point<T>::y() const {
    return this->yCoord;
}

/**
 * \~English
 * @brief Returns the \c z component of the point/vector
 * @return \c z component
 *
 * \~Italian
 * @brief Restituisce la componente \c z del punto/vettore
 * @return La componente \c z
 */
template <class T>
inline const T& Point<T>::z() const {
    return this->zCoord;
}

/**
 * \~English
 * @brief Function that calculates the euclidean distance between two points
 * @param[in] otherPoint: point on which is calculated the distance
 * @return The distance between the point and \c otherPoint
 *
 * \~Italian
 * @brief Funzione per il calcolo della distanza euclidea tra due punti
 * @param[in] otherPoint: punto su cui viene calcolata la distanza
 * @return La distanza euclidea tra this e \c otherPoint
 */
template <class T>
inline double Point<T>::dist(const Point<T>& otherPoint) const {
    return sqrt ( pow((xCoord - otherPoint.xCoord), 2) +
                  pow((yCoord - otherPoint.yCoord), 2) +
                  pow((zCoord - otherPoint.zCoord), 2) );
}

/**
 * \~English
 * @brief Function that calculates the dot product between two vectors
 * @param[in] otherVector: vector on which is calculated the dot product
 * @return The dot product between this and \c otherVector
 *
 * \~Italian
 * @brief Funzione per il calcolo del prodotto scalare tra due vettori
 * @param[in] otherVector: vettore con cui viene calcolato il prodotto scalare
 * @return Il prodotto scalare tra this e \c otherVector
 */
template <class T>
inline T Point<T>::dot(const Point<T>& otherVector) const {
    return xCoord * otherVector.xCoord +
           yCoord * otherVector.yCoord +
           zCoord * otherVector.zCoord;
}

/**
 * \~Italian
 * @brief Funzione per il calcolo del prodotto vettoriale tra due vettori
 * @param[in] otherVector: vettore con cui viene calcolato il prodotto vettoriale
 * @return Il prodotto vettoriale tra this e otherVector
 */
template <class T>
inline Point<T> Point<T>::cross(const Point<T>& otherVector) const {
    return std::move(Point<T>(yCoord * otherVector.zCoord - zCoord * otherVector.yCoord,
                 zCoord * otherVector.xCoord - xCoord * otherVector.zCoord,
                 xCoord * otherVector.yCoord - yCoord * otherVector.xCoord));
}

/**
 * \~Italian
 * @brief Funzione per il calcolo della lunghezza di un vettore
 * @return La lunghezza del vettore this
 */
template <class T>
inline double Point<T>::getLength() const {
    return sqrt( xCoord*xCoord + yCoord*yCoord + zCoord*zCoord );
}

/**
 * \~Italian
 * @brief Operatore per il calcolo della lunghezza al quadrato di un vettore
 * @return La lunghezza al quadrato del vettore this
 */
template <class T>
inline double Point<T>::getLengthSquared() const {
    return xCoord * xCoord + yCoord * yCoord + zCoord * zCoord;
}

/**
 * \~Italian
 * @brief Funzione di minimo tra punti/vettori.
 *
 * Ogni componente del punto/vettore restituito sarà uguale alla corrispondente componente minore tra il punto/vettore
 * this e otherPoint.
 *
 * @param[in] otherPoint: punto/vettore con cui viene calcolata la funzione di minimo
 *
 * @return Il punto/vettore dei minimi
 */
template <class T>
inline Point<T> Point<T>::min(const Point<T>& otherPoint) const {
    return std::move(Point<T>(std::min(x(), otherPoint.x()),
                    std::min(y(), otherPoint.y()),
                    std::min(z(), otherPoint.z())));
}

/**
 * \~Italian
 * @brief Funzione di massimo tra punti/vettori.
 *
 * Ogni componente del punto/vettore restituito sarà uguale alla corrispondente componente maggiore tra il punto/vettore
 * this e otherPoint.
 *
 * @param[in] otherPoint: punto/vettore con cui viene calcolata la funzione di massimo
 *
 * @return Il punto/vettore dei massimi
 */
template <class T>
inline Point<T> Point<T>::max(const Point<T>& otherPoint) const {
    return std::move(Point<T>(std::max(x(), otherPoint.x()),
                    std::max(y(), otherPoint.y()),
                    std::max(z(), otherPoint.z())));
}

/**
 * \~Italian
 * @brief Metodo per serializzare in un ofstream un Point
 * @param[in] myfile: l'ofstream (file) nel quale verrà serializzato il punto
 */
template <class T>
inline void Point<T>::serialize(std::ofstream &myfile) const{
    Serializer::serialize(xCoord, myfile);
    Serializer::serialize(yCoord, myfile);
    Serializer::serialize(zCoord, myfile);
}

/**
 * \~Italian
 * @brief Operatore di uguaglianza tra punti/vettori.
 *
 * Due punti/vettori sono considerati uguali se tutte e tre le loro componenti sono uguali.
 *
 * @param[in] otherPoint: punto/vettore con cui viene verificata l'uguaglianza
 * @return True se il punto e otherPoint sono uguali, false altrimenti
 */
template <class T>
inline bool Point<T>::operator == (const Point<T>& otherPoint) const {
    if ( otherPoint.xCoord != xCoord )	return false;
    if ( otherPoint.yCoord != yCoord )	return false;
    if ( otherPoint.zCoord != zCoord )	return false;
    return true;
}

/**
 * \~Italian
 * @brief Operatore di disuguaglianza tra punti/vettori.
 *
 * Due punti/vettori sono considerati diversi se almeno una delle loro componenti è diversa.
 *
 * @param[in] otherPoint: punto/vettore con cui viene verificata la disuguaglianza
 * @return True se il punto e otherPoint sono diversi, false altrimenti
 */
template <class T>
inline bool Point<T>::operator != (const Point<T>& otherPoint) const {
    if ( otherPoint.xCoord != xCoord )	return true;
    if ( otherPoint.yCoord != yCoord )	return true;
    if ( otherPoint.zCoord != zCoord )	return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore di minore tra punti/vettori.
 *
 * In questo contesto, il punto/vettore è minore di otherPoint se la sua componente x
 * è minore di quella di otherPoint; in caso di uguaglianza si verifica la componente y
 * e in caso di ultieriore uguaglianza la componente z.
 *
 * @param[in] otherPoint: altro punto/vettore
 * @return True se il punto/vettore this è minore di otherPoint, false altrimenti
 */
template <class T>
inline bool Point<T>::operator < (const Point<T>& otherPoint) const {
    if (this->xCoord < otherPoint.xCoord) return true;
    if (this->xCoord > otherPoint.xCoord) return false;
    if (this->yCoord < otherPoint.yCoord) return true;
    if (this->yCoord > otherPoint.yCoord) return false;
    if (this->zCoord < otherPoint.zCoord) return true;
    return false;
}

/**
 * \~Italian
 * @brief Operatore prefisso di negazione, restituisce il punto/vettore negato
 * @return Il punto/vettore negato
 */
template <class T>
inline Point<T> Point<T>::operator - () const {
    return std::move(Point<T>(-xCoord, -yCoord, -zCoord));
}

template <class T>
inline Point<T> Point<T>::operator +(const T& scalar) const {
    return std::move(Point<T>(xCoord + scalar,
                    yCoord + scalar,
                    zCoord + scalar));
}

/**
 * \~Italian
 * @brief Operatore di somma tra punti/vettori
 * @param[in] otherPoint: punto/vettore con cui verrà sommato il punto/vettore this
 * @return Il punto/vettore risultato della somma, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator + (const Point<T>& otherPoint) const {
    return std::move(Point<T>(xCoord + otherPoint.xCoord,
                    yCoord + otherPoint.yCoord,
                    zCoord + otherPoint.zCoord));
}

template <class T>
inline Point<T> Point<T>::operator -(const T& scalar) const {
    return std::move(Point<T>(xCoord - scalar,
                    yCoord - scalar,
                    zCoord - scalar));
}

/**
 * \~Italian
 * @brief Operatore di sottrazione tra punti/vettori
 * @param[in] otherPoint: punto/vettore che verrà sottratto al punto/vettore this
 * @return Il punto/vettore risultato della differenza, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator - (const Point<T>& otherPoint) const {
    return std::move(Point<T>(xCoord - otherPoint.xCoord,
                    yCoord - otherPoint.yCoord,
                    zCoord - otherPoint.zCoord));
}

/**
 * \~Italian
 * @brief Operatore di prodotto scalare tra un punto/vettore e uno scalare
 * @param[in] scalar: scalare con cui verrà eseguito il prodotto scalare
 * @return Il punto/vettore risultato del prodotto scalare tra tra il punto/vettore this e scalar
 */
template <class T>
inline Point<T> Point<T>::operator * (const T& scalar) const {
    return std::move(Point<T>(xCoord * scalar, yCoord * scalar, zCoord * scalar));
}

/**
 * \~Italian
 * @brief Operatore di prodotto, componente per componente, tra punti/vettori
 * @param[in] otherPoint: punto/vettore con cui verrà eseguito il prodotto
 * @return Il punto/vettore risultato del prodotto, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator * (const Point<T>& otherPoint) const {
    return std::move(Point<T>(xCoord * otherPoint.xCoord, yCoord * otherPoint.yCoord, zCoord * otherPoint.zCoord));
}

/**
 * \~Italian
 * @brief Operatore di quoziente scalare tra un punto/vettore e uno scalare
 * @param[in] scalar: scalare con cui verrà eseguito il quoziente scalare
 * @return Il punto/vettore risultato del quoziente scalare tra il punto/vettore this e scalar
 */
template <class T>
inline Point<T> Point<T>::operator / (const T& scalar) const {
    return std::move(Point<T>(xCoord / scalar, yCoord / scalar, zCoord / scalar));
}

/**
 * \~Italian
 * @brief Operatore di quoziente, componente per componente, tra punti/vettori
 * @param[in] otherPoint: punto/vettore con cui verrà eseguito il quoziente
 * @return Il punto/vettore risultato del quoziente, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator / (const Point<T>& otherPoint) const {
    return std::move(Point<T>(xCoord / otherPoint.xCoord, yCoord / otherPoint.yCoord, zCoord / otherPoint.zCoord));
}

/**
 * \~English
 * @brief Returns the \c x component of the point/vector
 * @return \c x component
 *
 * \~Italian
 * @brief Restituisce la componente \c x del punto/vettore
 * @return La componente \c x
 */
template <class T>
inline T& Point<T>::x() {
    return this->xCoord;
}

/**
 * \~English
 * @brief Returns the \c y component of the point/vector
 * @return \c y component
 *
 * \~Italian
 * @brief Restituisce la componente \c y del punto/vettore
 * @return La componente \c y
 */
template <class T>
inline T& Point<T>::y() {
    return this->yCoord;
}

/**
 * \~English
 * @brief Returns the \c z component of the point/vector
 * @return \c z component
 *
 * \~Italian
 * @brief Restituisce la componente \c z del punto/vettore
 * @return La componente \c z
 */
template <class T>
inline T& Point<T>::z() {
    return this->zCoord;
}

/**
 * \~Italian
 * @brief Modifica la componente x del punto/vettore this
 * @param[in] x: valore settato come componente x
 */
template <class T>
inline void Point<T>::setX(const T& x) {
    xCoord = x;
}

/**
 * \~Italian
 * @brief Modifica la componente y del punto/vettore this
 * @param[in] y: valore settato come componente y
 */
template <class T>
inline void Point<T>::setY(const T& y) {
    yCoord = y;
}

/**
 * \~Italian
 * @brief Modifica la componente z del punto/vettore this
 * @param[in] z: valore settato come componente z
 */
template <class T>
inline void Point<T>::setZ(const T& z) {
    zCoord = z;
}

/**
 * \~Italian
 * @brief Modifica le componenti del punto/vettore this
 * @param[in] x: valore settato come componente x
 * @param[in] y: valore settato come componente y
 * @param[in] z: valore settato come componente z
 */
template <class T>
inline void Point<T>::set(const T& x, const T& y, const T& z) {
    xCoord = x;
    yCoord = y;
    zCoord = z;
}

/**
 * \~Italian
 * @brief Funzione di normalizzazione di un vettore, in modo tale che la sua lunghezza sia pari a 1
 * @return La lunghezza precedente del vettore prima di essere normalizzato
 */
template <class T>
inline double Point<T>::normalize() {
    double len = getLength();
    xCoord /= len;
    yCoord /= len;
    zCoord /= len;
    return len;
}

#ifdef COMMON_WITH_EIGEN
template <class T>
void Point<T>::rotate(const Eigen::Matrix3d& matrix, const Point<T>& centroid) {
    *this -= centroid;
    *this = mul(matrix, *this);
    *this += centroid;
}
#endif //COMMON_WITH_EIGEN

/**
 * \~Italian
 * @brief Applica una matrice di rotazione 3x3 ad un punto/vettore
 * @param[in] m: matrice di rotazione 3x3
 * @param[in] centroid: punto centroide della rotazione, di default (0,0,0)
 */
template <class T>
inline void Point<T>::rotate(double matrix[3][3], const Point<T>& centroid) {
    *this -= centroid;
    *this = mul(matrix, *this);
    *this += centroid;
}

/**
 * \~Italian
 * @brief Metodo per deserializzare da un ifstream un Point
 * @param[in] myfile: l'ifstream (file) dal quale verrà deserializzato il punto
 */
template <class T>
inline void Point<T>::deserialize(std::ifstream& myfile) {
    Serializer::deserialize(xCoord, myfile);
    Serializer::deserialize(yCoord, myfile);
    Serializer::deserialize(zCoord, myfile);
}

/**
 * \~Italian
 * @brief Operatore di somma e assegnamento tra punti/vettori.
 *
 * Il risultato della somma è assegnato al punto/vettore this.
 *
 * @param[in] otherPoint: punto/vettore con cui verrà sommato il punto/vettore this
 * @return Il punto/vettore risultato della somma, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator += (const Point<T>& otherPoint) {
    xCoord += otherPoint.xCoord;
    yCoord += otherPoint.yCoord;
    zCoord += otherPoint.zCoord;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di sottrazione e assegnamento tra punti/vettori.
 *
 * Il risultato della differenza è assegnato al punto/vettore this.
 *
 * @param[in] otherPoint: punto/vettore che verrà sottratto al punto/vettore this
 * @return Il punto/vettore risultato della differenza, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator -= (const Point<T>& otherPoint) {
    xCoord -= otherPoint.xCoord;
    yCoord -= otherPoint.yCoord;
    zCoord -= otherPoint.zCoord;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di prodotto scalare e assegnamento tra un punto/vettore e uno scalare.
 *
 * Il risultato del prodotto scalare è assegnato al punto/vettore this.
 *
 * @param[in] scalar: scalare con cui verrà eseguito il prodotto scalare
 * @return Il punto/vettore risultato del prodotto scalare tra tra il punto/vettore this e scalar
 */
template <class T>
inline Point<T> Point<T>::operator *= (const T& scalar) {
    xCoord *= scalar;
    yCoord *= scalar;
    zCoord *= scalar;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di prodotto, componente per componente, e assegnamento tra punti/vettori.
 *
 * Il risultato del prodotto è assegnato al punto/vettore this.
 *
 * @param[in] otherPoint: punto/vettore con cui verrà eseguito il prodotto
 * @return Il punto/vettore risultato del prodotto, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator *= (const Point<T>& otherPoint) {
    xCoord *= otherPoint.xCoord;
    yCoord *= otherPoint.yCoord;
    zCoord *= otherPoint.zCoord;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di quoziente scalare e assegnamento tra un punto/vettore e uno scalare.
 *
 * Il risultato del quoziente scalare è assegnato al punto/vettore this.
 *
 * @param[in] scalar: scalare con cui verrà eseguito il quoziente scalare
 * @return Il punto/vettore risultato del quoziente scalare tra il punto/vettore this e scalar
 */
template <class T>
inline Point<T> Point<T>::operator /= (const T& scalar) {
    xCoord /= scalar;
    yCoord /= scalar;
    zCoord /= scalar;
    return *this;
}

/**
 * \~Italian
 * @brief Operatore di quoziente, componente per componente, e assegnamento tra punti/vettori.
 *
 * Il risultato del quoziente è assegnato al punto/vettore this.
 *
 * @param[in] otherPoint: punto/vettore con cui verrà eseguito il quoziente
 * @return Il punto/vettore risultato del quoziente, componente per componente, tra i punti/vettori this e otherPoint
 */
template <class T>
inline Point<T> Point<T>::operator /= (const Point<T>& otherPoint) {
    xCoord /= otherPoint.xCoord;
    yCoord /= otherPoint.yCoord;
    zCoord /= otherPoint.zCoord;
    return *this;
}

/*****************
* Public Methods *
******************/

/**
 * \~Italian
 * @brief Funzione toString di un punto/vettore
 * @return Una stringa rappresentativa del punto/vettore this
 */
template <class T>
std::string Point<T>::toString() const {
    std::stringstream ss;
    ss << "[" << xCoord << ", " << yCoord << ", " << zCoord << "]";
    std::string s1 = ss.str();
    return std::move(s1);
}

/****************
* Other Methods *
*****************/
/**
 * \~Italian
 * @brief Operatore di prodotto scalare tra un punto/vettore e uno scalare
 * @param[in] scalar: scalare con cui verrà eseguito il prodotto scalare
 * @param[in] point: punto/vettore con cui verrà eseguito il prodotto scalare
 * @return Il punto/vettore risultato del prodotto scalare tra tra point e scalar
 */
template <class T>
inline Point<T> operator * (const T& scalar, const Point<T>& point) {
    return std::move(Point<T>(point.xCoord * scalar,
                    point.yCoord * scalar,
                    point.zCoord * scalar));
}

template <class T>
inline Point<T> mul(const T m[][3], const Point<T>& point) {
    Point<T> tmp = point;
    tmp.setX(m[0][0]*point.x() + m[0][1]*point.y() + m[0][2]*point.z());
    tmp.setY(m[1][0]*point.x() + m[1][1]*point.y() + m[1][2]*point.z());
    tmp.setZ(m[2][0]*point.x() + m[2][1]*point.y() + m[2][2]*point.z());
    return std::move(tmp);
}

#ifdef COMMON_WITH_EIGEN
template <class T>
inline Point<T> mul(const Eigen::Matrix3d &m, const Point<T>& point) {
    Point<T> tmp = point;
    tmp.setX(m(0,0)*point.x() + m(0,1)*point.y() + m(0,2)*point.z());
    tmp.setY(m(1,0)*point.x() + m(1,1)*point.y() + m(1,2)*point.z());
    tmp.setZ(m(2,0)*point.x() + m(2,1)*point.y() + m(2,2)*point.z());
    return std::move(tmp);
}
#endif //COMMON_WITH_EIGEN

/**
 * \~Italian
 * @brief Operatore di stram sul punto/vettore
 * @param[in] input_stream: stream di input
 * @return Lo stream di input a cui è stato accodato lo stream del punto/vettore
 */
template <class T>
std::ostream& operator<<(std::ostream& inputStream, const Point<T>& p) {
    inputStream << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]";
    return inputStream;
}

#endif // DCEL_POINT_H
