#ifndef POINT2D_H
#define POINT2D_H

#include <sstream>
#include <assert.h>
#include <QDebug>
#include <string>
#include <iostream>
#include <math.h>

class Point2D
{
    public:
        Point2D(double x = 0.0, double y = 0.0);

        double x() const;
        double y() const;
        double dist(const Point2D &otherPoint) const;
        double dot(const Point2D &otherVector) const;
        double perpendicularDot(const Point2D &otherVector) const;
        double getLength() const;
        double getLengthSquared() const;
        Point2D min(const Point2D &otherPoint) const;
        Point2D max(const Point2D &otherPoint) const;
        bool operator == (const Point2D &otherPoint)   const;
        bool operator != (const Point2D &otherPoint)   const;
        bool operator < (const Point2D& otherPoint)    const;
        bool operator > (const Point2D& otherPoint)    const;
        bool operator <= (const Point2D& otherPoint)   const;
        bool operator >= (const Point2D& otherPoint)   const;
        Point2D operator - ()                          const;
        Point2D operator + (const Point2D& otherPoint) const;
        Point2D operator - (const Point2D& otherPoint) const;
        Point2D operator * (const double& scalar)      const;
        Point2D operator * (const Point2D& otherPoint) const;
        Point2D operator / (const double& scalar )     const;
        Point2D operator / (const Point2D& otherPoint) const;

        void setXCoord(const double& x);
        void setYCoord(const double& y);
        void set(const double& x, const double& y);
        double normalize();
        void rotate(double matrix[2][2], Point2D centroid = Point2D());
        Point2D operator += (const Point2D& otherPoint);
        Point2D operator -= (const Point2D& otherPoint);
        Point2D operator *= (const double& scalar);
        Point2D operator *= (const Point2D& otherPoint);
        Point2D operator /= (const double& scalar );
        Point2D operator /= (const Point2D& otherPoint);

    private:
        double xCoord, yCoord;
        void rot(double matrix[2][2]);
};

inline void Point2D::rot(double matrix[2][2]){
    Point2D p;
    p.setXCoord(matrix[0][0]*xCoord + matrix[0][1]*yCoord);
    p.setYCoord(matrix[1][0]*xCoord + matrix[1][1]*yCoord);
    xCoord = p.x();
    yCoord = p.y();
}

/****************
* Other Methods *
*****************/

Point2D operator * (const double& scalar, const Point2D& point);

Point2D normalOfSegment(const Point2D& p1, const Point2D& p2);

inline Point2D::Point2D(double x, double y): xCoord(x), yCoord(y) {
}

inline double Point2D::x() const {
    return xCoord;
}

inline double Point2D::y() const {
    return yCoord;
}

inline double Point2D::dist(const Point2D &otherPoint) const{
    return sqrt ( pow((xCoord - otherPoint.xCoord), 2) +
                  pow((yCoord - otherPoint.yCoord), 2) );
}

inline double Point2D::dot(const Point2D &otherVector) const{
    return xCoord * otherVector.xCoord +
           yCoord * otherVector.yCoord;
}

inline double Point2D::perpendicularDot(const Point2D& otherVector) const {
    return xCoord * otherVector.yCoord -
           yCoord * otherVector.xCoord;
}

inline double Point2D::getLength() const{
    return sqrt( xCoord*xCoord + yCoord*yCoord );
}

inline double Point2D::getLengthSquared() const{
    return xCoord*xCoord + yCoord*yCoord;
}

inline Point2D Point2D::min(const Point2D &otherPoint) const{
    return Point2D(std::min(x(), otherPoint.xCoord),
                   std::min(y(), otherPoint.yCoord));
}

inline Point2D Point2D::max(const Point2D &otherPoint) const{
    return Point2D(std::max(x(), otherPoint.xCoord),
                   std::max(y(), otherPoint.yCoord));
}

inline bool Point2D::operator == (const Point2D& otherPoint) const {
    if ( otherPoint.xCoord != xCoord )	return false;
    if ( otherPoint.yCoord != yCoord )	return false;
    return true;
}

inline bool Point2D::operator != (const Point2D& otherPoint) const {
    if ( otherPoint.xCoord != xCoord )	return true;
    if ( otherPoint.yCoord != yCoord )	return true;
    return false;
}

inline bool Point2D::operator < (const Point2D& otherPoint) const {
    if (this->xCoord < otherPoint.xCoord) return true;
    if (this->xCoord > otherPoint.xCoord) return false;
    if (this->yCoord < otherPoint.yCoord) return true;
    return false;
}

inline bool Point2D::operator >(const Point2D& otherPoint) const {
    if (this->xCoord < otherPoint.xCoord) return false;
    if (this->xCoord > otherPoint.xCoord) return true;
    if (this->yCoord > otherPoint.yCoord) return true;
    return false;
}

inline bool Point2D::operator <=(const Point2D& otherPoint) const {
    return *this == otherPoint || *this < otherPoint;
}

inline bool Point2D::operator >=(const Point2D& otherPoint) const {
    return *this == otherPoint || *this > otherPoint;
}

inline Point2D Point2D::operator - () const {
    return Point2D(-xCoord, -yCoord);
}

inline Point2D Point2D::operator + (const Point2D& otherPoint) const {
    return Point2D(xCoord + otherPoint.xCoord,
                    yCoord + otherPoint.yCoord);
}

inline Point2D Point2D::operator - (const Point2D& otherPoint) const {
    return Point2D(xCoord - otherPoint.xCoord,
                    yCoord - otherPoint.yCoord);
}

inline Point2D Point2D::operator * (const double& scalar) const {
    return Point2D(xCoord * scalar, yCoord * scalar);
}

inline Point2D Point2D::operator * (const Point2D& otherPoint) const {
    return Point2D(xCoord * otherPoint.xCoord, yCoord * otherPoint.yCoord);
}

inline Point2D Point2D::operator / (const double& scalar) const {
    return Point2D(xCoord / scalar, yCoord / scalar);
}

inline Point2D Point2D::operator / (const Point2D& otherPoint) const {
    return Point2D(xCoord / otherPoint.xCoord, yCoord / otherPoint.yCoord);
}

inline void Point2D::setXCoord(const double& x) {
    xCoord = x;
}

inline void Point2D::setYCoord(const double& y) {
    yCoord = y;
}

inline void Point2D::set(const double& x, const double& y) {
    xCoord = x;
    yCoord = y;
}

inline double Point2D::normalize() {
    double len = getLength();
    xCoord /= len;
    yCoord /= len;
    return len;
}

inline void Point2D::rotate(double matrix[2][2], Point2D centroid){
    *this -= centroid;
    rot(matrix);
    *this += centroid;
}

inline Point2D Point2D::operator += (const Point2D& otherPoint) {
    xCoord += otherPoint.xCoord;
    yCoord += otherPoint.yCoord;
    return *this;
}

inline Point2D Point2D::operator -= (const Point2D& otherPoint) {
    xCoord -= otherPoint.xCoord;
    yCoord -= otherPoint.yCoord;
    return *this;
}

inline Point2D Point2D::operator *= (const double& scalar) {
    xCoord *= scalar;
    yCoord *= scalar;
    return *this;
}

inline Point2D Point2D::operator *= (const Point2D& otherPoint) {
    xCoord *= otherPoint.xCoord;
    yCoord *= otherPoint.yCoord;
    return *this;
}

inline Point2D Point2D::operator /= (const double& scalar) {
    xCoord /= scalar;
    yCoord /= scalar;
    return *this;
}

inline Point2D Point2D::operator /= (const Point2D& otherPoint) {
    xCoord /= otherPoint.xCoord;
    yCoord /= otherPoint.yCoord;
    return *this;
}

inline Point2D operator * (const double& scalar, const Point2D& point) {
    return Point2D(point.x() * scalar,
                   point.y() * scalar);
}

inline Point2D normalOfSegment(const Point2D& p1, const Point2D& p2) {
    double matrix[2][2] = {{0,-1},
                           {1,0}};

    Point2D p = p1 - p2;
    p.normalize();
    p.rotate(matrix);
    return p;
}

inline double area(const Point2D& p0, const Point2D& p1, const Point2D& p2){
    //return (p1.x() - p0.x())*(p2.y()-p0.y()) - ((p2.x()-p0.x())*(p1.y()-p0.y()))/2;
    double A = -(p1.y() - p0.y());
    double B = p1.x() - p0.x();
    double C = -(A * p0.x() + B * p0.y());
    return A * p2.x() + B * p2.y() + C;
}

#endif // POINT2D_H
