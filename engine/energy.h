#ifndef ENERGY_H
#define ENERGY_H

#include "lib/grid/drawablegrid.h"
#include "box.h"

#define EPSILON_GRAD 1e-8

class Energy{
    public:
        Energy();
        Energy(DrawableGrid& g);

        // Gradient Discend
        double gradientDiscend(Box3D &b) const;

        // Gradient
        void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;

        // Barrier
        double lowConstraint(const Pointd &min, const Pointd &c, double s) const;
        double highConstraint(const Pointd &max, const Pointd &c, double s) const;

        double gBarrier(double x, double s) const;
        double fi(double x, double s) const;

        double barrierEnergy(const Box3D &b, double s = 0.2) const;

        // Integral
        static double integralTricubicInterpolation(const double* & a, double u1, double v1, double w1, double u2, double v2, double w2);
        double integralTricubicInterpolationEnergy(const Pointd& min, const Pointd& max) const;

        // Generic Evaluating Function
        double evaluateTricubicInterpolationFunction(const Pointd& bmin, const Pointd& bmax, double (*f)(const double* &a, double u1, double v1, double w1, double u2, double v2, double w2)) const;

        // Total Energy
        double energy(const Box3D& b) const;
        double energy(const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
        double energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;

    private:
        const DrawableGrid* g;


};

#endif // ENERGY_H
