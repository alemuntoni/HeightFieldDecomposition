#ifndef ENERGY_H
#define ENERGY_H

#include "lib/grid/drawablegrid.h"
#include "box.h"


class Energy{
    public:
        Energy();
        Energy(DrawableGrid& g);

        // Barrier
        double lowConstraint(const Pointd &min, const Pointd &c, double s);
        double highConstraint(const Pointd &max, const Pointd &c, double s);

        double gBarrier(double x, double s);
        double fi(double x, double s);

        double barrierEnergy(const Box3D &b, double s = 0.2);

        // Integral
        static double integralTricubicInterpolation(const std::vector<double>& a, double u1, double v1, double w1, double u2, double v2, double w2);
        double integralTricubicInterpolationEnergy(const Box3D& b);

        // Generic Evaluating Function
        double evaluateTricubicInterpolationFunction(const Box3D &b, double (*f)(const std::vector<double> &a, double u1, double v1, double w1, double u2, double v2, double w2));

        // Total Energy
        double energy(const Box3D& b);

    private:
        DrawableGrid* g;


};

#endif // ENERGY_H
