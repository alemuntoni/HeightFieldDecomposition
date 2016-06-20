#ifndef ENERGY_H
#define ENERGY_H

#include "lib/grid/drawablegrid.h"
#include "box.h"


class Energy{
    public:
        Energy();
        Energy(DrawableGrid& g);

        // Integral
        static double integralTricubicInterpolation(const Eigen::VectorXd& a, double u1, double v1, double w1, double u2, double v2, double w2);
        double integralTricubicInterpolationEnergy(const Box3D& b);

        // Generic Evaluating Function
        double evaluateTricubicInterpolationFunction(const Box3D &b, double (*f)(const Eigen::VectorXd &a, double u1, double v1, double w1, double u2, double v2, double w2));

        // Total Energy
        double energy(const Box3D& b);

    private:
        DrawableGrid* g;


};

#endif // ENERGY_H
