#ifndef ENERGY_H
#define ENERGY_H

#include "lib/grid/drawablegrid.h"
#include "box.h"
#include "boxlist.h"

#define EPSILON_GRAD 1e-8

class Energy{
    public:
        Energy();
        Energy(DrawableGrid& g);

        // Gradient Discend
        double gradientDiscend(Box3D &b, BoxList& iterations) const;

        // Gradient
        static double gradientXMinComponent(const double* &a, double u1, double v1, double w1, double v2, double w2);
        static double gradientYMinComponent(const double* &a, double u1, double v1, double w1, double u2, double w2);
        static double gradientZMinComponent(const double* &a, double u1, double v1, double w1, double u2, double v2);
        static double gradientXMaxComponent(const double* &a, double v1, double w1, double u2, double v2, double w2);
        static double gradientYMaxComponent(const double* &a, double u1, double w1, double u2, double v2, double w2);
        static double gradientZMaxComponent(const double* &a, double u1, double v1, double u2, double v2, double w2);

        double gradientEvaluateComponent(const Pointd& bmin, const Pointd& bmax, double (*f)(const double* &a, double u1, double v1, double w1, double u2, double v2, double w2)) const;
        double gradientEvaluateXMinComponent(const Pointd& bmin, const Pointd& bmax) const;
        double gradientEvaluateYMinComponent(const Pointd& bmin, const Pointd& bmax) const;
        double gradientEvaluateZMinComponent(const Pointd& bmin, const Pointd& bmax) const;
        double gradientEvaluateXMaxComponent(const Pointd& bmin, const Pointd& bmax) const;
        double gradientEvaluateYMaxComponent(const Pointd& bmin, const Pointd& bmax) const;
        double gradientEvaluateZMaxComponent(const Pointd& bmin, const Pointd& bmax) const;
        void gradientTricubicInterpolationEnergy(Eigen::VectorXd &gradient, const Pointd& min, const Pointd& max) const;
        void gradientEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
        void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Box3D b) const;
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

        // Total Energy
        double energy(const Box3D& b) const;
        double energy(const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
        double energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
    private:
        DrawableGrid* g;


};

#endif // ENERGY_H
