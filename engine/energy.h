#ifndef ENERGY_H
#define ENERGY_H

#include "lib/grid/drawablegrid.h"
#include "boxlist.h"

#define EPSILON_GRAD 1e-8
#define S_BARRIER 0.2

class Energy{
    public:
        Energy();
        Energy(const Grid& g);

        bool isInside(const Eigen::VectorXd &x) const;
        void calculateFullBoxValues(Grid& g) const;

        // Gradient Discend

        bool wolfeConditions(const Eigen::VectorXd& x, double alfa, const Eigen::VectorXd& direction, const Pointd& c1, const Pointd& c2, const Pointd& c3, double cos2) const;
        bool strongWolfeConditions(const Eigen::VectorXd& x, double alfa, const Eigen::VectorXd& direction, const Pointd& c1, const Pointd& c2, const Pointd& c3, double cos2) const;
        bool wolfeConditions(double value, double newValue, const Eigen::VectorXd &p, const Eigen::VectorXd &gradient, const Eigen::VectorXd& newGradient, double alfa, double cos2 = 0.1) const;

        int gradientDiscend(Box3D &b) const;
        int gradientDiscend(Box3D &b, BoxList& iterations, bool saveIt = true) const;

        int BFGS(Box3D &b) const;
        int BFGS(Box3D &b, BoxList& iterations, bool saveIt = true) const;

        //Gradient Barrier
        double derivateGBarrier(double x, double s) const;
        double derivateFi(double x, double s) const;
        void gradientBarrier(Eigen::VectorXd &gBarrier, const Eigen::VectorXd &x, const Pointd& c1, const Pointd& c2, const Pointd& c3, double s = S_BARRIER) const;
        void gradientBarrier(Eigen::VectorXd& gBarrier, const Box3D& b, double s = S_BARRIER) const;

        // Gradient
        static double gradientXMinComponentOld(const double* &a, double u1, double v1, double w1, double v2, double w2);
        static double gradientYMinComponentOld(const double* &a, double u1, double v1, double w1, double u2, double w2);
        static double gradientZMinComponentOld(const double* &a, double u1, double v1, double w1, double u2, double v2);
        static double gradientXMaxComponentOld(const double* &a, double v1, double w1, double u2, double v2, double w2);
        static double gradientYMaxComponentOld(const double* &a, double u1, double w1, double u2, double v2, double w2);
        static double gradientZMaxComponentOld(const double* &a, double u1, double v1, double u2, double v2, double w2);

        static double gradientXMinComponent(const double* &a, double u1, double v1, double w1, double v2, double w2);
        static double gradientYMinComponent(const double* &a, double u1, double v1, double w1, double u2, double w2);
        static double gradientZMinComponent(const double* &a, double u1, double v1, double w1, double u2, double v2);
        static double gradientXMaxComponent(const double* &a, double v1, double w1, double u2, double v2, double w2);
        static double gradientYMaxComponent(const double* &a, double u1, double w1, double u2, double v2, double w2);
        static double gradientZMaxComponent(const double* &a, double u1, double v1, double u2, double v2, double w2);

        double gradientEvaluateXMinComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateYMinComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateZMinComponent(const Eigen::VectorXd &x) const;
        double gradientEvaluateXMaxComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateYMaxComponent(const Eigen::VectorXd &x) const;
        double gradientEvaluateZMaxComponent(const Eigen::VectorXd& x) const;
        void gradientTricubicInterpolationEnergy(Eigen::VectorXd &gradient, const Pointd& min, const Pointd& max) const;
        void gradientTricubicInterpolationEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x) const;
        void gradientEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
        void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Box3D b) const;
        void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;

        // Barrier
        double lowConstraint(const Pointd &min, const Pointd &c, double s) const;
        double highConstraint(const Pointd &max, const Pointd &c, double s) const;

        double gBarrier(double x, double s) const;
        double fi(double x, double s) const;

        double barrierEnergy(const Box3D &b, double s = S_BARRIER) const;

        // Integral
        static double integralTricubicInterpolationOld(const double* & a, double u1, double v1, double w1, double u2, double v2, double w2);
        static double integralTricubicInterpolation(const double* & a, double u1, double v1, double w1, double u2, double v2, double w2);
        double integralTricubicInterpolationEnergy(const Pointd& min, const Pointd& max) const;
        double integralTricubicInterpolationEnergy(const Eigen::VectorXd &x) const;

        // Total Energy
        double energy(const Box3D& b) const;
        double energy(const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;
        double energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const Pointd& c1, const Pointd& c2, const Pointd& c3) const;

    private:
        void initializeMinMax(Pointd& min, Pointd& max, const Eigen::VectorXd &x) const;
        double volumeOfBox(const Eigen::VectorXd &x) const;

        const Grid* g;

};

inline bool Energy::isInside(const Eigen::VectorXd& x) const {
    return g->getBoundingBox().isIntern(Pointd(x(0), x(1), x(2))) && g->getBoundingBox().isIntern(Pointd(x(3), x(4), x(5)));
}

inline int Energy::gradientDiscend(Box3D& b) const {
    BoxList dummy;
    return gradientDiscend(b, dummy, false);
}

inline int Energy::BFGS(Box3D& b) const {
    BoxList dummy;
    return BFGS(b, dummy, false);
}

inline void Energy::gradientTricubicInterpolationEnergy(Eigen::VectorXd& gradient, const Pointd& min, const Pointd& max) const {
    Eigen::VectorXd x(6);
    x << min.x(), min.y(), min.z(), max.x(), max.y(), max.z();

    gradient(0) = gradientEvaluateXMinComponent(x);
    gradient(1) = gradientEvaluateYMinComponent(x);
    gradient(2) = gradientEvaluateZMinComponent(x);
    gradient(3) = gradientEvaluateXMaxComponent(x);
    gradient(4) = gradientEvaluateYMaxComponent(x);
    gradient(5) = gradientEvaluateZMaxComponent(x);
}

inline void Energy::gradientTricubicInterpolationEnergy(Eigen::VectorXd& gradient, const Eigen::VectorXd& x) const {
    gradient(0) = gradientEvaluateXMinComponent(x);
    gradient(1) = gradientEvaluateYMinComponent(x);
    gradient(2) = gradientEvaluateZMinComponent(x);
    gradient(3) = gradientEvaluateXMaxComponent(x);
    gradient(4) = gradientEvaluateYMaxComponent(x);
    gradient(5) = gradientEvaluateZMaxComponent(x);
}

inline double Energy::derivateGBarrier(double x, double s) const {
    return (3/(pow(s,3)))*pow(x,2) - (6/(pow(s,2)))*x + 3/s;
}

inline double Energy::derivateFi(double x, double s) const {
    return x <= 0 ? 0 :
                    x > s ?
                        0 : -(derivateGBarrier(x,s)/pow(gBarrier(x,s), 2));
}

inline double Energy::lowConstraint(const Pointd& min, const Pointd& c, double s) const {
    return fi(c.x()-min.x(),s) + fi(c.y()-min.y(),s)+ fi(c.z()-min.z(),s);
}

inline double Energy::highConstraint(const Pointd& max, const Pointd& c, double s) const {
    return fi(max.x()-c.x(),s) + fi(max.y()-c.y(),s) + fi(max.z()-c.z(),s);
}

inline double Energy::gBarrier(double x, double s) const {
    return (1/(pow(s,3)))*pow(x,3) - (3/(pow(s,2)))*pow(x,2) + (3/(s))*x;
}

inline double Energy::fi(double x, double s) const {
    return x <= 0 ? std::numeric_limits<double>::max() :
                    x > s ?
                        0 : (1 / gBarrier(x, s) - 1);
}

inline double Energy::barrierEnergy(const Box3D& b, double s) const {
    Pointd c1 = b.getConstraint1(), c2 = b.getConstraint2(), c3 = b.getConstraint3();
    Pointd min = b.getMin(), max = b.getMax();
    return lowConstraint(min, c1, s) + lowConstraint(min, c2, s) + lowConstraint(min, c3, s) + highConstraint(max, c1, s) + highConstraint(max, c2, s) + highConstraint(max, c3, s);
}

inline double Energy::energy(const Box3D& b) const {
    return integralTricubicInterpolationEnergy(b.getMin(), b.getMax()) + barrierEnergy(b);
}

inline double Energy::energy(const Eigen::VectorXd &x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    assert(x.rows() == 6);
    Pointd min(x(0), x(1), x(2)), max(x(3), x(4), x(5));
    Box3D b(min, max, c1, c2, c3);
    return integralTricubicInterpolationEnergy(x) + barrierEnergy(b);
}

inline double Energy::energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    Box3D b(Pointd(minx, miny, minz), Pointd(maxx, maxy, maxz), c1, c2, c3);
    return integralTricubicInterpolationEnergy(b.getMin(), b.getMax()) + barrierEnergy(b);
}

inline void Energy::initializeMinMax(Pointd& min, Pointd&max, const Eigen::VectorXd& x) const{
    double unit = g->getUnit();
    const BoundingBox &bb = g->getBoundingBox();
    if (bb.isStrictlyIntern(x(0), x(1), x(2))){
        min = g->getNearestGridPoint(Pointd(x(0), x(1), x(2)));
    }
    else {
        const Pointd &c = bb.getMin();
        Pointd d = c-Pointd(x(0), x(1), x(2));
        Pointd nu = d / unit;
        Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
        Pointi di = ni * unit;
        min = c - Pointd(di.x(), di.y(), di.z());
    }
    if ((bb.isStrictlyIntern(x(3), x(4), x(5)))){
        max = g->getNearestGridPoint(Pointd(x(3), x(4), x(5)));
    }
    else {
        const Pointd &c = bb.getMin();
        Pointd d = c-Pointd(x(3), x(4), x(5));
        Pointd nu = d / unit;
        Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
        Pointi di = ni * unit;
        max = c - Pointd(di.x(), di.y(), di.z());
    }
}

inline double Energy::volumeOfBox(const Eigen::VectorXd& x) const {
    return (x(3)-x(0))*(x(4)-x(1))*(x(5)-x(2));

}

#endif // ENERGY_H
