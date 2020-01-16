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

		bool wolfeConditions(const Eigen::VectorXd& x, double alfa, const Eigen::VectorXd& direction, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, double cos2) const;
		bool strongWolfeConditions(const Eigen::VectorXd& x, double alfa, const Eigen::VectorXd& direction, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, double cos2) const;
        bool wolfeConditions(double value, double newValue, const Eigen::VectorXd &p, const Eigen::VectorXd &gradient, const Eigen::VectorXd& newGradient, double alfa, double cos2 = 0.1) const;

        int gradientDiscend(Box3D &b) const;
        int gradientDiscend(Box3D &b, BoxList& iterations, bool saveIt = true) const;

        int BFGS(Box3D &b) const;
		int BFGS(Box3D &b, const cg3::Point3d &limits) const;
        int BFGS(Box3D &b, BoxList& iterations, bool saveIt = true) const;
		int BFGS(Box3D &b, const cg3::Point3d& limits, BoxList& iterations, bool saveIt = true) const;

        //Gradient Barrier
        double derivateGBarrier(double x, double s) const;
        double derivateFi(double x, double s) const;
		void gradientBarrier(Eigen::VectorXd &gBarrier, const Eigen::VectorXd &x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, double s = S_BARRIER) const;
        void gradientBarrier(Eigen::VectorXd& gBarrier, const Box3D& b, double s = S_BARRIER) const;
		void gradientBarrierLimits(Eigen::VectorXd& gBarrier, const Eigen::VectorXd &x, const cg3::Point3d& l, double s = S_BARRIER) const;
		void gradientBarrierLimits(Eigen::VectorXd& gBarrier, const cg3::BoundingBox3& b, const cg3::Point3d& l, double s = S_BARRIER) const;

        // Gradient
        static double gradientXMinComponent(const gridreal*& a, double u1, double v1, double w1, double v2, double w2);
        static double gradientYMinComponent(const gridreal* &a, double u1, double v1, double w1, double u2, double w2);
        static double gradientZMinComponent(const gridreal*& a, double u1, double v1, double w1, double u2, double v2);
        static double gradientXMaxComponent(const gridreal* &a, double v1, double w1, double u2, double v2, double w2);
        static double gradientYMaxComponent(const gridreal*& a, double u1, double w1, double u2, double v2, double w2);
        static double gradientZMaxComponent(const gridreal* &a, double u1, double v1, double u2, double v2, double w2);

        double gradientEvaluateXMinComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateYMinComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateZMinComponent(const Eigen::VectorXd &x) const;
        double gradientEvaluateXMaxComponent(const Eigen::VectorXd& x) const;
        double gradientEvaluateYMaxComponent(const Eigen::VectorXd &x) const;
        double gradientEvaluateZMaxComponent(const Eigen::VectorXd& x) const;
		void gradientTricubicInterpolationEnergy(Eigen::VectorXd &gradient, const cg3::Point3d& min, const cg3::Point3d& max) const;
        void gradientTricubicInterpolationEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x) const;
		void gradientEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const;
		void gradientEnergy(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, const cg3::Point3d &l) const;
        void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Box3D b) const;
		void gradientEnergyFiniteDifference(Eigen::VectorXd &gradient, const Eigen::VectorXd& x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const;

        // Barrier
		double lowConstraint(const cg3::Point3d &min, const cg3::Point3d &c, double s) const;
		double highConstraint(const cg3::Point3d &max, const cg3::Point3d &c, double s) const;

        double gBarrier(double x, double s) const;
        double fi(double x, double s) const;

        double barrierEnergy(const Box3D &b, double s = S_BARRIER) const;
		double barrierLimitsEnergy(const cg3::BoundingBox3& b, const cg3::Point3d& limits, double s = S_BARRIER) const;

        // Integral
        static double integralTricubicInterpolation(const gridreal*& a, double u1, double v1, double w1, double u2, double v2, double w2);
		double integralTricubicInterpolationEnergy(const cg3::Point3d& min, const cg3::Point3d& max) const;
        double integralTricubicInterpolationEnergy(const Eigen::VectorXd &x) const;

        // Total Energy
        double energy(const Box3D& b) const;
		double energy(const Eigen::VectorXd& x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const;
		double energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const;
		double energy(const Box3D& b, const cg3::Point3d &limits) const;
		double energy(const Eigen::VectorXd& x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, const cg3::Point3d &limits) const;
		double energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, const cg3::Point3d &limits) const;

    private:
		void initializeMinMax(cg3::Point3d& min, cg3::Point3d& max, const Eigen::VectorXd &x) const;
        double volumeOfBox(const Eigen::VectorXd &x) const;

        const Grid* g;

};

inline bool Energy::isInside(const Eigen::VectorXd& x) const {
	return g->getBoundingBox().isIntern(cg3::Point3d(x(0), x(1), x(2))) && g->getBoundingBox().isIntern(cg3::Point3d(x(3), x(4), x(5)));
}

inline int Energy::gradientDiscend(Box3D& b) const {
    BoxList dummy;
    return gradientDiscend(b, dummy, false);
}

inline int Energy::BFGS(Box3D& b) const {
    BoxList dummy;
    return BFGS(b, dummy, false);
}

inline int Energy::BFGS(Box3D& b, const cg3::Point3d& limits) const {
    BoxList dummy;
    return BFGS(b, limits, dummy, false);
}

inline void Energy::gradientTricubicInterpolationEnergy(Eigen::VectorXd& gradient, const cg3::Point3d& min, const cg3::Point3d& max) const {
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
    return (3/(s*s*s))*(x*x) - (6/(s*s))*x + 3/s;
}

inline double Energy::derivateFi(double x, double s) const {
    return x <= 0 ? 0 :
                    x > s ?
                        0 : -(derivateGBarrier(x,s)/(gBarrier(x,s)*gBarrier(x,s)));
}

inline double Energy::lowConstraint(const cg3::Point3d& min, const cg3::Point3d& c, double s) const {
    return fi(c.x()-min.x(),s) + fi(c.y()-min.y(),s)+ fi(c.z()-min.z(),s);
}

inline double Energy::highConstraint(const cg3::Point3d& max, const cg3::Point3d& c, double s) const {
    return fi(max.x()-c.x(),s) + fi(max.y()-c.y(),s) + fi(max.z()-c.z(),s);
}

inline double Energy::gBarrier(double x, double s) const {
    return (1/((s*s*s)))*(x*x*x) - (3/((s*s)))*(x*x) + (3/(s))*x;
}

inline double Energy::fi(double x, double s) const {
    return x <= 0 ? std::numeric_limits<double>::max() :
                    x > s ?
                        0 : (1 / gBarrier(x, s) - 1);
}

inline double Energy::barrierEnergy(const Box3D& b, double s) const {
	cg3::Point3d c1 = b.getConstraint1(), c2 = b.getConstraint2(), c3 = b.getConstraint3();
	cg3::Point3d min = b.min(), max = b.max();
    return lowConstraint(min, c1, s) + lowConstraint(min, c2, s) + lowConstraint(min, c3, s) + highConstraint(max, c1, s) + highConstraint(max, c2, s) + highConstraint(max, c3, s);
}

inline double Energy::barrierLimitsEnergy(const cg3::BoundingBox3& b, const cg3::Point3d& limits, double s) const {
	cg3::Point3d bl(b.lengthX(), b.lengthY(), b.lengthZ());
    return lowConstraint(bl, limits, s);
}



inline double Energy::energy(const Box3D& b) const {
	return integralTricubicInterpolationEnergy(b.min(), b.max()) + barrierEnergy(b);
}

inline double Energy::energy(const Eigen::VectorXd &x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const {
    assert(x.rows() == 6);
	cg3::Point3d min(x(0), x(1), x(2)), max(x(3), x(4), x(5));
    Box3D b(min, max, c1, c2, c3);
    return integralTricubicInterpolationEnergy(x) + barrierEnergy(b);
}

inline double Energy::energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3) const {
	Box3D b(cg3::Point3d(minx, miny, minz), cg3::Point3d(maxx, maxy, maxz), c1, c2, c3);
	return integralTricubicInterpolationEnergy(b.min(), b.max()) + barrierEnergy(b);
}

inline double Energy::energy(const Box3D& b, const cg3::Point3d &limits) const {
    return energy(b)+ barrierLimitsEnergy(b,limits);
}

inline double Energy::energy(const Eigen::VectorXd &x, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, const cg3::Point3d &limits) const {
    assert(x.rows() == 6);
	cg3::Point3d min(x(0), x(1), x(2)), max(x(3), x(4), x(5));
    Box3D b(min, max, c1, c2, c3);
    return integralTricubicInterpolationEnergy(x) + barrierEnergy(b) + barrierLimitsEnergy(b,limits);
}

inline double Energy::energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const cg3::Point3d& c1, const cg3::Point3d& c2, const cg3::Point3d& c3, const cg3::Point3d &limits) const {
	Box3D b(cg3::Point3d(minx, miny, minz), cg3::Point3d(maxx, maxy, maxz), c1, c2, c3);
	return integralTricubicInterpolationEnergy(b.min(), b.max()) + barrierEnergy(b) + barrierLimitsEnergy(b,limits);
}

inline void Energy::initializeMinMax(cg3::Point3d& min, cg3::Point3d& max, const Eigen::VectorXd& x) const{
    double unit = g->getUnit();
	const cg3::BoundingBox3 &bb = g->getBoundingBox();
    if (bb.isStrictlyIntern(x(0), x(1), x(2))){
		min = g->getNearestGridPoint(cg3::Point3d(x(0), x(1), x(2)));
    }
    else {
		const cg3::Point3d &c = bb.min();
		cg3::Point3d d = c-cg3::Point3d(x(0), x(1), x(2));
		cg3::Point3d nu = d / unit;
		cg3::Point3i ni = cg3::Point3i(nu.x(), nu.y(), nu.z());
		cg3::Point3i di = ni * unit;
		min = c - cg3::Point3d(di.x(), di.y(), di.z());
    }
    if ((bb.isStrictlyIntern(x(3), x(4), x(5)))){
		max = g->getNearestGridPoint(cg3::Point3d(x(3), x(4), x(5)));
    }
    else {
		const cg3::Point3d &c = bb.min();
		cg3::Point3d d = c-cg3::Point3d(x(3), x(4), x(5));
		cg3::Point3d nu = d / unit;
		cg3::Point3i ni = cg3::Point3i(nu.x(), nu.y(), nu.z());
		cg3::Point3i di = ni * unit;
		max = c - cg3::Point3d(di.x(), di.y(), di.z());
    }
}

inline double Energy::volumeOfBox(const Eigen::VectorXd& x) const {
    return (x(3)-x(0))*(x(4)-x(1))*(x(5)-x(2));

}

#endif // ENERGY_H
