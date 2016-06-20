#include "energy.h"


Energy::Energy() {
}

Energy::Energy(DrawableGrid& g) : g(&g){
}

double Energy::integralTricubicInterpolation(const Eigen::VectorXd& a, double u1, double v1, double w1, double u2, double v2, double w2) {
    //Simpy generated code
    assert (a.rows() == 64);

}

double Energy::integralTricubicInterpolationEnergy(const Box& b) {
    return evaluateTricubicInterpolationFunction(b, integralTricubicInterpolation);
}

double Energy::evaluateTricubicInterpolationFunction(const Box& b, double (*f)(const Eigen::VectorXd&, double, double, double, double, double, double)) {

}

double Energy::energy(const Box& b) {
    return integralTricubicInterpolationEnergy(b)/* + barrierEnergy(b)*/;
}
