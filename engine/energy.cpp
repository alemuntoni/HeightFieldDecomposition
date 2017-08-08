#include "energy.h"

#include "common/timer.h"
#include "common.h"

Energy::Energy() {
}

Energy::Energy(const Grid& g) : g(&g){
}

void Energy::calculateFullBoxValues(Grid &g) const {
    g.calculateFullBoxValues(integralTricubicInterpolation);
}

bool Energy::wolfeConditions(const Eigen::VectorXd &x, double alfa, const Eigen::VectorXd &direction, const Pointd &c1, const Pointd &c2, const Pointd &c3, double cos2) const {
    double cos1 = 1e-4;
    Eigen::VectorXd gradient(6);
    gradientEnergy(gradient, x, c1, c2, c3);
    bool con1 = energy(x+alfa*direction, c1, c2, c3) <= energy(x,c1,c2,c3)+cos1*alfa*gradient.transpose()*direction;
    Eigen::VectorXd newgradient(6);
    gradientEnergy(newgradient, x+alfa*direction, c1, c2, c3);
    bool con2 = newgradient.transpose()*direction >= cos2*gradient.transpose()*direction;
    return con1 && con2;
}

bool Energy::strongWolfeConditions(const Eigen::VectorXd &x, double alfa, const Eigen::VectorXd &direction, const Pointd &c1, const Pointd &c2, const Pointd &c3, double cos2) const {
    double cos1 = 1e-4;
    Eigen::VectorXd gradient(6);
    gradientEnergy(gradient, x, c1, c2, c3);
    bool con1 = energy(x+alfa*direction, c1, c2, c3) <= energy(x,c1,c2,c3)+cos1*alfa*gradient.transpose()*direction;
    Eigen::VectorXd newgradient(6);
    gradientEnergy(newgradient, x+alfa*direction, c1, c2, c3);
    bool con2 = std::abs(newgradient.transpose()*direction) >= cos2*std::abs(gradient.transpose()*direction);
    return con1 && con2;
}

bool Energy::wolfeConditions(double value, double newValue, const Eigen::VectorXd &p, const Eigen::VectorXd &gradient, const Eigen::VectorXd &newGradient, double alfa, double cos2) const {
    double cos1 = 1e-4;
    bool con1 = newValue <= value + cos1*alfa*gradient.transpose()*p;
    bool con2 = std::abs(newGradient.transpose()*p) >= cos2*std::abs(gradient.transpose()*p);
    return con1 && con2;
}

int Energy::gradientDiscend(Box3D& b, BoxList& iterations, bool saveIt) const {
    int nIterations = 0;
    double objValue, newObjValue;
    double alfa = 1;
    Pointd c1 = b.getConstraint1();
    Pointd c2 = b.getConstraint2();
    Pointd c3 = b.getConstraint3();
    Eigen::VectorXd x(6);
    x << b.getMin().x(), b.getMin().y(), b.getMin().z(), b.getMax().x(), b.getMax().y(), b.getMax().z();
    Eigen::VectorXd new_x(6);
    Eigen::VectorXd gradient(6);
    Eigen::VectorXd newGradient(6);
    objValue = energy(x, c1, c2, c3);
    gradientEnergy(gradient, x, c1, c2, c3);
    alfa = 1 / gradient.norm();
    do{
        new_x = x - alfa * gradient;
        newObjValue = energy(new_x, c1, c2, c3);
        gradientEnergy(newGradient, new_x, c1, c2, c3);
        //if (wolfeConditions(objValue, newObjValue, -gradient, gradient, newGradient, alfa)) {
        if (newObjValue < objValue) {
            nIterations++;
            x = new_x;

            objValue = newObjValue;
            gradient = newGradient;
            if (saveIt){
                b.setMin(Pointd(x(0), x(1), x(2)));
                b.setMax(Pointd(x(3), x(4), x(5)));
                iterations.addBox(b);
            }
            alfa*=2;
            std::cerr.precision(17);
            std::cerr << "Energy: " << newObjValue << "\n\nGradient: \n" << gradient << "\n\nalfa: " << alfa << "\n\n\n";
        }
        else{
            alfa /= 2;
        }
        //std::cerr << new_x << "\n\n";
    } while (/*nIterations < 500 &&*/ gradient.norm() > 1e-3 && alfa > 0);
    std::cerr << "Alfa: " << alfa << "\n\n";
    std::cerr << "Gradient norm: " << gradient.norm() << "\n";
    b.setMin(Pointd(x(0), x(1), x(2)));
    b.setMax(Pointd(x(3), x(4), x(5)));
    if (saveIt) iterations.addBox(b);
    return nIterations;
}

int Energy::BFGS(Box3D& b, BoxList& iterations, bool saveIt) const {
    int nIterations = 0;
    double alfa = 1;
    double ro;
    Pointd c1 = b.getConstraint1();
    Pointd c2 = b.getConstraint2();
    Pointd c3 = b.getConstraint3();
    Eigen::VectorXd x(6);
    x << b.getMin().x(), b.getMin().y(), b.getMin().z(), b.getMax().x(), b.getMax().y(), b.getMax().z();
    Eigen::VectorXd new_x(6), gradient(6), newGradient(6), direction(6);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Binv = I;
    Eigen::VectorXd s(6), y(6);
    double objValue = energy(x, c1, c2, c3), newObjValue;

    gradientEnergy(gradient, x, c1, c2, c3);

    direction = -Binv*gradient;

    do{
        new_x = x +alfa * direction;
        while ((! g->getBoundingBox().isIntern(new_x(0), new_x(1), new_x(2)) || ! g->getBoundingBox().isIntern(new_x(3), new_x(4), new_x(5))) && alfa != 0){
            alfa /= 2;
            new_x = x +alfa * direction;
        }

        newObjValue = energy(new_x, c1, c2, c3);

        while (newObjValue >= objValue && alfa > 1e-10){
            alfa/= 2;
            new_x = x +alfa * direction;

            newObjValue = energy(new_x, c1, c2, c3);
        }

        if (alfa > 1e-10){
            gradientEnergy(newGradient, new_x, c1, c2, c3);

            new_x = x +alfa * direction;

            s = new_x - x;
            y = newGradient - gradient;
            double tmp = (y.transpose()*s);
            ro = 1.0 / tmp;
            Binv = (I - ro*s*y.transpose())*Binv*(I - ro*y*s.transpose()) + ro*s*s.transpose();
            if (saveIt){
                b.setMin(Pointd(x(0), x(1), x(2)));
                b.setMax(Pointd(x(3), x(4), x(5)));
                iterations.addBox(b);
            }
            x = new_x;
            objValue = newObjValue;
            nIterations++;
            gradient = newGradient;
            direction = -Binv*gradient;
            double dot = gradient.dot(direction);
            if (dot >= 0) {
                Binv = I;
                direction = -gradient;
            }
            alfa *= 2;
        }
    }while (alfa > 1e-6 && gradient.norm() > 1e-7 && nIterations < MAX_BFGS_ITERATIONS);
    b.setMin(Pointd(x(0), x(1), x(2)));
    b.setMax(Pointd(x(3), x(4), x(5)));
    if (saveIt) iterations.addBox(b);

    return nIterations;
}

int Energy::BFGS(Box3D& b, const Pointd& limits, BoxList& iterations, bool saveIt) const {
    int nIterations = 0;
    double alfa = 1;
    double ro;
    Pointd c1 = b.getConstraint1();
    Pointd c2 = b.getConstraint2();
    Pointd c3 = b.getConstraint3();
    Eigen::VectorXd x(6);
    x << b.getMin().x(), b.getMin().y(), b.getMin().z(), b.getMax().x(), b.getMax().y(), b.getMax().z();
    Eigen::VectorXd new_x(6), gradient(6), newGradient(6), direction(6);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Binv = I;
    Eigen::VectorXd s(6), y(6);
    double objValue = energy(x, c1, c2, c3, limits), newObjValue;

    gradientEnergy(gradient, x, c1, c2, c3, limits);

    direction = -Binv*gradient;

    do{
        new_x = x +alfa * direction;
        while ((! g->getBoundingBox().isIntern(new_x(0), new_x(1), new_x(2)) || ! g->getBoundingBox().isIntern(new_x(3), new_x(4), new_x(5))) && alfa != 0){
            alfa /= 2;
            new_x = x +alfa * direction;
        }

        newObjValue = energy(new_x, c1, c2, c3, limits);

        while (newObjValue >= objValue && alfa > 1e-10){
            alfa/= 2;
            new_x = x +alfa * direction;

            newObjValue = energy(new_x, c1, c2, c3, limits);
        }

        if (alfa > 1e-10){
            gradientEnergy(newGradient, new_x, c1, c2, c3, limits);

            new_x = x +alfa * direction;

            s = new_x - x;
            y = newGradient - gradient;
            double tmp = (y.transpose()*s);
            ro = 1.0 / tmp;
            Binv = (I - ro*s*y.transpose())*Binv*(I - ro*y*s.transpose()) + ro*s*s.transpose();
            if (saveIt){
                b.setMin(Pointd(x(0), x(1), x(2)));
                b.setMax(Pointd(x(3), x(4), x(5)));
                iterations.addBox(b);
            }
            x = new_x;
            objValue = newObjValue;
            nIterations++;
            gradient = newGradient;
            direction = -Binv*gradient;
            double dot = gradient.dot(direction);
            if (dot >= 0) {
                Binv = I;
                direction = -gradient;
            }
            alfa *= 2;
        }
    }while (alfa > 1e-6 && gradient.norm() > 1e-7 && nIterations < 100);
    b.setMin(Pointd(x(0), x(1), x(2)));
    b.setMax(Pointd(x(3), x(4), x(5)));
    if (saveIt) iterations.addBox(b);

    return nIterations;
}

void Energy::gradientBarrier(Eigen::VectorXd &gBarrier, const Eigen::VectorXd &x, const Pointd& c1, const Pointd& c2, const Pointd& c3, double s)  const {

    gBarrier <<
        -derivateFi(c1.x()-x(0),s) - derivateFi(c2.x()-x(0),s) - derivateFi(c3.x()-x(0),s),
        -derivateFi(c1.y()-x(1),s) - derivateFi(c2.y()-x(1),s) - derivateFi(c3.y()-x(1),s),
        -derivateFi(c1.z()-x(2),s) - derivateFi(c2.z()-x(2),s) - derivateFi(c3.z()-x(2),s),
        derivateFi(x(3)-c1.x(),s) + derivateFi(x(3)-c2.x(),s) + derivateFi(x(3)-c3.x(),s),
        derivateFi(x(4)-c1.y(),s) + derivateFi(x(4)-c2.y(),s) + derivateFi(x(4)-c3.y(),s),
        derivateFi(x(5)-c1.z(),s) + derivateFi(x(5)-c2.z(),s) + derivateFi(x(5)-c3.z(),s);
}

void Energy::gradientBarrier(Eigen::VectorXd &gBarrier, const Box3D& b, double s)  const {

    gBarrier <<
    -derivateFi(b.getConstraint1().x()-b.getMin().x(),s) - derivateFi(b.getConstraint2().x()-b.getMin().x(),s) - derivateFi(b.getConstraint3().x()-b.getMin().x(),s),
    -derivateFi(b.getConstraint1().y()-b.getMin().y(),s) - derivateFi(b.getConstraint2().y()-b.getMin().y(),s) - derivateFi(b.getConstraint3().y()-b.getMin().y(),s),
    -derivateFi(b.getConstraint1().z()-b.getMin().z(),s) - derivateFi(b.getConstraint2().z()-b.getMin().z(),s) - derivateFi(b.getConstraint3().z()-b.getMin().z(),s),
    derivateFi(b.getMax().x()-b.getConstraint1().x(),s) + derivateFi(b.getMax().x()-b.getConstraint2().x(),s) + derivateFi(b.getMax().x()-b.getConstraint3().x(),s),
    derivateFi(b.getMax().y()-b.getConstraint1().y(),s) + derivateFi(b.getMax().y()-b.getConstraint2().y(),s) + derivateFi(b.getMax().y()-b.getConstraint3().y(),s),
    derivateFi(b.getMax().z()-b.getConstraint1().z(),s) + derivateFi(b.getMax().z()-b.getConstraint2().z(),s) + derivateFi(b.getMax().z()-b.getConstraint3().z(),s);
}

void Energy::gradientBarrierLimits(Eigen::VectorXd& gBarrier, const Eigen::VectorXd &x, const Pointd& l, double s) const {
    gBarrier <<
            derivateFi(l.x()-x(3)+x(0),s),
            derivateFi(l.y()-x(4)+x(1),s),
            derivateFi(l.z()-x(5)+x(2),s),
            -derivateFi(l.x()-x(3)+x(0),s),
            -derivateFi(l.y()-x(4)+x(1),s),
            -derivateFi(l.z()-x(5)+x(2),s);
}

void Energy::gradientBarrierLimits(Eigen::VectorXd& gBarrier, const BoundingBox& b, const Pointd& l, double s) const {
    gBarrier <<
            derivateFi(l.x()-b.maxX()+b.minX(),s),
            derivateFi(l.y()-b.maxY()+b.minY(),s),
            derivateFi(l.z()-b.maxZ()+b.minZ(),s),
            -derivateFi(l.x()-b.maxX()+b.minX(),s),
            -derivateFi(l.y()-b.maxY()+b.minY(),s),
            -derivateFi(l.z()-b.maxZ()+b.minZ(),s);
}

double Energy::gradientXMinComponent(const gridreal*& a, double u1, double v1, double w1, double v2, double w2) {
    double u12 = u1*u1; double u13 = u12*u1;
    double v12 = v1*v1; double v13 = v12*v1; double v14 = v13*v1;
    double w12 = w1*w1; double w13 = w12*w1; double w14 = w13*w1;
    double v22 = v2*v2; double v23 = v22*v2; double v24 = v23*v2;
    double w22 = w2*w2; double w23 = w22*w2; double w24 = w23*w2;
    double G_result;
    G_result =
            -w14 *
            ((1.0L/4.0L)*a[48]*v1 - 1.0L/4.0L*a[48]*v2 + (1.0L/4.0L)*a[49]*u1*v1 - 1.0L/4.0L*a[49]*u1*v2 +
            (1.0L/4.0L)*a[50]*u12*v1 - 1.0L/4.0L*a[50]*u12*v2 + (1.0L/4.0L)*a[51]*u13*v1 - 1.0L/4.0L*a[51]*u13*v2 +
            (1.0L/8.0L)*a[52]*v12 - 1.0L/8.0L*a[52]*v22 + (1.0L/8.0L)*a[53]*u1*v12 - 1.0L/8.0L*a[53]*u1*v22 +
            (1.0L/8.0L)*a[54]*u12*v12 - 1.0L/8.0L*a[54]*u12*v22 + (1.0L/8.0L)*a[55]*u13*v12 - 1.0L/8.0L*a[55]*u13*v22 +
            (1.0L/12.0L)*a[56]*v13 - 1.0L/12.0L*a[56]*v23 + (1.0L/12.0L)*a[57]*u1*v13 - 1.0L/12.0L*a[57]*u1*v23 +
            (1.0L/12.0L)*a[58]*u12*v13 - 1.0L/12.0L*a[58]*u12*v23 + (1.0L/12.0L)*a[59]*u13*v13 - 1.0L/12.0L*a[59]*u13*v23 +
            (1.0L/16.0L)*a[60]*v14 - 1.0L/16.0L*a[60]*v24 + (1.0L/16.0L)*a[61]*u1*v14 - 1.0L/16.0L*a[61]*u1*v24 +
            (1.0L/16.0L)*a[62]*u12*v14 - 1.0L/16.0L*a[62]*u12*v24 + (1.0L/16.0L)*a[63]*u13*v14 - 1.0L/16.0L*a[63]*u13*v24)
            - w13 *
            ((1.0L/3.0L)*a[32]*v1 - 1.0L/3.0L*a[32]*v2 + (1.0L/3.0L)*a[33]*u1*v1 - 1.0L/3.0L*a[33]*u1*v2 +
            (1.0L/3.0L)*a[34]*u12*v1 - 1.0L/3.0L*a[34]*u12*v2 + (1.0L/3.0L)*a[35]*u13*v1 - 1.0L/3.0L*a[35]*u13*v2 +
            (1.0L/6.0L)*a[36]*v12 - 1.0L/6.0L*a[36]*v22 + (1.0L/6.0L)*a[37]*u1*v12 - 1.0L/6.0L*a[37]*u1*v22 +
            (1.0L/6.0L)*a[38]*u12*v12 - 1.0L/6.0L*a[38]*u12*v22 + (1.0L/6.0L)*a[39]*u13*v12 - 1.0L/6.0L*a[39]*u13*v22 +
            (1.0L/9.0L)*a[40]*v13 - 1.0L/9.0L*a[40]*v23 + (1.0L/9.0L)*a[41]*u1*v13 - 1.0L/9.0L*a[41]*u1*v23 +
            (1.0L/9.0L)*a[42]*u12*v13 - 1.0L/9.0L*a[42]*u12*v23 + (1.0L/9.0L)*a[43]*u13*v13 - 1.0L/9.0L*a[43]*u13*v23 +
            (1.0L/12.0L)*a[44]*v14 - 1.0L/12.0L*a[44]*v24 + (1.0L/12.0L)*a[45]*u1*v14 - 1.0L/12.0L*a[45]*u1*v24 +
            (1.0L/12.0L)*a[46]*u12*v14 - 1.0L/12.0L*a[46]*u12*v24 + (1.0L/12.0L)*a[47]*u13*v14 - 1.0L/12.0L*a[47]*u13*v24)
            - w12 *
            ((1.0L/2.0L)*a[16]*v1 - 1.0L/2.0L*a[16]*v2 + (1.0L/2.0L)*a[17]*u1*v1 - 1.0L/2.0L*a[17]*u1*v2 +
            (1.0L/2.0L)*a[18]*u12*v1 - 1.0L/2.0L*a[18]*u12*v2 + (1.0L/2.0L)*a[19]*u13*v1 - 1.0L/2.0L*a[19]*u13*v2 +
            (1.0L/4.0L)*a[20]*v12 - 1.0L/4.0L*a[20]*v22 + (1.0L/4.0L)*a[21]*u1*v12 - 1.0L/4.0L*a[21]*u1*v22 +
            (1.0L/4.0L)*a[22]*u12*v12 - 1.0L/4.0L*a[22]*u12*v22 + (1.0L/4.0L)*a[23]*u13*v12 - 1.0L/4.0L*a[23]*u13*v22 +
            (1.0L/6.0L)*a[24]*v13 - 1.0L/6.0L*a[24]*v23 + (1.0L/6.0L)*a[25]*u1*v13 - 1.0L/6.0L*a[25]*u1*v23 +
            (1.0L/6.0L)*a[26]*u12*v13 - 1.0L/6.0L*a[26]*u12*v23 + (1.0L/6.0L)*a[27]*u13*v13 - 1.0L/6.0L*a[27]*u13*v23 +
            (1.0L/8.0L)*a[28]*v14 - 1.0L/8.0L*a[28]*v24 + (1.0L/8.0L)*a[29]*u1*v14 - 1.0L/8.0L*a[29]*u1*v24 +
            (1.0L/8.0L)*a[30]*u12*v14 - 1.0L/8.0L*a[30]*u12*v24 + (1.0L/8.0L)*a[31]*u13*v14 - 1.0L/8.0L*a[31]*u13*v24)
            - w1 *
            (a[0]*v1 - a[0]*v2 + a[1]*u1*v1 - a[1]*u1*v2 +
            (1.0L/3.0L)*a[10]*u12*v13 - 1.0L/3.0L*a[10]*u12*v23 + (1.0L/3.0L)*a[11]*u13*v13 - 1.0L/3.0L*a[11]*u13*v23 +
            (1.0L/4.0L)*a[12]*v14 - 1.0L/4.0L*a[12]*v24 + (1.0L/4.0L)*a[13]*u1*v14 - 1.0L/4.0L*a[13]*u1*v24 +
            (1.0L/4.0L)*a[14]*u12*v14 - 1.0L/4.0L*a[14]*u12*v24 + (1.0L/4.0L)*a[15]*u13*v14 - 1.0L/4.0L*a[15]*u13*v24 +
            a[2]*u12*v1 - a[2]*u12*v2 + a[3]*u13*v1 - a[3]*u13*v2 +
            (1.0L/2.0L)*a[4]*v12 - 1.0L/2.0L*a[4]*v22 + (1.0L/2.0L)*a[5]*u1*v12 - 1.0L/2.0L*a[5]*u1*v22 +
            (1.0L/2.0L)*a[6]*u12*v12 - 1.0L/2.0L*a[6]*u12*v22 + (1.0L/2.0L)*a[7]*u13*v12 - 1.0L/2.0L*a[7]*u13*v22 +
            (1.0L/3.0L)*a[8]*v13 - 1.0L/3.0L*a[8]*v23 + (1.0L/3.0L)*a[9]*u1*v13 - 1.0L/3.0L*a[9]*u1*v23)
            + w24 *
            ((1.0L/4.0L)*a[48]*v1 - 1.0L/4.0L*a[48]*v2 + (1.0L/4.0L)*a[49]*u1*v1 - 1.0L/4.0L*a[49]*u1*v2 +
            (1.0L/4.0L)*a[50]*u12*v1 - 1.0L/4.0L*a[50]*u12*v2 + (1.0L/4.0L)*a[51]*u13*v1 - 1.0L/4.0L*a[51]*u13*v2 +
            (1.0L/8.0L)*a[52]*v12 - 1.0L/8.0L*a[52]*v22 + (1.0L/8.0L)*a[53]*u1*v12 - 1.0L/8.0L*a[53]*u1*v22 +
            (1.0L/8.0L)*a[54]*u12*v12 - 1.0L/8.0L*a[54]*u12*v22 + (1.0L/8.0L)*a[55]*u13*v12 - 1.0L/8.0L*a[55]*u13*v22 +
            (1.0L/12.0L)*a[56]*v13 - 1.0L/12.0L*a[56]*v23 + (1.0L/12.0L)*a[57]*u1*v13 - 1.0L/12.0L*a[57]*u1*v23 +
            (1.0L/12.0L)*a[58]*u12*v13 - 1.0L/12.0L*a[58]*u12*v23 + (1.0L/12.0L)*a[59]*u13*v13 - 1.0L/12.0L*a[59]*u13*v23 +
            (1.0L/16.0L)*a[60]*v14 - 1.0L/16.0L*a[60]*v24 + (1.0L/16.0L)*a[61]*u1*v14 - 1.0L/16.0L*a[61]*u1*v24 +
            (1.0L/16.0L)*a[62]*u12*v14 - 1.0L/16.0L*a[62]*u12*v24 + (1.0L/16.0L)*a[63]*u13*v14 - 1.0L/16.0L*a[63]*u13*v24)
            + w23 *
            ((1.0L/3.0L)*a[32]*v1 - 1.0L/3.0L*a[32]*v2 + (1.0L/3.0L)*a[33]*u1*v1 - 1.0L/3.0L*a[33]*u1*v2 +
            (1.0L/3.0L)*a[34]*u12*v1 - 1.0L/3.0L*a[34]*u12*v2 + (1.0L/3.0L)*a[35]*u13*v1 - 1.0L/3.0L*a[35]*u13*v2 +
            (1.0L/6.0L)*a[36]*v12 - 1.0L/6.0L*a[36]*v22 + (1.0L/6.0L)*a[37]*u1*v12 - 1.0L/6.0L*a[37]*u1*v22 +
            (1.0L/6.0L)*a[38]*u12*v12 - 1.0L/6.0L*a[38]*u12*v22 + (1.0L/6.0L)*a[39]*u13*v12 - 1.0L/6.0L*a[39]*u13*v22 +
            (1.0L/9.0L)*a[40]*v13 - 1.0L/9.0L*a[40]*v23 + (1.0L/9.0L)*a[41]*u1*v13 - 1.0L/9.0L*a[41]*u1*v23 +
            (1.0L/9.0L)*a[42]*u12*v13 - 1.0L/9.0L*a[42]*u12*v23 + (1.0L/9.0L)*a[43]*u13*v13 - 1.0L/9.0L*a[43]*u13*v23 +
            (1.0L/12.0L)*a[44]*v14 - 1.0L/12.0L*a[44]*v24 + (1.0L/12.0L)*a[45]*u1*v14 - 1.0L/12.0L*a[45]*u1*v24 +
            (1.0L/12.0L)*a[46]*u12*v14 - 1.0L/12.0L*a[46]*u12*v24 + (1.0L/12.0L)*a[47]*u13*v14 - 1.0L/12.0L*a[47]*u13*v24)
            + w22 *
            ((1.0L/2.0L)*a[16]*v1 - 1.0L/2.0L*a[16]*v2 + (1.0L/2.0L)*a[17]*u1*v1 - 1.0L/2.0L*a[17]*u1*v2 +
            (1.0L/2.0L)*a[18]*u12*v1 - 1.0L/2.0L*a[18]*u12*v2 + (1.0L/2.0L)*a[19]*u13*v1 - 1.0L/2.0L*a[19]*u13*v2 +
            (1.0L/4.0L)*a[20]*v12 - 1.0L/4.0L*a[20]*v22 + (1.0L/4.0L)*a[21]*u1*v12 - 1.0L/4.0L*a[21]*u1*v22 +
            (1.0L/4.0L)*a[22]*u12*v12 - 1.0L/4.0L*a[22]*u12*v22 + (1.0L/4.0L)*a[23]*u13*v12 - 1.0L/4.0L*a[23]*u13*v22 +
            (1.0L/6.0L)*a[24]*v13 - 1.0L/6.0L*a[24]*v23 + (1.0L/6.0L)*a[25]*u1*v13 - 1.0L/6.0L*a[25]*u1*v23 +
            (1.0L/6.0L)*a[26]*u12*v13 - 1.0L/6.0L*a[26]*u12*v23 + (1.0L/6.0L)*a[27]*u13*v13 - 1.0L/6.0L*a[27]*u13*v23 +
            (1.0L/8.0L)*a[28]*v14 - 1.0L/8.0L*a[28]*v24 + (1.0L/8.0L)*a[29]*u1*v14 - 1.0L/8.0L*a[29]*u1*v24 +
            (1.0L/8.0L)*a[30]*u12*v14 - 1.0L/8.0L*a[30]*u12*v24 + (1.0L/8.0L)*a[31]*u13*v14 - 1.0L/8.0L*a[31]*u13*v24)
            + w2 *
            (a[0]*v1 - a[0]*v2 + a[1]*u1*v1 - a[1]*u1*v2 +
            (1.0L/3.0L)*a[10]*u12*v13 - 1.0L/3.0L*a[10]*u12*v23 + (1.0L/3.0L)*a[11]*u13*v13 - 1.0L/3.0L*a[11]*u13*v23 +
            (1.0L/4.0L)*a[12]*v14 - 1.0L/4.0L*a[12]*v24 + (1.0L/4.0L)*a[13]*u1*v14 - 1.0L/4.0L*a[13]*u1*v24 +
            (1.0L/4.0L)*a[14]*u12*v14 - 1.0L/4.0L*a[14]*u12*v24 + (1.0L/4.0L)*a[15]*u13*v14 - 1.0L/4.0L*a[15]*u13*v24 +
            a[2]*u12*v1 - a[2]*u12*v2 + a[3]*u13*v1 - a[3]*u13*v2 +
            (1.0L/2.0L)*a[4]*v12 - 1.0L/2.0L*a[4]*v22 + (1.0L/2.0L)*a[5]*u1*v12 - 1.0L/2.0L*a[5]*u1*v22 +
            (1.0L/2.0L)*a[6]*u12*v12 - 1.0L/2.0L*a[6]*u12*v22 + (1.0L/2.0L)*a[7]*u13*v12 - 1.0L/2.0L*a[7]*u13*v22 +
            (1.0L/3.0L)*a[8]*v13 - 1.0L/3.0L*a[8]*v23 + (1.0L/3.0L)*a[9]*u1*v13 - 1.0L/3.0L*a[9]*u1*v23);
    return G_result;
}

double Energy::gradientYMinComponent(const gridreal*& a, double u1, double v1, double w1, double u2, double w2) {
    double u12 = u1*u1; double u13 = u12*u1; double u14 = u13*u1;
    double v12 = v1*v1; double v13 = v12*v1;
    double w12 = w1*w1; double w13 = w12*w1; double w14 = w13*w1;
    double u22 = u2*u2; double u23 = u22*u2; double u24 = u23*u2;
    double w22 = w2*w2; double w23 = w22*w2; double w24 = w23*w2;
    double G_result;
    G_result =
            -w14 *
            ((1.0L/4.0L)*a[48]*u1 - 1.0L/4.0L*a[48]*u2 + (1.0L/8.0L)*a[49]*u12 - 1.0L/8.0L*a[49]*u22 +
            (1.0L/12.0L)*a[50]*u13 - 1.0L/12.0L*a[50]*u23 + (1.0L/16.0L)*a[51]*u14 - 1.0L/16.0L*a[51]*u24 +
            (1.0L/4.0L)*a[52]*u1*v1 - 1.0L/4.0L*a[52]*u2*v1 + (1.0L/8.0L)*a[53]*u12*v1 - 1.0L/8.0L*a[53]*u22*v1 +
            (1.0L/12.0L)*a[54]*u13*v1 - 1.0L/12.0L*a[54]*u23*v1 + (1.0L/16.0L)*a[55]*u14*v1 - 1.0L/16.0L*a[55]*u24*v1 +
            (1.0L/4.0L)*a[56]*u1*v12 - 1.0L/4.0L*a[56]*u2*v12 + (1.0L/8.0L)*a[57]*u12*v12 - 1.0L/8.0L*a[57]*u22*v12 +
            (1.0L/12.0L)*a[58]*u13*v12 - 1.0L/12.0L*a[58]*u23*v12 + (1.0L/16.0L)*a[59]*u14*v12 - 1.0L/16.0L*a[59]*u24*v12 +
            (1.0L/4.0L)*a[60]*u1*v13 - 1.0L/4.0L*a[60]*u2*v13 + (1.0L/8.0L)*a[61]*u12*v13 - 1.0L/8.0L*a[61]*u22*v13 +
            (1.0L/12.0L)*a[62]*u13*v13 - 1.0L/12.0L*a[62]*u23*v13 + (1.0L/16.0L)*a[63]*u14*v13 - 1.0L/16.0L*a[63]*u24*v13)
            - w13 *
            ((1.0L/3.0L)*a[32]*u1 - 1.0L/3.0L*a[32]*u2 + (1.0L/6.0L)*a[33]*u12 - 1.0L/6.0L*a[33]*u22 +
            (1.0L/9.0L)*a[34]*u13 - 1.0L/9.0L*a[34]*u23 + (1.0L/12.0L)*a[35]*u14 - 1.0L/12.0L*a[35]*u24 +
            (1.0L/3.0L)*a[36]*u1*v1 - 1.0L/3.0L*a[36]*u2*v1 + (1.0L/6.0L)*a[37]*u12*v1 - 1.0L/6.0L*a[37]*u22*v1 +
            (1.0L/9.0L)*a[38]*u13*v1 - 1.0L/9.0L*a[38]*u23*v1 + (1.0L/12.0L)*a[39]*u14*v1 - 1.0L/12.0L*a[39]*u24*v1 +
            (1.0L/3.0L)*a[40]*u1*v12 - 1.0L/3.0L*a[40]*u2*v12 + (1.0L/6.0L)*a[41]*u12*v12 - 1.0L/6.0L*a[41]*u22*v12 +
            (1.0L/9.0L)*a[42]*u13*v12 - 1.0L/9.0L*a[42]*u23*v12 + (1.0L/12.0L)*a[43]*u14*v12 - 1.0L/12.0L*a[43]*u24*v12 +
            (1.0L/3.0L)*a[44]*u1*v13 - 1.0L/3.0L*a[44]*u2*v13 + (1.0L/6.0L)*a[45]*u12*v13 - 1.0L/6.0L*a[45]*u22*v13 +
            (1.0L/9.0L)*a[46]*u13*v13 - 1.0L/9.0L*a[46]*u23*v13 + (1.0L/12.0L)*a[47]*u14*v13 - 1.0L/12.0L*a[47]*u24*v13)
            - w12 *
            ((1.0L/2.0L)*a[16]*u1 - 1.0L/2.0L*a[16]*u2 + (1.0L/4.0L)*a[17]*u12 - 1.0L/4.0L*a[17]*u22 +
            (1.0L/6.0L)*a[18]*u13 - 1.0L/6.0L*a[18]*u23 + (1.0L/8.0L)*a[19]*u14 - 1.0L/8.0L*a[19]*u24 +
            (1.0L/2.0L)*a[20]*u1*v1 - 1.0L/2.0L*a[20]*u2*v1 + (1.0L/4.0L)*a[21]*u12*v1 - 1.0L/4.0L*a[21]*u22*v1 +
            (1.0L/6.0L)*a[22]*u13*v1 - 1.0L/6.0L*a[22]*u23*v1 + (1.0L/8.0L)*a[23]*u14*v1 - 1.0L/8.0L*a[23]*u24*v1 +
            (1.0L/2.0L)*a[24]*u1*v12 - 1.0L/2.0L*a[24]*u2*v12 + (1.0L/4.0L)*a[25]*u12*v12 - 1.0L/4.0L*a[25]*u22*v12 +
            (1.0L/6.0L)*a[26]*u13*v12 - 1.0L/6.0L*a[26]*u23*v12 + (1.0L/8.0L)*a[27]*u14*v12 - 1.0L/8.0L*a[27]*u24*v12 +
            (1.0L/2.0L)*a[28]*u1*v13 - 1.0L/2.0L*a[28]*u2*v13 + (1.0L/4.0L)*a[29]*u12*v13 - 1.0L/4.0L*a[29]*u22*v13 +
            (1.0L/6.0L)*a[30]*u13*v13 - 1.0L/6.0L*a[30]*u23*v13 + (1.0L/8.0L)*a[31]*u14*v13 - 1.0L/8.0L*a[31]*u24*v13)
            - w1 *
            (a[0]*u1 - a[0]*u2 + (1.0L/2.0L)*a[1]*u12 - 1.0L/2.0L*a[1]*u22 +
            (1.0L/3.0L)*a[10]*u13*v12 - 1.0L/3.0L*a[10]*u23*v12 + (1.0L/4.0L)*a[11]*u14*v12 - 1.0L/4.0L*a[11]*u24*v12 +
            a[12]*u1*v13 - a[12]*u2*v13 + (1.0L/2.0L)*a[13]*u12*v13 - 1.0L/2.0L*a[13]*u22*v13 +
            (1.0L/3.0L)*a[14]*u13*v13 - 1.0L/3.0L*a[14]*u23*v13 + (1.0L/4.0L)*a[15]*u14*v13 - 1.0L/4.0L*a[15]*u24*v13 +
            (1.0L/3.0L)*a[2]*u13 - 1.0L/3.0L*a[2]*u23 + (1.0L/4.0L)*a[3]*u14 - 1.0L/4.0L*a[3]*u24 +
            a[4]*u1*v1 - a[4]*u2*v1 + (1.0L/2.0L)*a[5]*u12*v1 - 1.0L/2.0L*a[5]*u22*v1 +
            (1.0L/3.0L)*a[6]*u13*v1 - 1.0L/3.0L*a[6]*u23*v1 + (1.0L/4.0L)*a[7]*u14*v1 - 1.0L/4.0L*a[7]*u24*v1 +
            a[8]*u1*v12 - a[8]*u2*v12 + (1.0L/2.0L)*a[9]*u12*v12 - 1.0L/2.0L*a[9]*u22*v12)
            + w24 *
            ((1.0L/4.0L)*a[48]*u1 - 1.0L/4.0L*a[48]*u2 + (1.0L/8.0L)*a[49]*u12 - 1.0L/8.0L*a[49]*u22 +
            (1.0L/12.0L)*a[50]*u13 - 1.0L/12.0L*a[50]*u23 + (1.0L/16.0L)*a[51]*u14 - 1.0L/16.0L*a[51]*u24 +
            (1.0L/4.0L)*a[52]*u1*v1 - 1.0L/4.0L*a[52]*u2*v1 + (1.0L/8.0L)*a[53]*u12*v1 - 1.0L/8.0L*a[53]*u22*v1 +
            (1.0L/12.0L)*a[54]*u13*v1 - 1.0L/12.0L*a[54]*u23*v1 + (1.0L/16.0L)*a[55]*u14*v1 - 1.0L/16.0L*a[55]*u24*v1 +
            (1.0L/4.0L)*a[56]*u1*v12 - 1.0L/4.0L*a[56]*u2*v12 + (1.0L/8.0L)*a[57]*u12*v12 - 1.0L/8.0L*a[57]*u22*v12 +
            (1.0L/12.0L)*a[58]*u13*v12 - 1.0L/12.0L*a[58]*u23*v12 + (1.0L/16.0L)*a[59]*u14*v12 - 1.0L/16.0L*a[59]*u24*v12 +
            (1.0L/4.0L)*a[60]*u1*v13 - 1.0L/4.0L*a[60]*u2*v13 + (1.0L/8.0L)*a[61]*u12*v13 - 1.0L/8.0L*a[61]*u22*v13 +
            (1.0L/12.0L)*a[62]*u13*v13 - 1.0L/12.0L*a[62]*u23*v13 + (1.0L/16.0L)*a[63]*u14*v13 - 1.0L/16.0L*a[63]*u24*v13)
            + w23 *
            ((1.0L/3.0L)*a[32]*u1 - 1.0L/3.0L*a[32]*u2 + (1.0L/6.0L)*a[33]*u12 - 1.0L/6.0L*a[33]*u22 +
            (1.0L/9.0L)*a[34]*u13 - 1.0L/9.0L*a[34]*u23 + (1.0L/12.0L)*a[35]*u14 - 1.0L/12.0L*a[35]*u24 +
            (1.0L/3.0L)*a[36]*u1*v1 - 1.0L/3.0L*a[36]*u2*v1 + (1.0L/6.0L)*a[37]*u12*v1 - 1.0L/6.0L*a[37]*u22*v1 +
            (1.0L/9.0L)*a[38]*u13*v1 - 1.0L/9.0L*a[38]*u23*v1 + (1.0L/12.0L)*a[39]*u14*v1 - 1.0L/12.0L*a[39]*u24*v1 +
            (1.0L/3.0L)*a[40]*u1*v12 - 1.0L/3.0L*a[40]*u2*v12 + (1.0L/6.0L)*a[41]*u12*v12 - 1.0L/6.0L*a[41]*u22*v12 +
            (1.0L/9.0L)*a[42]*u13*v12 - 1.0L/9.0L*a[42]*u23*v12 + (1.0L/12.0L)*a[43]*u14*v12 - 1.0L/12.0L*a[43]*u24*v12 +
            (1.0L/3.0L)*a[44]*u1*v13 - 1.0L/3.0L*a[44]*u2*v13 + (1.0L/6.0L)*a[45]*u12*v13 - 1.0L/6.0L*a[45]*u22*v13 +
            (1.0L/9.0L)*a[46]*u13*v13 - 1.0L/9.0L*a[46]*u23*v13 + (1.0L/12.0L)*a[47]*u14*v13 - 1.0L/12.0L*a[47]*u24*v13)
            + w22 *
            ((1.0L/2.0L)*a[16]*u1 - 1.0L/2.0L*a[16]*u2 + (1.0L/4.0L)*a[17]*u12 - 1.0L/4.0L*a[17]*u22 +
            (1.0L/6.0L)*a[18]*u13 - 1.0L/6.0L*a[18]*u23 + (1.0L/8.0L)*a[19]*u14 - 1.0L/8.0L*a[19]*u24 +
            (1.0L/2.0L)*a[20]*u1*v1 - 1.0L/2.0L*a[20]*u2*v1 + (1.0L/4.0L)*a[21]*u12*v1 - 1.0L/4.0L*a[21]*u22*v1 +
            (1.0L/6.0L)*a[22]*u13*v1 - 1.0L/6.0L*a[22]*u23*v1 + (1.0L/8.0L)*a[23]*u14*v1 - 1.0L/8.0L*a[23]*u24*v1 +
            (1.0L/2.0L)*a[24]*u1*v12 - 1.0L/2.0L*a[24]*u2*v12 + (1.0L/4.0L)*a[25]*u12*v12 - 1.0L/4.0L*a[25]*u22*v12 +
            (1.0L/6.0L)*a[26]*u13*v12 - 1.0L/6.0L*a[26]*u23*v12 + (1.0L/8.0L)*a[27]*u14*v12 - 1.0L/8.0L*a[27]*u24*v12 +
            (1.0L/2.0L)*a[28]*u1*v13 - 1.0L/2.0L*a[28]*u2*v13 + (1.0L/4.0L)*a[29]*u12*v13 - 1.0L/4.0L*a[29]*u22*v13 +
            (1.0L/6.0L)*a[30]*u13*v13 - 1.0L/6.0L*a[30]*u23*v13 + (1.0L/8.0L)*a[31]*u14*v13 - 1.0L/8.0L*a[31]*u24*v13)
            + w2 *
            (a[0]*u1 - a[0]*u2 + (1.0L/2.0L)*a[1]*u12 - 1.0L/2.0L*a[1]*u22 +
            (1.0L/3.0L)*a[10]*u13*v12 - 1.0L/3.0L*a[10]*u23*v12 + (1.0L/4.0L)*a[11]*u14*v12 - 1.0L/4.0L*a[11]*u24*v12 +
            a[12]*u1*v13 - a[12]*u2*v13 + (1.0L/2.0L)*a[13]*u12*v13 - 1.0L/2.0L*a[13]*u22*v13 +
            (1.0L/3.0L)*a[14]*u13*v13 - 1.0L/3.0L*a[14]*u23*v13 + (1.0L/4.0L)*a[15]*u14*v13 - 1.0L/4.0L*a[15]*u24*v13 +
            (1.0L/3.0L)*a[2]*u13 - 1.0L/3.0L*a[2]*u23 + (1.0L/4.0L)*a[3]*u14 - 1.0L/4.0L*a[3]*u24 +
            a[4]*u1*v1 - a[4]*u2*v1 + (1.0L/2.0L)*a[5]*u12*v1 - 1.0L/2.0L*a[5]*u22*v1 +
            (1.0L/3.0L)*a[6]*u13*v1 - 1.0L/3.0L*a[6]*u23*v1 + (1.0L/4.0L)*a[7]*u14*v1 - 1.0L/4.0L*a[7]*u24*v1 +
            a[8]*u1*v12 - a[8]*u2*v12 + (1.0L/2.0L)*a[9]*u12*v12 - 1.0L/2.0L*a[9]*u22*v12);
    return G_result;
}

double Energy::gradientZMinComponent(const gridreal*& a, double u1, double v1, double w1, double u2, double v2) {
    double u12 = u1*u1; double u13 = u12*u1; double u14 = u13*u1;
    double v12 = v1*v1; double v13 = v12*v1; double v14 = v13*v1;
    double w12 = w1*w1; double w13 = w12*w1;
    double u22 = u2*u2; double u23 = u22*u2; double u24 = u23*u2;
    double v22 = v2*v2; double v23 = v22*v2; double v24 = v23*v2;
    double G_result;
    G_result =
            -a[0]*u1*v1 + a[0]*u1*v2 + a[0]*u2*v1 - a[0]*u2*v2
            - 1.0L/2.0L*a[1]*u12*v1 +  (1.0L/2.0L)*a[1]*u12*v2 + (1.0L/2.0L)*a[1]*u22*v1 - 1.0L/2.0L*a[1]*u22*v2
            - 1.0L/9.0L*a[10]*u13*v13 + (1.0L/9.0L)*a[10]*u13*v23 + (1.0L/9.0L)*a[10]*u23*v13 - 1.0L/9.0L*a[10]*u23*v23
            - 1.0L/12.0L*a[11]*u14*v13 + (1.0L/12.0L)*a[11]*u14*v23 + (1.0L/12.0L)*a[11]*u24*v13 - 1.0L/12.0L*a[11]*u24*v23
            - 1.0L/4.0L*a[12]*u1*v14 + (1.0L/4.0L)*a[12]*u1*v24 + (1.0L/4.0L)*a[12]*u2*v14 - 1.0L/4.0L*a[12]*u2*v24
            - 1.0L/8.0L*a[13]*u12*v14 + (1.0L/8.0L)*a[13]*u12*v24 + (1.0L/8.0L)*a[13]*u22*v14 - 1.0L/8.0L*a[13]*u22*v24
            - 1.0L/12.0L*a[14]*u13*v14 + (1.0L/12.0L)*a[14]*u13*v24 + (1.0L/12.0L)*a[14]*u23*v14 - 1.0L/12.0L*a[14]*u23*v24
            - 1.0L/16.0L*a[15]*u14*v14 + (1.0L/16.0L)*a[15]*u14*v24 + (1.0L/16.0L)*a[15]*u24*v14 - 1.0L/16.0L*a[15]*u24*v24
            - 1.0L/3.0L*a[2]*u13*v1 + (1.0L/3.0L)*a[2]*u13*v2 + (1.0L/3.0L)*a[2]*u23*v1 - 1.0L/3.0L*a[2]*u23*v2
            - 1.0L/4.0L*a[3]*u14*v1 + (1.0L/4.0L)*a[3]*u14*v2 + (1.0L/4.0L)*a[3]*u24*v1 - 1.0L/4.0L*a[3]*u24*v2
            - 1.0L/2.0L*a[4]*u1*v12 + (1.0L/2.0L)*a[4]*u1*v22 + (1.0L/2.0L)*a[4]*u2*v12 - 1.0L/2.0L*a[4]*u2*v22
            - 1.0L/4.0L*a[5]*u12*v12 + (1.0L/4.0L)*a[5]*u12*v22 + (1.0L/4.0L)*a[5]*u22*v12 - 1.0L/4.0L*a[5]*u22*v22
            - 1.0L/6.0L*a[6]*u13*v12 + (1.0L/6.0L)*a[6]*u13*v22 + (1.0L/6.0L)*a[6]*u23*v12 - 1.0L/6.0L*a[6]*u23*v22
            - 1.0L/8.0L*a[7]*u14*v12 + (1.0L/8.0L)*a[7]*u14*v22 + (1.0L/8.0L)*a[7]*u24*v12 - 1.0L/8.0L*a[7]*u24*v22
            - 1.0L/3.0L*a[8]*u1*v13 + (1.0L/3.0L)*a[8]*u1*v23 + (1.0L/3.0L)*a[8]*u2*v13 - 1.0L/3.0L*a[8]*u2*v23
            - 1.0L/6.0L*a[9]*u12*v13 + (1.0L/6.0L)*a[9]*u12*v23 + (1.0L/6.0L)*a[9]*u22*v13 - 1.0L/6.0L*a[9]*u22*v23
            - 4*w13 *
            ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
            (1.0L/8.0L)*a[49]*u12*v1 - 1.0L/8.0L*a[49]*u12*v2 - 1.0L/8.0L*a[49]*u22*v1 + (1.0L/8.0L)*a[49]*u22*v2 +
            (1.0L/12.0L)*a[50]*u13*v1 - 1.0L/12.0L*a[50]*u13*v2 - 1.0L/12.0L*a[50]*u23*v1 + (1.0L/12.0L)*a[50]*u23*v2 +
            (1.0L/16.0L)*a[51]*u14*v1 - 1.0L/16.0L*a[51]*u14*v2 - 1.0L/16.0L*a[51]*u24*v1 + (1.0L/16.0L)*a[51]*u24*v2 +
            (1.0L/8.0L)*a[52]*u1*v12 - 1.0L/8.0L*a[52]*u1*v22 - 1.0L/8.0L*a[52]*u2*v12 + (1.0L/8.0L)*a[52]*u2*v22 +
            (1.0L/16.0L)*a[53]*u12*v12 - 1.0L/16.0L*a[53]*u12*v22 - 1.0L/16.0L*a[53]*u22*v12 + (1.0L/16.0L)*a[53]*u22*v22 +
            (1.0L/24.0L)*a[54]*u13*v12 - 1.0L/24.0L*a[54]*u13*v22 - 1.0L/24.0L*a[54]*u23*v12 + (1.0L/24.0L)*a[54]*u23*v22 +
            (1.0L/32.0L)*a[55]*u14*v12 - 1.0L/32.0L*a[55]*u14*v22 - 1.0L/32.0L*a[55]*u24*v12 + (1.0L/32.0L)*a[55]*u24*v22 +
            (1.0L/12.0L)*a[56]*u1*v13 - 1.0L/12.0L*a[56]*u1*v23 - 1.0L/12.0L*a[56]*u2*v13 + (1.0L/12.0L)*a[56]*u2*v23 +
            (1.0L/24.0L)*a[57]*u12*v13 - 1.0L/24.0L*a[57]*u12*v23 - 1.0L/24.0L*a[57]*u22*v13 + (1.0L/24.0L)*a[57]*u22*v23 +
            (1.0L/36.0L)*a[58]*u13*v13 - 1.0L/36.0L*a[58]*u13*v23 - 1.0L/36.0L*a[58]*u23*v13 + (1.0L/36.0L)*a[58]*u23*v23 +
            (1.0L/48.0L)*a[59]*u14*v13 - 1.0L/48.0L*a[59]*u14*v23 - 1.0L/48.0L*a[59]*u24*v13 + (1.0L/48.0L)*a[59]*u24*v23 +
            (1.0L/16.0L)*a[60]*u1*v14 - 1.0L/16.0L*a[60]*u1*v24 - 1.0L/16.0L*a[60]*u2*v14 + (1.0L/16.0L)*a[60]*u2*v24 +
            (1.0L/32.0L)*a[61]*u12*v14 - 1.0L/32.0L*a[61]*u12*v24 - 1.0L/32.0L*a[61]*u22*v14 + (1.0L/32.0L)*a[61]*u22*v24 +
            (1.0L/48.0L)*a[62]*u13*v14 - 1.0L/48.0L*a[62]*u13*v24 - 1.0L/48.0L*a[62]*u23*v14 + (1.0L/48.0L)*a[62]*u23*v24 +
            (1.0L/64.0L)*a[63]*u14*v14 - 1.0L/64.0L*a[63]*u14*v24 - 1.0L/64.0L*a[63]*u24*v14 + (1.0L/64.0L)*a[63]*u24*v24)
            - 3*w12 *
            ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
            (1.0L/6.0L)*a[33]*u12*v1 - 1.0L/6.0L*a[33]*u12*v2 - 1.0L/6.0L*a[33]*u22*v1 + (1.0L/6.0L)*a[33]*u22*v2 +
            (1.0L/9.0L)*a[34]*u13*v1 - 1.0L/9.0L*a[34]*u13*v2 - 1.0L/9.0L*a[34]*u23*v1 + (1.0L/9.0L)*a[34]*u23*v2 +
            (1.0L/12.0L)*a[35]*u14*v1 - 1.0L/12.0L*a[35]*u14*v2 - 1.0L/12.0L*a[35]*u24*v1 + (1.0L/12.0L)*a[35]*u24*v2 +
            (1.0L/6.0L)*a[36]*u1*v12 - 1.0L/6.0L*a[36]*u1*v22 - 1.0L/6.0L*a[36]*u2*v12 + (1.0L/6.0L)*a[36]*u2*v22 +
            (1.0L/12.0L)*a[37]*u12*v12 - 1.0L/12.0L*a[37]*u12*v22 - 1.0L/12.0L*a[37]*u22*v12 + (1.0L/12.0L)*a[37]*u22*v22 +
            (1.0L/18.0L)*a[38]*u13*v12 - 1.0L/18.0L*a[38]*u13*v22 - 1.0L/18.0L*a[38]*u23*v12 + (1.0L/18.0L)*a[38]*u23*v22 +
            (1.0L/24.0L)*a[39]*u14*v12 - 1.0L/24.0L*a[39]*u14*v22 - 1.0L/24.0L*a[39]*u24*v12 + (1.0L/24.0L)*a[39]*u24*v22 +
            (1.0L/9.0L)*a[40]*u1*v13 - 1.0L/9.0L*a[40]*u1*v23 - 1.0L/9.0L*a[40]*u2*v13 + (1.0L/9.0L)*a[40]*u2*v23 +
            (1.0L/18.0L)*a[41]*u12*v13 - 1.0L/18.0L*a[41]*u12*v23 - 1.0L/18.0L*a[41]*u22*v13 + (1.0L/18.0L)*a[41]*u22*v23 +
            (1.0L/27.0L)*a[42]*u13*v13 - 1.0L/27.0L*a[42]*u13*v23 - 1.0L/27.0L*a[42]*u23*v13 + (1.0L/27.0L)*a[42]*u23*v23 +
            (1.0L/36.0L)*a[43]*u14*v13 - 1.0L/36.0L*a[43]*u14*v23 - 1.0L/36.0L*a[43]*u24*v13 + (1.0L/36.0L)*a[43]*u24*v23 +
            (1.0L/12.0L)*a[44]*u1*v14 - 1.0L/12.0L*a[44]*u1*v24 - 1.0L/12.0L*a[44]*u2*v14 + (1.0L/12.0L)*a[44]*u2*v24 +
            (1.0L/24.0L)*a[45]*u12*v14 - 1.0L/24.0L*a[45]*u12*v24 - 1.0L/24.0L*a[45]*u22*v14 + (1.0L/24.0L)*a[45]*u22*v24 +
            (1.0L/36.0L)*a[46]*u13*v14 - 1.0L/36.0L*a[46]*u13*v24 - 1.0L/36.0L*a[46]*u23*v14 + (1.0L/36.0L)*a[46]*u23*v24 +
            (1.0L/48.0L)*a[47]*u14*v14 - 1.0L/48.0L*a[47]*u14*v24 - 1.0L/48.0L*a[47]*u24*v14 + (1.0L/48.0L)*a[47]*u24*v24)
            - 2*w1 *
            ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
            (1.0L/4.0L)*a[17]*u12*v1 - 1.0L/4.0L*a[17]*u12*v2 - 1.0L/4.0L*a[17]*u22*v1 + (1.0L/4.0L)*a[17]*u22*v2 +
            (1.0L/6.0L)*a[18]*u13*v1 - 1.0L/6.0L*a[18]*u13*v2 - 1.0L/6.0L*a[18]*u23*v1 + (1.0L/6.0L)*a[18]*u23*v2 +
            (1.0L/8.0L)*a[19]*u14*v1 - 1.0L/8.0L*a[19]*u14*v2 - 1.0L/8.0L*a[19]*u24*v1 + (1.0L/8.0L)*a[19]*u24*v2 +
            (1.0L/4.0L)*a[20]*u1*v12 - 1.0L/4.0L*a[20]*u1*v22 - 1.0L/4.0L*a[20]*u2*v12 + (1.0L/4.0L)*a[20]*u2*v22 +
            (1.0L/8.0L)*a[21]*u12*v12 - 1.0L/8.0L*a[21]*u12*v22 - 1.0L/8.0L*a[21]*u22*v12 + (1.0L/8.0L)*a[21]*u22*v22 +
            (1.0L/12.0L)*a[22]*u13*v12 - 1.0L/12.0L*a[22]*u13*v22 - 1.0L/12.0L*a[22]*u23*v12 + (1.0L/12.0L)*a[22]*u23*v22 +
            (1.0L/16.0L)*a[23]*u14*v12 - 1.0L/16.0L*a[23]*u14*v22 - 1.0L/16.0L*a[23]*u24*v12 + (1.0L/16.0L)*a[23]*u24*v22 +
            (1.0L/6.0L)*a[24]*u1*v13 - 1.0L/6.0L*a[24]*u1*v23 - 1.0L/6.0L*a[24]*u2*v13 + (1.0L/6.0L)*a[24]*u2*v23 +
            (1.0L/12.0L)*a[25]*u12*v13 - 1.0L/12.0L*a[25]*u12*v23 - 1.0L/12.0L*a[25]*u22*v13 + (1.0L/12.0L)*a[25]*u22*v23 +
            (1.0L/18.0L)*a[26]*u13*v13 - 1.0L/18.0L*a[26]*u13*v23 - 1.0L/18.0L*a[26]*u23*v13 + (1.0L/18.0L)*a[26]*u23*v23 +
            (1.0L/24.0L)*a[27]*u14*v13 - 1.0L/24.0L*a[27]*u14*v23 - 1.0L/24.0L*a[27]*u24*v13 + (1.0L/24.0L)*a[27]*u24*v23 +
            (1.0L/8.0L)*a[28]*u1*v14 - 1.0L/8.0L*a[28]*u1*v24 - 1.0L/8.0L*a[28]*u2*v14 + (1.0L/8.0L)*a[28]*u2*v24 +
            (1.0L/16.0L)*a[29]*u12*v14 - 1.0L/16.0L*a[29]*u12*v24 - 1.0L/16.0L*a[29]*u22*v14 + (1.0L/16.0L)*a[29]*u22*v24 +
            (1.0L/24.0L)*a[30]*u13*v14 - 1.0L/24.0L*a[30]*u13*v24 - 1.0L/24.0L*a[30]*u23*v14 + (1.0L/24.0L)*a[30]*u23*v24 +
            (1.0L/32.0L)*a[31]*u14*v14 - 1.0L/32.0L*a[31]*u14*v24 - 1.0L/32.0L*a[31]*u24*v14 + (1.0L/32.0L)*a[31]*u24*v24);
    return G_result;

}

double Energy::gradientXMaxComponent(const gridreal*& a, double v1, double w1, double u2, double v2, double w2) {
    double v12 = v1*v1; double v13 = v12*v1; double v14 = v13*v1;
    double w12 = w1*w1; double w13 = w12*w1; double w14 = w13*w1;
    double u22 = u2*u2; double u23 = u22*u2;
    double v22 = v2*v2; double v23 = v22*v2; double v24 = v23*v2;
    double w22 = w2*w2; double w23 = w22*w2; double w24 = w23*w2;
    double G_result;
    G_result =
            -w14 *
            (-1.0L/4.0L*a[48]*v1 + (1.0L/4.0L)*a[48]*v2 - 1.0L/4.0L*a[49]*u2*v1 + (1.0L/4.0L)*a[49]*u2*v2 -
            1.0L/4.0L*a[50]*u22*v1 + (1.0L/4.0L)*a[50]*u22*v2 - 1.0L/4.0L*a[51]*u23*v1 + (1.0L/4.0L)*a[51]*u23*v2 -
            1.0L/8.0L*a[52]*v12 + (1.0L/8.0L)*a[52]*v22 - 1.0L/8.0L*a[53]*u2*v12 + (1.0L/8.0L)*a[53]*u2*v22 -
            1.0L/8.0L*a[54]*u22*v12 + (1.0L/8.0L)*a[54]*u22*v22 - 1.0L/8.0L*a[55]*u23*v12 + (1.0L/8.0L)*a[55]*u23*v22 -
            1.0L/12.0L*a[56]*v13 + (1.0L/12.0L)*a[56]*v23 - 1.0L/12.0L*a[57]*u2*v13 + (1.0L/12.0L)*a[57]*u2*v23 -
            1.0L/12.0L*a[58]*u22*v13 + (1.0L/12.0L)*a[58]*u22*v23 - 1.0L/12.0L*a[59]*u23*v13 + (1.0L/12.0L)*a[59]*u23*v23 -
            1.0L/16.0L*a[60]*v14 + (1.0L/16.0L)*a[60]*v24 - 1.0L/16.0L*a[61]*u2*v14 + (1.0L/16.0L)*a[61]*u2*v24 -
            1.0L/16.0L*a[62]*u22*v14 + (1.0L/16.0L)*a[62]*u22*v24 - 1.0L/16.0L*a[63]*u23*v14 + (1.0L/16.0L)*a[63]*u23*v24)
            - w13 *
            (-1.0L/3.0L*a[32]*v1 + (1.0L/3.0L)*a[32]*v2 - 1.0L/3.0L*a[33]*u2*v1 + (1.0L/3.0L)*a[33]*u2*v2 -
            1.0L/3.0L*a[34]*u22*v1 + (1.0L/3.0L)*a[34]*u22*v2 - 1.0L/3.0L*a[35]*u23*v1 + (1.0L/3.0L)*a[35]*u23*v2 -
            1.0L/6.0L*a[36]*v12 + (1.0L/6.0L)*a[36]*v22 - 1.0L/6.0L*a[37]*u2*v12 + (1.0L/6.0L)*a[37]*u2*v22 -
            1.0L/6.0L*a[38]*u22*v12 + (1.0L/6.0L)*a[38]*u22*v22 - 1.0L/6.0L*a[39]*u23*v12 + (1.0L/6.0L)*a[39]*u23*v22 -
            1.0L/9.0L*a[40]*v13 + (1.0L/9.0L)*a[40]*v23 - 1.0L/9.0L*a[41]*u2*v13 + (1.0L/9.0L)*a[41]*u2*v23 -
            1.0L/9.0L*a[42]*u22*v13 + (1.0L/9.0L)*a[42]*u22*v23 - 1.0L/9.0L*a[43]*u23*v13 + (1.0L/9.0L)*a[43]*u23*v23 -
            1.0L/12.0L*a[44]*v14 + (1.0L/12.0L)*a[44]*v24 - 1.0L/12.0L*a[45]*u2*v14 + (1.0L/12.0L)*a[45]*u2*v24 -
            1.0L/12.0L*a[46]*u22*v14 + (1.0L/12.0L)*a[46]*u22*v24 - 1.0L/12.0L*a[47]*u23*v14 + (1.0L/12.0L)*a[47]*u23*v24)
            - w12 *
            (-1.0L/2.0L*a[16]*v1 + (1.0L/2.0L)*a[16]*v2 - 1.0L/2.0L*a[17]*u2*v1 + (1.0L/2.0L)*a[17]*u2*v2 -
            1.0L/2.0L*a[18]*u22*v1 + (1.0L/2.0L)*a[18]*u22*v2 - 1.0L/2.0L*a[19]*u23*v1 + (1.0L/2.0L)*a[19]*u23*v2 -
            1.0L/4.0L*a[20]*v12 + (1.0L/4.0L)*a[20]*v22 - 1.0L/4.0L*a[21]*u2*v12 + (1.0L/4.0L)*a[21]*u2*v22 -
            1.0L/4.0L*a[22]*u22*v12 + (1.0L/4.0L)*a[22]*u22*v22 - 1.0L/4.0L*a[23]*u23*v12 + (1.0L/4.0L)*a[23]*u23*v22 -
            1.0L/6.0L*a[24]*v13 + (1.0L/6.0L)*a[24]*v23 - 1.0L/6.0L*a[25]*u2*v13 + (1.0L/6.0L)*a[25]*u2*v23 -
            1.0L/6.0L*a[26]*u22*v13 + (1.0L/6.0L)*a[26]*u22*v23 - 1.0L/6.0L*a[27]*u23*v13 + (1.0L/6.0L)*a[27]*u23*v23 -
            1.0L/8.0L*a[28]*v14 + (1.0L/8.0L)*a[28]*v24 - 1.0L/8.0L*a[29]*u2*v14 + (1.0L/8.0L)*a[29]*u2*v24 -
            1.0L/8.0L*a[30]*u22*v14 + (1.0L/8.0L)*a[30]*u22*v24 - 1.0L/8.0L*a[31]*u23*v14 + (1.0L/8.0L)*a[31]*u23*v24)
            - w1 *
            (-a[0]*v1 + a[0]*v2 - a[1]*u2*v1 + a[1]*u2*v2 -
            1.0L/3.0L*a[10]*u22*v13 + (1.0L/3.0L)*a[10]*u22*v23 - 1.0L/3.0L*a[11]*u23*v13 + (1.0L/3.0L)*a[11]*u23*v23 -
            1.0L/4.0L*a[12]*v14 + (1.0L/4.0L)*a[12]*v24 - 1.0L/4.0L*a[13]*u2*v14 + (1.0L/4.0L)*a[13]*u2*v24 -
            1.0L/4.0L*a[14]*u22*v14 + (1.0L/4.0L)*a[14]*u22*v24 - 1.0L/4.0L*a[15]*u23*v14 + (1.0L/4.0L)*a[15]*u23*v24 -
            a[2]*u22*v1 + a[2]*u22*v2 - a[3]*u23*v1 + a[3]*u23*v2 -
            1.0L/2.0L*a[4]*v12 + (1.0L/2.0L)*a[4]*v22 - 1.0L/2.0L*a[5]*u2*v12 + (1.0L/2.0L)*a[5]*u2*v22 -
            1.0L/2.0L*a[6]*u22*v12 + (1.0L/2.0L)*a[6]*u22*v22 - 1.0L/2.0L*a[7]*u23*v12 + (1.0L/2.0L)*a[7]*u23*v22 -
            1.0L/3.0L*a[8]*v13 + (1.0L/3.0L)*a[8]*v23 - 1.0L/3.0L*a[9]*u2*v13 + (1.0L/3.0L)*a[9]*u2*v23)
            + w24 *
            (-1.0L/4.0L*a[48]*v1 + (1.0L/4.0L)*a[48]*v2 - 1.0L/4.0L*a[49]*u2*v1 + (1.0L/4.0L)*a[49]*u2*v2 -
            1.0L/4.0L*a[50]*u22*v1 + (1.0L/4.0L)*a[50]*u22*v2 - 1.0L/4.0L*a[51]*u23*v1 + (1.0L/4.0L)*a[51]*u23*v2 -
            1.0L/8.0L*a[52]*v12 + (1.0L/8.0L)*a[52]*v22 - 1.0L/8.0L*a[53]*u2*v12 + (1.0L/8.0L)*a[53]*u2*v22 -
            1.0L/8.0L*a[54]*u22*v12 + (1.0L/8.0L)*a[54]*u22*v22 - 1.0L/8.0L*a[55]*u23*v12 + (1.0L/8.0L)*a[55]*u23*v22 -
            1.0L/12.0L*a[56]*v13 + (1.0L/12.0L)*a[56]*v23 - 1.0L/12.0L*a[57]*u2*v13 + (1.0L/12.0L)*a[57]*u2*v23 -
            1.0L/12.0L*a[58]*u22*v13 + (1.0L/12.0L)*a[58]*u22*v23 - 1.0L/12.0L*a[59]*u23*v13 + (1.0L/12.0L)*a[59]*u23*v23 -
            1.0L/16.0L*a[60]*v14 + (1.0L/16.0L)*a[60]*v24 - 1.0L/16.0L*a[61]*u2*v14 + (1.0L/16.0L)*a[61]*u2*v24 -
            1.0L/16.0L*a[62]*u22*v14 + (1.0L/16.0L)*a[62]*u22*v24 - 1.0L/16.0L*a[63]*u23*v14 + (1.0L/16.0L)*a[63]*u23*v24)
            + w23 *
            (-1.0L/3.0L*a[32]*v1 + (1.0L/3.0L)*a[32]*v2 - 1.0L/3.0L*a[33]*u2*v1 + (1.0L/3.0L)*a[33]*u2*v2 -
            1.0L/3.0L*a[34]*u22*v1 + (1.0L/3.0L)*a[34]*u22*v2 - 1.0L/3.0L*a[35]*u23*v1 + (1.0L/3.0L)*a[35]*u23*v2 -
            1.0L/6.0L*a[36]*v12 + (1.0L/6.0L)*a[36]*v22 - 1.0L/6.0L*a[37]*u2*v12 + (1.0L/6.0L)*a[37]*u2*v22 -
            1.0L/6.0L*a[38]*u22*v12 + (1.0L/6.0L)*a[38]*u22*v22 - 1.0L/6.0L*a[39]*u23*v12 + (1.0L/6.0L)*a[39]*u23*v22 -
            1.0L/9.0L*a[40]*v13 + (1.0L/9.0L)*a[40]*v23 - 1.0L/9.0L*a[41]*u2*v13 + (1.0L/9.0L)*a[41]*u2*v23 -
            1.0L/9.0L*a[42]*u22*v13 + (1.0L/9.0L)*a[42]*u22*v23 - 1.0L/9.0L*a[43]*u23*v13 + (1.0L/9.0L)*a[43]*u23*v23 -
            1.0L/12.0L*a[44]*v14 + (1.0L/12.0L)*a[44]*v24 - 1.0L/12.0L*a[45]*u2*v14 + (1.0L/12.0L)*a[45]*u2*v24 -
            1.0L/12.0L*a[46]*u22*v14 + (1.0L/12.0L)*a[46]*u22*v24 - 1.0L/12.0L*a[47]*u23*v14 + (1.0L/12.0L)*a[47]*u23*v24)
            + w22 *
            (-1.0L/2.0L*a[16]*v1 + (1.0L/2.0L)*a[16]*v2 - 1.0L/2.0L*a[17]*u2*v1 + (1.0L/2.0L)*a[17]*u2*v2 -
            1.0L/2.0L*a[18]*u22*v1 + (1.0L/2.0L)*a[18]*u22*v2 - 1.0L/2.0L*a[19]*u23*v1 + (1.0L/2.0L)*a[19]*u23*v2 -
            1.0L/4.0L*a[20]*v12 + (1.0L/4.0L)*a[20]*v22 - 1.0L/4.0L*a[21]*u2*v12 + (1.0L/4.0L)*a[21]*u2*v22 -
            1.0L/4.0L*a[22]*u22*v12 + (1.0L/4.0L)*a[22]*u22*v22 - 1.0L/4.0L*a[23]*u23*v12 + (1.0L/4.0L)*a[23]*u23*v22 -
            1.0L/6.0L*a[24]*v13 + (1.0L/6.0L)*a[24]*v23 - 1.0L/6.0L*a[25]*u2*v13 + (1.0L/6.0L)*a[25]*u2*v23 -
            1.0L/6.0L*a[26]*u22*v13 + (1.0L/6.0L)*a[26]*u22*v23 - 1.0L/6.0L*a[27]*u23*v13 + (1.0L/6.0L)*a[27]*u23*v23 -
            1.0L/8.0L*a[28]*v14 + (1.0L/8.0L)*a[28]*v24 - 1.0L/8.0L*a[29]*u2*v14 + (1.0L/8.0L)*a[29]*u2*v24 -
            1.0L/8.0L*a[30]*u22*v14 + (1.0L/8.0L)*a[30]*u22*v24 - 1.0L/8.0L*a[31]*u23*v14 + (1.0L/8.0L)*a[31]*u23*v24)
            + w2 *
            (-a[0]*v1 + a[0]*v2 - a[1]*u2*v1 + a[1]*u2*v2 -
            1.0L/3.0L*a[10]*u22*v13 + (1.0L/3.0L)*a[10]*u22*v23 - 1.0L/3.0L*a[11]*u23*v13 + (1.0L/3.0L)*a[11]*u23*v23 -
            1.0L/4.0L*a[12]*v14 + (1.0L/4.0L)*a[12]*v24 - 1.0L/4.0L*a[13]*u2*v14 + (1.0L/4.0L)*a[13]*u2*v24 -
            1.0L/4.0L*a[14]*u22*v14 + (1.0L/4.0L)*a[14]*u22*v24 - 1.0L/4.0L*a[15]*u23*v14 + (1.0L/4.0L)*a[15]*u23*v24 -
            a[2]*u22*v1 + a[2]*u22*v2 - a[3]*u23*v1 + a[3]*u23*v2 -
            1.0L/2.0L*a[4]*v12 + (1.0L/2.0L)*a[4]*v22 - 1.0L/2.0L*a[5]*u2*v12 + (1.0L/2.0L)*a[5]*u2*v22 -
            1.0L/2.0L*a[6]*u22*v12 + (1.0L/2.0L)*a[6]*u22*v22 - 1.0L/2.0L*a[7]*u23*v12 + (1.0L/2.0L)*a[7]*u23*v22 -
            1.0L/3.0L*a[8]*v13 + (1.0L/3.0L)*a[8]*v23 - 1.0L/3.0L*a[9]*u2*v13 + (1.0L/3.0L)*a[9]*u2*v23);
    return G_result;
}

double Energy::gradientYMaxComponent(const gridreal*& a, double u1, double w1, double u2, double v2, double w2) {
    double u12 = u1*u1; double u13 = u12*u1; double u14 = u13*u1;
    double w12 = w1*w1; double w13 = w12*w1; double w14 = w13*w1;
    double u22 = u2*u2; double u23 = u22*u2; double u24 = u23*u2;
    double v22 = v2*v2; double v23 = v22*v2;
    double w22 = w2*w2; double w23 = w22*w2; double w24 = w23*w2;
    double G_result;
    G_result =
            -w14 *
            (-1.0L/4.0L*a[48]*u1 + (1.0L/4.0L)*a[48]*u2 - 1.0L/8.0L*a[49]*u12 + (1.0L/8.0L)*a[49]*u22 -
            1.0L/12.0L*a[50]*u13 + (1.0L/12.0L)*a[50]*u23 - 1.0L/16.0L*a[51]*u14 + (1.0L/16.0L)*a[51]*u24 -
            1.0L/4.0L*a[52]*u1*v2 + (1.0L/4.0L)*a[52]*u2*v2 - 1.0L/8.0L*a[53]*u12*v2 + (1.0L/8.0L)*a[53]*u22*v2 -
            1.0L/12.0L*a[54]*u13*v2 + (1.0L/12.0L)*a[54]*u23*v2 - 1.0L/16.0L*a[55]*u14*v2 + (1.0L/16.0L)*a[55]*u24*v2 -
            1.0L/4.0L*a[56]*u1*v22 + (1.0L/4.0L)*a[56]*u2*v22 - 1.0L/8.0L*a[57]*u12*v22 + (1.0L/8.0L)*a[57]*u22*v22 -
            1.0L/12.0L*a[58]*u13*v22 + (1.0L/12.0L)*a[58]*u23*v22 - 1.0L/16.0L*a[59]*u14*v22 + (1.0L/16.0L)*a[59]*u24*v22 -
            1.0L/4.0L*a[60]*u1*v23 + (1.0L/4.0L)*a[60]*u2*v23 - 1.0L/8.0L*a[61]*u12*v23 + (1.0L/8.0L)*a[61]*u22*v23 -
            1.0L/12.0L*a[62]*u13*v23 + (1.0L/12.0L)*a[62]*u23*v23 - 1.0L/16.0L*a[63]*u14*v23 + (1.0L/16.0L)*a[63]*u24*v23)
            - w13 *
            (-1.0L/3.0L*a[32]*u1 + (1.0L/3.0L)*a[32]*u2 - 1.0L/6.0L*a[33]*u12 + (1.0L/6.0L)*a[33]*u22 -
            1.0L/9.0L*a[34]*u13 + (1.0L/9.0L)*a[34]*u23 - 1.0L/12.0L*a[35]*u14 + (1.0L/12.0L)*a[35]*u24 -
            1.0L/3.0L*a[36]*u1*v2 + (1.0L/3.0L)*a[36]*u2*v2 - 1.0L/6.0L*a[37]*u12*v2 + (1.0L/6.0L)*a[37]*u22*v2 -
            1.0L/9.0L*a[38]*u13*v2 + (1.0L/9.0L)*a[38]*u23*v2 - 1.0L/12.0L*a[39]*u14*v2 + (1.0L/12.0L)*a[39]*u24*v2 -
            1.0L/3.0L*a[40]*u1*v22 + (1.0L/3.0L)*a[40]*u2*v22 - 1.0L/6.0L*a[41]*u12*v22 + (1.0L/6.0L)*a[41]*u22*v22 -
            1.0L/9.0L*a[42]*u13*v22 + (1.0L/9.0L)*a[42]*u23*v22 - 1.0L/12.0L*a[43]*u14*v22 + (1.0L/12.0L)*a[43]*u24*v22 -
            1.0L/3.0L*a[44]*u1*v23 + (1.0L/3.0L)*a[44]*u2*v23 - 1.0L/6.0L*a[45]*u12*v23 + (1.0L/6.0L)*a[45]*u22*v23 -
            1.0L/9.0L*a[46]*u13*v23 + (1.0L/9.0L)*a[46]*u23*v23 - 1.0L/12.0L*a[47]*u14*v23 + (1.0L/12.0L)*a[47]*u24*v23)
            - w12 *
            (-1.0L/2.0L*a[16]*u1 + (1.0L/2.0L)*a[16]*u2 - 1.0L/4.0L*a[17]*u12 + (1.0L/4.0L)*a[17]*u22 -
            1.0L/6.0L*a[18]*u13 + (1.0L/6.0L)*a[18]*u23 - 1.0L/8.0L*a[19]*u14 + (1.0L/8.0L)*a[19]*u24 -
            1.0L/2.0L*a[20]*u1*v2 + (1.0L/2.0L)*a[20]*u2*v2 - 1.0L/4.0L*a[21]*u12*v2 + (1.0L/4.0L)*a[21]*u22*v2 -
            1.0L/6.0L*a[22]*u13*v2 + (1.0L/6.0L)*a[22]*u23*v2 - 1.0L/8.0L*a[23]*u14*v2 + (1.0L/8.0L)*a[23]*u24*v2 -
            1.0L/2.0L*a[24]*u1*v22 + (1.0L/2.0L)*a[24]*u2*v22 - 1.0L/4.0L*a[25]*u12*v22 + (1.0L/4.0L)*a[25]*u22*v22 -
            1.0L/6.0L*a[26]*u13*v22 + (1.0L/6.0L)*a[26]*u23*v22 - 1.0L/8.0L*a[27]*u14*v22 + (1.0L/8.0L)*a[27]*u24*v22 -
            1.0L/2.0L*a[28]*u1*v23 + (1.0L/2.0L)*a[28]*u2*v23 - 1.0L/4.0L*a[29]*u12*v23 + (1.0L/4.0L)*a[29]*u22*v23 -
            1.0L/6.0L*a[30]*u13*v23 + (1.0L/6.0L)*a[30]*u23*v23 - 1.0L/8.0L*a[31]*u14*v23 + (1.0L/8.0L)*a[31]*u24*v23)
            - w1 *
            (-a[0]*u1 + a[0]*u2 - 1.0L/2.0L*a[1]*u12 + (1.0L/2.0L)*a[1]*u22 -
            1.0L/3.0L*a[10]*u13*v22 + (1.0L/3.0L)*a[10]*u23*v22 - 1.0L/4.0L*a[11]*u14*v22 + (1.0L/4.0L)*a[11]*u24*v22 -
            a[12]*u1*v23 + a[12]*u2*v23 - 1.0L/2.0L*a[13]*u12*v23 + (1.0L/2.0L)*a[13]*u22*v23 -
            1.0L/3.0L*a[14]*u13*v23 + (1.0L/3.0L)*a[14]*u23*v23 - 1.0L/4.0L*a[15]*u14*v23 + (1.0L/4.0L)*a[15]*u24*v23 -
            1.0L/3.0L*a[2]*u13 + (1.0L/3.0L)*a[2]*u23 - 1.0L/4.0L*a[3]*u14 + (1.0L/4.0L)*a[3]*u24 -
            a[4]*u1*v2 + a[4]*u2*v2 -  1.0L/2.0L*a[5]*u12*v2 + (1.0L/2.0L)*a[5]*u22*v2 -
            1.0L/3.0L*a[6]*u13*v2 + (1.0L/3.0L)*a[6]*u23*v2 - 1.0L/4.0L*a[7]*u14*v2 + (1.0L/4.0L)*a[7]*u24*v2 -
            a[8]*u1*v22 + a[8]*u2*v22 - 1.0L/2.0L*a[9]*u12*v22 + (1.0L/2.0L)*a[9]*u22*v22)
            + w24 *
            (-1.0L/4.0L*a[48]*u1 + (1.0L/4.0L)*a[48]*u2 - 1.0L/8.0L*a[49]*u12 + (1.0L/8.0L)*a[49]*u22 -
            1.0L/12.0L*a[50]*u13 + (1.0L/12.0L)*a[50]*u23 - 1.0L/16.0L*a[51]*u14 + (1.0L/16.0L)*a[51]*u24 -
            1.0L/4.0L*a[52]*u1*v2 + (1.0L/4.0L)*a[52]*u2*v2 - 1.0L/8.0L*a[53]*u12*v2 + (1.0L/8.0L)*a[53]*u22*v2 -
            1.0L/12.0L*a[54]*u13*v2 + (1.0L/12.0L)*a[54]*u23*v2 - 1.0L/16.0L*a[55]*u14*v2 + (1.0L/16.0L)*a[55]*u24*v2 -
            1.0L/4.0L*a[56]*u1*v22 + (1.0L/4.0L)*a[56]*u2*v22 - 1.0L/8.0L*a[57]*u12*v22 + (1.0L/8.0L)*a[57]*u22*v22 -
            1.0L/12.0L*a[58]*u13*v22 + (1.0L/12.0L)*a[58]*u23*v22 - 1.0L/16.0L*a[59]*u14*v22 + (1.0L/16.0L)*a[59]*u24*v22 -
            1.0L/4.0L*a[60]*u1*v23 + (1.0L/4.0L)*a[60]*u2*v23 - 1.0L/8.0L*a[61]*u12*v23 + (1.0L/8.0L)*a[61]*u22*v23 -
            1.0L/12.0L*a[62]*u13*v23 + (1.0L/12.0L)*a[62]*u23*v23 - 1.0L/16.0L*a[63]*u14*v23 + (1.0L/16.0L)*a[63]*u24*v23)
            + w23 *
            (-1.0L/3.0L*a[32]*u1 + (1.0L/3.0L)*a[32]*u2 - 1.0L/6.0L*a[33]*u12 + (1.0L/6.0L)*a[33]*u22 -
            1.0L/9.0L*a[34]*u13 + (1.0L/9.0L)*a[34]*u23 - 1.0L/12.0L*a[35]*u14 + (1.0L/12.0L)*a[35]*u24 -
            1.0L/3.0L*a[36]*u1*v2 + (1.0L/3.0L)*a[36]*u2*v2 - 1.0L/6.0L*a[37]*u12*v2 + (1.0L/6.0L)*a[37]*u22*v2 -
            1.0L/9.0L*a[38]*u13*v2 + (1.0L/9.0L)*a[38]*u23*v2 - 1.0L/12.0L*a[39]*u14*v2 + (1.0L/12.0L)*a[39]*u24*v2 -
            1.0L/3.0L*a[40]*u1*v22 + (1.0L/3.0L)*a[40]*u2*v22 - 1.0L/6.0L*a[41]*u12*v22 + (1.0L/6.0L)*a[41]*u22*v22 -
            1.0L/9.0L*a[42]*u13*v22 + (1.0L/9.0L)*a[42]*u23*v22 - 1.0L/12.0L*a[43]*u14*v22 + (1.0L/12.0L)*a[43]*u24*v22 -
            1.0L/3.0L*a[44]*u1*v23 + (1.0L/3.0L)*a[44]*u2*v23 - 1.0L/6.0L*a[45]*u12*v23 + (1.0L/6.0L)*a[45]*u22*v23 -
            1.0L/9.0L*a[46]*u13*v23 + (1.0L/9.0L)*a[46]*u23*v23 - 1.0L/12.0L*a[47]*u14*v23 + (1.0L/12.0L)*a[47]*u24*v23)
            + w22 *
            (-1.0L/2.0L*a[16]*u1 + (1.0L/2.0L)*a[16]*u2 - 1.0L/4.0L*a[17]*u12 + (1.0L/4.0L)*a[17]*u22 -
            1.0L/6.0L*a[18]*u13 + (1.0L/6.0L)*a[18]*u23 - 1.0L/8.0L*a[19]*u14 + (1.0L/8.0L)*a[19]*u24 -
            1.0L/2.0L*a[20]*u1*v2 + (1.0L/2.0L)*a[20]*u2*v2 - 1.0L/4.0L*a[21]*u12*v2 + (1.0L/4.0L)*a[21]*u22*v2 -
            1.0L/6.0L*a[22]*u13*v2 + (1.0L/6.0L)*a[22]*u23*v2 - 1.0L/8.0L*a[23]*u14*v2 + (1.0L/8.0L)*a[23]*u24*v2 -
            1.0L/2.0L*a[24]*u1*v22 + (1.0L/2.0L)*a[24]*u2*v22 - 1.0L/4.0L*a[25]*u12*v22 + (1.0L/4.0L)*a[25]*u22*v22 -
            1.0L/6.0L*a[26]*u13*v22 + (1.0L/6.0L)*a[26]*u23*v22 - 1.0L/8.0L*a[27]*u14*v22 + (1.0L/8.0L)*a[27]*u24*v22 -
            1.0L/2.0L*a[28]*u1*v23 + (1.0L/2.0L)*a[28]*u2*v23 - 1.0L/4.0L*a[29]*u12*v23 + (1.0L/4.0L)*a[29]*u22*v23 -
            1.0L/6.0L*a[30]*u13*v23 + (1.0L/6.0L)*a[30]*u23*v23 - 1.0L/8.0L*a[31]*u14*v23 + (1.0L/8.0L)*a[31]*u24*v23)
            + w2 *
            (-a[0]*u1 + a[0]*u2 - 1.0L/2.0L*a[1]*u12 + (1.0L/2.0L)*a[1]*u22 -
            1.0L/3.0L*a[10]*u13*v22 + (1.0L/3.0L)*a[10]*u23*v22 - 1.0L/4.0L*a[11]*u14*v22 + (1.0L/4.0L)*a[11]*u24*v22 -
            a[12]*u1*v23 + a[12]*u2*v23 - 1.0L/2.0L*a[13]*u12*v23 + (1.0L/2.0L)*a[13]*u22*v23 -
            1.0L/3.0L*a[14]*u13*v23 + (1.0L/3.0L)*a[14]*u23*v23 - 1.0L/4.0L*a[15]*u14*v23 + (1.0L/4.0L)*a[15]*u24*v23 -
            1.0L/3.0L*a[2]*u13 + (1.0L/3.0L)*a[2]*u23 - 1.0L/4.0L*a[3]*u14 + (1.0L/4.0L)*a[3]*u24
            - a[4]*u1*v2 + a[4]*u2*v2 - 1.0L/2.0L*a[5]*u12*v2 + (1.0L/2.0L)*a[5]*u22*v2 -
            1.0L/3.0L*a[6]*u13*v2 + (1.0L/3.0L)*a[6]*u23*v2 - 1.0L/4.0L*a[7]*u14*v2 + (1.0L/4.0L)*a[7]*u24*v2 -
            a[8]*u1*v22 + a[8]*u2*v22 - 1.0L/2.0L*a[9]*u12*v22 + (1.0L/2.0L)*a[9]*u22*v22);
    return G_result;
}

double Energy::gradientZMaxComponent(const gridreal*& a, double u1, double v1, double u2, double v2, double w2) {
    double u12 = u1*u1; double u13 = u12*u1; double u14 = u13*u1;
    double v12 = v1*v1; double v13 = v12*v1; double v14 = v13*v1;
    double u22 = u2*u2; double u23 = u22*u2; double u24 = u23*u2;
    double v22 = v2*v2; double v23 = v22*v2; double v24 = v23*v2;
    double w22 = w2*w2; double w23 = w22*w2;
    double G_result;
    G_result =
            a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 +
            (1.0L/2.0L)*a[1]*u12*v1 - 1.0L/2.0L*a[1]*u12*v2 - 1.0L/2.0L*a[1]*u22*v1 + (1.0L/2.0L)*a[1]*u22*v2 +
            (1.0L/9.0L)*a[10]*u13*v13 - 1.0L/9.0L*a[10]*u13*v23 - 1.0L/9.0L*a[10]*u23*v13 + (1.0L/9.0L)*a[10]*u23*v23 +
            (1.0L/12.0L)*a[11]*u14*v13 - 1.0L/12.0L*a[11]*u14*v23 - 1.0L/12.0L*a[11]*u24*v13 + (1.0L/12.0L)*a[11]*u24*v23 +
            (1.0L/4.0L)*a[12]*u1*v14 - 1.0L/4.0L*a[12]*u1*v24 - 1.0L/4.0L*a[12]*u2*v14 + (1.0L/4.0L)*a[12]*u2*v24 +
            (1.0L/8.0L)*a[13]*u12*v14 - 1.0L/8.0L*a[13]*u12*v24 - 1.0L/8.0L*a[13]*u22*v14 + (1.0L/8.0L)*a[13]*u22*v24 +
            (1.0L/12.0L)*a[14]*u13*v14 - 1.0L/12.0L*a[14]*u13*v24 - 1.0L/12.0L*a[14]*u23*v14 + (1.0L/12.0L)*a[14]*u23*v24 +
            (1.0L/16.0L)*a[15]*u14*v14 - 1.0L/16.0L*a[15]*u14*v24 - 1.0L/16.0L*a[15]*u24*v14 + (1.0L/16.0L)*a[15]*u24*v24 +
            (1.0L/3.0L)*a[2]*u13*v1 - 1.0L/3.0L*a[2]*u13*v2 - 1.0L/3.0L*a[2]*u23*v1 + (1.0L/3.0L)*a[2]*u23*v2 +
            (1.0L/4.0L)*a[3]*u14*v1 - 1.0L/4.0L*a[3]*u14*v2 - 1.0L/4.0L*a[3]*u24*v1 + (1.0L/4.0L)*a[3]*u24*v2 +
            (1.0L/2.0L)*a[4]*u1*v12 - 1.0L/2.0L*a[4]*u1*v22 - 1.0L/2.0L*a[4]*u2*v12 + (1.0L/2.0L)*a[4]*u2*v22 +
            (1.0L/4.0L)*a[5]*u12*v12 - 1.0L/4.0L*a[5]*u12*v22 - 1.0L/4.0L*a[5]*u22*v12 + (1.0L/4.0L)*a[5]*u22*v22 +
            (1.0L/6.0L)*a[6]*u13*v12 - 1.0L/6.0L*a[6]*u13*v22 - 1.0L/6.0L*a[6]*u23*v12 + (1.0L/6.0L)*a[6]*u23*v22 +
            (1.0L/8.0L)*a[7]*u14*v12 - 1.0L/8.0L*a[7]*u14*v22 - 1.0L/8.0L*a[7]*u24*v12 + (1.0L/8.0L)*a[7]*u24*v22 +
            (1.0L/3.0L)*a[8]*u1*v13 - 1.0L/3.0L*a[8]*u1*v23 - 1.0L/3.0L*a[8]*u2*v13 + (1.0L/3.0L)*a[8]*u2*v23 +
            (1.0L/6.0L)*a[9]*u12*v13 - 1.0L/6.0L*a[9]*u12*v23 - 1.0L/6.0L*a[9]*u22*v13 + (1.0L/6.0L)*a[9]*u22*v23 +
            4*w23 *
            ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
            (1.0L/8.0L)*a[49]*u12*v1 - 1.0L/8.0L*a[49]*u12*v2 - 1.0L/8.0L*a[49]*u22*v1 + (1.0L/8.0L)*a[49]*u22*v2 +
            (1.0L/12.0L)*a[50]*u13*v1 - 1.0L/12.0L*a[50]*u13*v2 - 1.0L/12.0L*a[50]*u23*v1 + (1.0L/12.0L)*a[50]*u23*v2 +
            (1.0L/16.0L)*a[51]*u14*v1 - 1.0L/16.0L*a[51]*u14*v2 - 1.0L/16.0L*a[51]*u24*v1 + (1.0L/16.0L)*a[51]*u24*v2 +
            (1.0L/8.0L)*a[52]*u1*v12 - 1.0L/8.0L*a[52]*u1*v22 - 1.0L/8.0L*a[52]*u2*v12 + (1.0L/8.0L)*a[52]*u2*v22 +
            (1.0L/16.0L)*a[53]*u12*v12 - 1.0L/16.0L*a[53]*u12*v22 - 1.0L/16.0L*a[53]*u22*v12 + (1.0L/16.0L)*a[53]*u22*v22 +
            (1.0L/24.0L)*a[54]*u13*v12 - 1.0L/24.0L*a[54]*u13*v22 - 1.0L/24.0L*a[54]*u23*v12 + (1.0L/24.0L)*a[54]*u23*v22 +
            (1.0L/32.0L)*a[55]*u14*v12 - 1.0L/32.0L*a[55]*u14*v22 - 1.0L/32.0L*a[55]*u24*v12 + (1.0L/32.0L)*a[55]*u24*v22 +
            (1.0L/12.0L)*a[56]*u1*v13 - 1.0L/12.0L*a[56]*u1*v23 - 1.0L/12.0L*a[56]*u2*v13 + (1.0L/12.0L)*a[56]*u2*v23 +
            (1.0L/24.0L)*a[57]*u12*v13 - 1.0L/24.0L*a[57]*u12*v23 - 1.0L/24.0L*a[57]*u22*v13 + (1.0L/24.0L)*a[57]*u22*v23 +
            (1.0L/36.0L)*a[58]*u13*v13 - 1.0L/36.0L*a[58]*u13*v23 - 1.0L/36.0L*a[58]*u23*v13 + (1.0L/36.0L)*a[58]*u23*v23 +
            (1.0L/48.0L)*a[59]*u14*v13 - 1.0L/48.0L*a[59]*u14*v23 - 1.0L/48.0L*a[59]*u24*v13 + (1.0L/48.0L)*a[59]*u24*v23 +
            (1.0L/16.0L)*a[60]*u1*v14 - 1.0L/16.0L*a[60]*u1*v24 - 1.0L/16.0L*a[60]*u2*v14 + (1.0L/16.0L)*a[60]*u2*v24 +
            (1.0L/32.0L)*a[61]*u12*v14 - 1.0L/32.0L*a[61]*u12*v24 - 1.0L/32.0L*a[61]*u22*v14 + (1.0L/32.0L)*a[61]*u22*v24 +
            (1.0L/48.0L)*a[62]*u13*v14 - 1.0L/48.0L*a[62]*u13*v24 - 1.0L/48.0L*a[62]*u23*v14 + (1.0L/48.0L)*a[62]*u23*v24 +
            (1.0L/64.0L)*a[63]*u14*v14 - 1.0L/64.0L*a[63]*u14*v24 - 1.0L/64.0L*a[63]*u24*v14 + (1.0L/64.0L)*a[63]*u24*v24)
            + 3*w22 *
            ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
            (1.0L/6.0L)*a[33]*u12*v1 - 1.0L/6.0L*a[33]*u12*v2 - 1.0L/6.0L*a[33]*u22*v1 + (1.0L/6.0L)*a[33]*u22*v2 +
            (1.0L/9.0L)*a[34]*u13*v1 - 1.0L/9.0L*a[34]*u13*v2 - 1.0L/9.0L*a[34]*u23*v1 + (1.0L/9.0L)*a[34]*u23*v2 +
            (1.0L/12.0L)*a[35]*u14*v1 - 1.0L/12.0L*a[35]*u14*v2 - 1.0L/12.0L*a[35]*u24*v1 + (1.0L/12.0L)*a[35]*u24*v2 +
            (1.0L/6.0L)*a[36]*u1*v12 - 1.0L/6.0L*a[36]*u1*v22 - 1.0L/6.0L*a[36]*u2*v12 + (1.0L/6.0L)*a[36]*u2*v22 +
            (1.0L/12.0L)*a[37]*u12*v12 - 1.0L/12.0L*a[37]*u12*v22 - 1.0L/12.0L*a[37]*u22*v12 + (1.0L/12.0L)*a[37]*u22*v22 +
            (1.0L/18.0L)*a[38]*u13*v12 - 1.0L/18.0L*a[38]*u13*v22 - 1.0L/18.0L*a[38]*u23*v12 + (1.0L/18.0L)*a[38]*u23*v22 +
            (1.0L/24.0L)*a[39]*u14*v12 - 1.0L/24.0L*a[39]*u14*v22 - 1.0L/24.0L*a[39]*u24*v12 + (1.0L/24.0L)*a[39]*u24*v22 +
            (1.0L/9.0L)*a[40]*u1*v13 - 1.0L/9.0L*a[40]*u1*v23 - 1.0L/9.0L*a[40]*u2*v13 + (1.0L/9.0L)*a[40]*u2*v23 +
            (1.0L/18.0L)*a[41]*u12*v13 - 1.0L/18.0L*a[41]*u12*v23 - 1.0L/18.0L*a[41]*u22*v13 + (1.0L/18.0L)*a[41]*u22*v23 +
            (1.0L/27.0L)*a[42]*u13*v13 - 1.0L/27.0L*a[42]*u13*v23 - 1.0L/27.0L*a[42]*u23*v13 + (1.0L/27.0L)*a[42]*u23*v23 +
            (1.0L/36.0L)*a[43]*u14*v13 - 1.0L/36.0L*a[43]*u14*v23 - 1.0L/36.0L*a[43]*u24*v13 + (1.0L/36.0L)*a[43]*u24*v23 +
            (1.0L/12.0L)*a[44]*u1*v14 - 1.0L/12.0L*a[44]*u1*v24 - 1.0L/12.0L*a[44]*u2*v14 + (1.0L/12.0L)*a[44]*u2*v24 +
            (1.0L/24.0L)*a[45]*u12*v14 - 1.0L/24.0L*a[45]*u12*v24 - 1.0L/24.0L*a[45]*u22*v14 + (1.0L/24.0L)*a[45]*u22*v24 +
            (1.0L/36.0L)*a[46]*u13*v14 - 1.0L/36.0L*a[46]*u13*v24 - 1.0L/36.0L*a[46]*u23*v14 + (1.0L/36.0L)*a[46]*u23*v24 +
            (1.0L/48.0L)*a[47]*u14*v14 - 1.0L/48.0L*a[47]*u14*v24 - 1.0L/48.0L*a[47]*u24*v14 + (1.0L/48.0L)*a[47]*u24*v24)
            + 2*w2 *
            ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
            (1.0L/4.0L)*a[17]*u12*v1 - 1.0L/4.0L*a[17]*u12*v2 - 1.0L/4.0L*a[17]*u22*v1 + (1.0L/4.0L)*a[17]*u22*v2 +
            (1.0L/6.0L)*a[18]*u13*v1 - 1.0L/6.0L*a[18]*u13*v2 - 1.0L/6.0L*a[18]*u23*v1 + (1.0L/6.0L)*a[18]*u23*v2 +
            (1.0L/8.0L)*a[19]*u14*v1 - 1.0L/8.0L*a[19]*u14*v2 - 1.0L/8.0L*a[19]*u24*v1 + (1.0L/8.0L)*a[19]*u24*v2 +
            (1.0L/4.0L)*a[20]*u1*v12 - 1.0L/4.0L*a[20]*u1*v22 - 1.0L/4.0L*a[20]*u2*v12 + (1.0L/4.0L)*a[20]*u2*v22 +
            (1.0L/8.0L)*a[21]*u12*v12 - 1.0L/8.0L*a[21]*u12*v22 - 1.0L/8.0L*a[21]*u22*v12 + (1.0L/8.0L)*a[21]*u22*v22 +
            (1.0L/12.0L)*a[22]*u13*v12 - 1.0L/12.0L*a[22]*u13*v22 - 1.0L/12.0L*a[22]*u23*v12 + (1.0L/12.0L)*a[22]*u23*v22 +
            (1.0L/16.0L)*a[23]*u14*v12 - 1.0L/16.0L*a[23]*u14*v22 - 1.0L/16.0L*a[23]*u24*v12 + (1.0L/16.0L)*a[23]*u24*v22 +
            (1.0L/6.0L)*a[24]*u1*v13 - 1.0L/6.0L*a[24]*u1*v23 - 1.0L/6.0L*a[24]*u2*v13 + (1.0L/6.0L)*a[24]*u2*v23 +
            (1.0L/12.0L)*a[25]*u12*v13 - 1.0L/12.0L*a[25]*u12*v23 - 1.0L/12.0L*a[25]*u22*v13 + (1.0L/12.0L)*a[25]*u22*v23 +
            (1.0L/18.0L)*a[26]*u13*v13 - 1.0L/18.0L*a[26]*u13*v23 - 1.0L/18.0L*a[26]*u23*v13 + (1.0L/18.0L)*a[26]*u23*v23 +
            (1.0L/24.0L)*a[27]*u14*v13 - 1.0L/24.0L*a[27]*u14*v23 - 1.0L/24.0L*a[27]*u24*v13 + (1.0L/24.0L)*a[27]*u24*v23 +
            (1.0L/8.0L)*a[28]*u1*v14 - 1.0L/8.0L*a[28]*u1*v24 - 1.0L/8.0L*a[28]*u2*v14 + (1.0L/8.0L)*a[28]*u2*v24 +
            (1.0L/16.0L)*a[29]*u12*v14 - 1.0L/16.0L*a[29]*u12*v24 - 1.0L/16.0L*a[29]*u22*v14 + (1.0L/16.0L)*a[29]*u22*v24 +
            (1.0L/24.0L)*a[30]*u13*v14 - 1.0L/24.0L*a[30]*u13*v24 - 1.0L/24.0L*a[30]*u23*v14 + (1.0L/24.0L)*a[30]*u23*v24 +
            (1.0L/32.0L)*a[31]*u14*v14 - 1.0L/32.0L*a[31]*u14*v24 - 1.0L/32.0L*a[31]*u24*v14 + (1.0L/32.0L)*a[31]*u24*v24);
    return G_result;
}

double Energy::gradientEvaluateXMinComponent(const Eigen::VectorXd &x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;
    //fissare x1
    x1 = firstX;
    while (x1 <= minbx){
        x1+=unit;
    }
    double x2 = x1;
    x1-=unit;
    for (y1 = firstY; y1 <= lastY; y1+=unit){
        double y2 = y1 + unit;
        for (z1 = firstZ; z1<=lastZ; z1+=unit){
            double z2 = z1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                maxby <= y1 || minby >= y2 ||
                maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double u1 = minbx < x1 ? x1 : minbx;
                double v1 = minby < y1 ? y1 : minby;
                double w1 = minbz < z1 ? z1 : minbz;
                double v2 = maxby > y2 ? y2 : maxby;
                double w2 = maxbz > z2 ? z2 : maxbz;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u1 = (u1-x1)/unit;
                v1 = (v1-y1)/unit;
                v2 = (v2-y1)/unit;
                w1 = (w1-z1)/unit;
                w2 = (w2-z1)/unit;
                energy += gradientXMinComponent(coeffs, u1,v1,w1,v2,w2);
            }

        }
    }

    return energy;
}

double Energy::gradientEvaluateYMinComponent(const Eigen::VectorXd &x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;

    //fissare y1
    y1 = firstY;
    while (y1 <= minby){
        y1+=unit;
    }
    double y2 = y1;
    y1-=unit;

    for (x1 = firstX; x1 <= lastX; x1+=unit){
        double x2 = x1 + unit;
        for (z1 = firstZ; z1<=lastZ; z1+=unit){
            double z2 = z1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                    maxby <= y1 || minby >= y2 ||
                    maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double u1 = minbx < x1 ? x1 : minbx;
                double v1 = minby < y1 ? y1 : minby;
                double w1 = minbz < z1 ? z1 : minbz;
                double u2 = maxbx > x2 ? x2 : maxbx;
                double w2 = maxbz > z2 ? z2 : maxbz;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u1 = (u1-x1)/unit;
                u2 = (u2-x1)/unit;
                v1 = (v1-y1)/unit;
                w1 = (w1-z1)/unit;
                w2 = (w2-z1)/unit;
                energy += gradientYMinComponent(coeffs, u1,v1,w1,u2,w2);
            }
        }
    }

    return energy;
}

double Energy::gradientEvaluateZMinComponent(const Eigen::VectorXd& x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;

    //fissare z1
    z1 = firstZ;
    while (z1 <= minbz){
        z1+=unit;
    }
    double z2 = z1;
    z1-=unit;

    for (x1 = firstX; x1 <= lastX; x1+=unit){
        double x2 = x1 + unit;
        for (y1 = firstY; y1<=lastY; y1+=unit){
            double y2 = y1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                    maxby <= y1 || minby >= y2 ||
                    maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double u1 = minbx < x1 ? x1 : minbx;
                double v1 = minby < y1 ? y1 : minby;
                double w1 = minbz < z1 ? z1 : minbz;
                double u2 = maxbx > x2 ? x2 : maxbx;
                double v2 = maxby > y2 ? y2 : maxby;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u1 = (u1-x1)/unit;
                u2 = (u2-x1)/unit;
                v1 = (v1-y1)/unit;
                v2 = (v2-y1)/unit;
                w1 = (w1-z1)/unit;
                energy += gradientZMinComponent(coeffs, u1,v1,w1,u2,v2);
            }
        }
    }

    return energy;
}

double Energy::gradientEvaluateXMaxComponent(const Eigen::VectorXd &x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;
    //fissare x1
    x1 = lastX;
    while (x1 >= maxbx){
        x1-=unit;
    }
    double x2 = x1+unit;

    for (y1 = firstY; y1 <= lastY; y1+=unit){
        double y2 = y1 + unit;
        for (z1 = firstZ; z1<=lastZ; z1+=unit){
            double z2 = z1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                maxby <= y1 || minby >= y2 ||
                maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double v1 = minby < y1 ? y1 : minby;
                double w1 = minbz < z1 ? z1 : minbz;
                double u2 = maxbx > x2 ? x2 : maxbx;
                double v2 = maxby > y2 ? y2 : maxby;
                double w2 = maxbz > z2 ? z2 : maxbz;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u2 = (u2-x1)/unit;
                v1 = (v1-y1)/unit;
                v2 = (v2-y1)/unit;
                w1 = (w1-z1)/unit;
                w2 = (w2-z1)/unit;
                energy += gradientXMaxComponent(coeffs,v1,w1,u2,v2,w2);
            }

        }
    }

    return energy;
}

double Energy::gradientEvaluateYMaxComponent(const Eigen::VectorXd& x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstX = min.x()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;

    //fissare y1
    y1 = lastY;
    while (y1 >= maxby){
        y1-=unit;
    }
    double y2 = y1+unit;

    for (x1 = firstX; x1 <= lastX; x1+=unit){
        double x2 = x1 + unit;
        for (z1 = firstZ; z1<=lastZ; z1+=unit){
            double z2 = z1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                    maxby <= y1 || minby >= y2 ||
                    maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double u1 = minbx < x1 ? x1 : minbx;
                double w1 = minbz < z1 ? z1 : minbz;
                double u2 = maxbx > x2 ? x2 : maxbx;
                double v2 = maxby > y2 ? y2 : maxby;
                double w2 = maxbz > z2 ? z2 : maxbz;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u1 = (u1-x1)/unit;
                u2 = (u2-x1)/unit;
                v2 = (v2-y1)/unit;
                w1 = (w1-z1)/unit;
                w2 = (w2-z1)/unit;
                energy += gradientYMaxComponent(coeffs, u1,w1,u2,v2,w2);
            }
        }
    }

    return energy;
}

double Energy::gradientEvaluateZMaxComponent(const Eigen::VectorXd &x) const {
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    double firstX = min.x()-unit, firstY = min.y()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;

    //fissare z1
    z1 = lastZ;
    while (z1 >= maxbz){
        z1-=unit;
    }
    double z2 = z1+unit;

    for (x1 = firstX; x1 <= lastX; x1+=unit){
        double x2 = x1 + unit;
        for (y1 = firstY; y1<=lastY; y1+=unit){
            double y2 = y1 + unit;

            if (maxbx <= x1 || minbx >= x2 ||
                    maxby <= y1 || minby >= y2 ||
                    maxbz <= z1 || minbz >= z2 ) // not contained
                energy += 0;
            else {
                const gridreal* coeffs;
                g->getCoefficients(coeffs, Pointd(x1,y1,z1));

                double u1 = minbx < x1 ? x1 : minbx;
                double v1 = minby < y1 ? y1 : minby;
                double u2 = maxbx > x2 ? x2 : maxbx;
                double v2 = maxby > y2 ? y2 : maxby;
                double w2 = maxbz > z2 ? z2 : maxbz;
                //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                u1 = (u1-x1)/unit;
                u2 = (u2-x1)/unit;
                v1 = (v1-y1)/unit;
                v2 = (v2-y1)/unit;
                w2 = (w2-z1)/unit;
                energy += gradientZMaxComponent(coeffs, u1,v1,u2,v2,w2);
            }
        }
    }

    return energy;
}

void Energy::gradientEnergy(Eigen::VectorXd& gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    assert(x.rows() == 6);
    gradientTricubicInterpolationEnergy(gradient, x);
    //for (int i = 0; i < 6; i++) gradient(i) /= 2;
    Eigen::VectorXd gBarrier(6);
    gradientBarrier(gBarrier, x, c1, c2, c3);
    gradient += gBarrier;
    //gradient.normalize();
}

void Energy::gradientEnergy(Eigen::VectorXd& gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3, const Pointd& l) const {
    assert(x.rows() == 6);
    gradientEnergy(gradient, x, c1, c2, c3);
    Eigen::VectorXd gBarrier(6);
    gradientBarrierLimits(gBarrier, x, l);
    gradient += gBarrier;
}

void Energy::gradientEnergyFiniteDifference(Eigen::VectorXd& gradient, const Box3D b) const {
    Eigen::VectorXd x(6);
    x << b.getMin().x(), b.getMin().y(), b.getMin().z(), b.getMax().x(), b.getMax().y(), b.getMax().z();
    gradientEnergyFiniteDifference(gradient, x, b.getConstraint1(), b.getConstraint2(), b.getConstraint3());
}

void Energy::gradientEnergyFiniteDifference(Eigen::VectorXd& gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    gradient <<
            (energy(x(0)+EPSILON_GRAD, x(1), x(2), x(3), x(4), x(5), c1, c2, c3) - energy(x(0)-EPSILON_GRAD, x(1), x(2), x(3), x(4), x(5), c1, c2, c3)) / (2*EPSILON_GRAD),
            (energy(x(0), x(1)+EPSILON_GRAD, x(2), x(3), x(4), x(5), c1, c2, c3) - energy(x(0), x(1)-EPSILON_GRAD, x(2), x(3), x(4), x(5), c1, c2, c3)) / (2*EPSILON_GRAD),
            (energy(x(0), x(1), x(2)+EPSILON_GRAD, x(3), x(4), x(5), c1, c2, c3) - energy(x(0), x(1), x(2)-EPSILON_GRAD, x(3), x(4), x(5), c1, c2, c3)) / (2*EPSILON_GRAD),
            (energy(x(0), x(1), x(2), x(3)+EPSILON_GRAD, x(4), x(5), c1, c2, c3) - energy(x(0), x(1), x(2), x(3)-EPSILON_GRAD, x(4), x(5), c1, c2, c3)) / (2*EPSILON_GRAD),
            (energy(x(0), x(1), x(2), x(3), x(4)+EPSILON_GRAD, x(5), c1, c2, c3) - energy(x(0), x(1), x(2), x(3), x(4)-EPSILON_GRAD, x(5), c1, c2, c3)) / (2*EPSILON_GRAD),
            (energy(x(0), x(1), x(2), x(3), x(4), x(5)+EPSILON_GRAD, c1, c2, c3) - energy(x(0), x(1), x(2), x(3), x(4), x(5)-EPSILON_GRAD, c1, c2, c3)) / (2*EPSILON_GRAD);
}

//sia i coefficienti che la sotto-box integrata devono essere nell'intervallo 0-1
double Energy::integralTricubicInterpolation(const gridreal*& a, double u1, double v1, double w1, double u2, double v2, double w2) {
    //Simpy generated code
    double u12 = u1*u1; double u13 = u12*u1; double u14 = u13*u1;
    double v12 = v1*v1; double v13 = v12*v1; double v14 = v13*v1;
    double w12 = w1*w1; double w13 = w12*w1; double w14 = w13*w1;
    double u22 = u2*u2; double u23 = u22*u2; double u24 = u23*u2;
    double v22 = v2*v2; double v23 = v22*v2; double v24 = v23*v2;
    double w22 = w2*w2; double w23 = w22*w2; double w24 = w23*w2;

    double C_result;
    C_result = - w14 *
                   ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
                    (1.0L/8.0L)*a[49]*u12*v1 - 1.0L/8.0L*a[49]*u12*v2 - 1.0L/8.0L*a[49]*u22*v1 + (1.0L/8.0L)*a[49]*u22*v2 +
                    (1.0L/12.0L)*a[50]*u13*v1 - 1.0L/12.0L*a[50]*u13*v2 - 1.0L/12.0L*a[50]*u23*v1 + (1.0L/12.0L)*a[50]*u23*v2 +
                    (1.0L/16.0L)*a[51]*u14*v1 - 1.0L/16.0L*a[51]*u14*v2 - 1.0L/16.0L*a[51]*u24*v1 + (1.0L/16.0L)*a[51]*u24*v2 +
                    (1.0L/8.0L)*a[52]*u1*v12 - 1.0L/8.0L*a[52]*u1*v22 - 1.0L/8.0L*a[52]*u2*v12 + (1.0L/8.0L)*a[52]*u2*v22 +
                    (1.0L/16.0L)*a[53]*u12*v12 - 1.0L/16.0L*a[53]*u12*v22 - 1.0L/16.0L*a[53]*u22*v12 + (1.0L/16.0L)*a[53]*u22*v22 +
                    (1.0L/24.0L)*a[54]*u13*v12 - 1.0L/24.0L*a[54]*u13*v22 - 1.0L/24.0L*a[54]*u23*v12 + (1.0L/24.0L)*a[54]*u23*v22 +
                    (1.0L/32.0L)*a[55]*u14*v12 - 1.0L/32.0L*a[55]*u14*v22 - 1.0L/32.0L*a[55]*u24*v12 + (1.0L/32.0L)*a[55]*u24*v22 +
                    (1.0L/12.0L)*a[56]*u1*v13 - 1.0L/12.0L*a[56]*u1*v23 - 1.0L/12.0L*a[56]*u2*v13 + (1.0L/12.0L)*a[56]*u2*v23 +
                    (1.0L/24.0L)*a[57]*u12*v13 - 1.0L/24.0L*a[57]*u12*v23 - 1.0L/24.0L*a[57]*u22*v13 + (1.0L/24.0L)*a[57]*u22*v23 +
                    (1.0L/36.0L)*a[58]*u13*v13 - 1.0L/36.0L*a[58]*u13*v23 - 1.0L/36.0L*a[58]*u23*v13 + (1.0L/36.0L)*a[58]*u23*v23 +
                    (1.0L/48.0L)*a[59]*u14*v13 - 1.0L/48.0L*a[59]*u14*v23 - 1.0L/48.0L*a[59]*u24*v13 + (1.0L/48.0L)*a[59]*u24*v23 +
                    (1.0L/16.0L)*a[60]*u1*v14 - 1.0L/16.0L*a[60]*u1*v24 - 1.0L/16.0L*a[60]*u2*v14 + (1.0L/16.0L)*a[60]*u2*v24 +
                    (1.0L/32.0L)*a[61]*u12*v14 - 1.0L/32.0L*a[61]*u12*v24 - 1.0L/32.0L*a[61]*u22*v14 + (1.0L/32.0L)*a[61]*u22*v24 +
                    (1.0L/48.0L)*a[62]*u13*v14 - 1.0L/48.0L*a[62]*u13*v24 - 1.0L/48.0L*a[62]*u23*v14 + (1.0L/48.0L)*a[62]*u23*v24 +
                    (1.0L/64.0L)*a[63]*u14*v14 - 1.0L/64.0L*a[63]*u14*v24 - 1.0L/64.0L*a[63]*u24*v14 + (1.0L/64.0L)*a[63]*u24*v24)
                   - w13 *
                   ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
                    (1.0L/6.0L)*a[33]*u12*v1 - 1.0L/6.0L*a[33]*u12*v2 - 1.0L/6.0L*a[33]*u22*v1 + (1.0L/6.0L)*a[33]*u22*v2 +
                    (1.0L/9.0L)*a[34]*u13*v1 - 1.0L/9.0L*a[34]*u13*v2 - 1.0L/9.0L*a[34]*u23*v1 + (1.0L/9.0L)*a[34]*u23*v2 +
                    (1.0L/12.0L)*a[35]*u14*v1 - 1.0L/12.0L*a[35]*u14*v2 - 1.0L/12.0L*a[35]*u24*v1 + (1.0L/12.0L)*a[35]*u24*v2 +
                    (1.0L/6.0L)*a[36]*u1*v12 - 1.0L/6.0L*a[36]*u1*v22 - 1.0L/6.0L*a[36]*u2*v12 + (1.0L/6.0L)*a[36]*u2*v22 +
                    (1.0L/12.0L)*a[37]*u12*v12 - 1.0L/12.0L*a[37]*u12*v22 - 1.0L/12.0L*a[37]*u22*v12 + (1.0L/12.0L)*a[37]*u22*v22 +
                    (1.0L/18.0L)*a[38]*u13*v12 - 1.0L/18.0L*a[38]*u13*v22 - 1.0L/18.0L*a[38]*u23*v12 + (1.0L/18.0L)*a[38]*u23*v22 +
                    (1.0L/24.0L)*a[39]*u14*v12 - 1.0L/24.0L*a[39]*u14*v22 - 1.0L/24.0L*a[39]*u24*v12 + (1.0L/24.0L)*a[39]*u24*v22 +
                    (1.0L/9.0L)*a[40]*u1*v13 - 1.0L/9.0L*a[40]*u1*v23 - 1.0L/9.0L*a[40]*u2*v13 + (1.0L/9.0L)*a[40]*u2*v23 +
                    (1.0L/18.0L)*a[41]*u12*v13 - 1.0L/18.0L*a[41]*u12*v23 - 1.0L/18.0L*a[41]*u22*v13 + (1.0L/18.0L)*a[41]*u22*v23 +
                    (1.0L/27.0L)*a[42]*u13*v13 - 1.0L/27.0L*a[42]*u13*v23 - 1.0L/27.0L*a[42]*u23*v13 + (1.0L/27.0L)*a[42]*u23*v23 +
                    (1.0L/36.0L)*a[43]*u14*v13 - 1.0L/36.0L*a[43]*u14*v23 - 1.0L/36.0L*a[43]*u24*v13 + (1.0L/36.0L)*a[43]*u24*v23 +
                    (1.0L/12.0L)*a[44]*u1*v14 - 1.0L/12.0L*a[44]*u1*v24 - 1.0L/12.0L*a[44]*u2*v14 + (1.0L/12.0L)*a[44]*u2*v24 +
                    (1.0L/24.0L)*a[45]*u12*v14 - 1.0L/24.0L*a[45]*u12*v24 - 1.0L/24.0L*a[45]*u22*v14 + (1.0L/24.0L)*a[45]*u22*v24 +
                    (1.0L/36.0L)*a[46]*u13*v14 - 1.0L/36.0L*a[46]*u13*v24 - 1.0L/36.0L*a[46]*u23*v14 + (1.0L/36.0L)*a[46]*u23*v24 +
                    (1.0L/48.0L)*a[47]*u14*v14 - 1.0L/48.0L*a[47]*u14*v24 - 1.0L/48.0L*a[47]*u24*v14 + (1.0L/48.0L)*a[47]*u24*v24)
                   - w12 *
                   ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
                    (1.0L/4.0L)*a[17]*u12*v1 - 1.0L/4.0L*a[17]*u12*v2 - 1.0L/4.0L*a[17]*u22*v1 + (1.0L/4.0L)*a[17]*u22*v2 +
                    (1.0L/6.0L)*a[18]*u13*v1 - 1.0L/6.0L*a[18]*u13*v2 - 1.0L/6.0L*a[18]*u23*v1 + (1.0L/6.0L)*a[18]*u23*v2 +
                    (1.0L/8.0L)*a[19]*u14*v1 - 1.0L/8.0L*a[19]*u14*v2 - 1.0L/8.0L*a[19]*u24*v1 + (1.0L/8.0L)*a[19]*u24*v2 +
                    (1.0L/4.0L)*a[20]*u1*v12 - 1.0L/4.0L*a[20]*u1*v22 - 1.0L/4.0L*a[20]*u2*v12 + (1.0L/4.0L)*a[20]*u2*v22 +
                    (1.0L/8.0L)*a[21]*u12*v12 - 1.0L/8.0L*a[21]*u12*v22 - 1.0L/8.0L*a[21]*u22*v12 + (1.0L/8.0L)*a[21]*u22*v22 +
                    (1.0L/12.0L)*a[22]*u13*v12 - 1.0L/12.0L*a[22]*u13*v22 - 1.0L/12.0L*a[22]*u23*v12 + (1.0L/12.0L)*a[22]*u23*v22 +
                    (1.0L/16.0L)*a[23]*u14*v12 - 1.0L/16.0L*a[23]*u14*v22 - 1.0L/16.0L*a[23]*u24*v12 + (1.0L/16.0L)*a[23]*u24*v22 +
                    (1.0L/6.0L)*a[24]*u1*v13 - 1.0L/6.0L*a[24]*u1*v23 - 1.0L/6.0L*a[24]*u2*v13 + (1.0L/6.0L)*a[24]*u2*v23 +
                    (1.0L/12.0L)*a[25]*u12*v13 - 1.0L/12.0L*a[25]*u12*v23 - 1.0L/12.0L*a[25]*u22*v13 + (1.0L/12.0L)*a[25]*u22*v23 +
                    (1.0L/18.0L)*a[26]*u13*v13 - 1.0L/18.0L*a[26]*u13*v23 - 1.0L/18.0L*a[26]*u23*v13 + (1.0L/18.0L)*a[26]*u23*v23 +
                    (1.0L/24.0L)*a[27]*u14*v13 - 1.0L/24.0L*a[27]*u14*v23 - 1.0L/24.0L*a[27]*u24*v13 + (1.0L/24.0L)*a[27]*u24*v23 +
                    (1.0L/8.0L)*a[28]*u1*v14 - 1.0L/8.0L*a[28]*u1*v24 - 1.0L/8.0L*a[28]*u2*v14 + (1.0L/8.0L)*a[28]*u2*v24 +
                    (1.0L/16.0L)*a[29]*u12*v14 - 1.0L/16.0L*a[29]*u12*v24 - 1.0L/16.0L*a[29]*u22*v14 + (1.0L/16.0L)*a[29]*u22*v24 +
                    (1.0L/24.0L)*a[30]*u13*v14 - 1.0L/24.0L*a[30]*u13*v24 - 1.0L/24.0L*a[30]*u23*v14 + (1.0L/24.0L)*a[30]*u23*v24 +
                    (1.0L/32.0L)*a[31]*u14*v14 - 1.0L/32.0L*a[31]*u14*v24 - 1.0L/32.0L*a[31]*u24*v14 + (1.0L/32.0L)*a[31]*u24*v24)
                   - w1 *
                   (a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 +
                    (1.0L/2.0L)*a[1]*u12*v1 - 1.0L/2.0L*a[1]*u12*v2 - 1.0L/2.0L*a[1]*u22*v1 + (1.0L/2.0L)*a[1]*u22*v2 +
                    (1.0L/9.0L)*a[10]*u13*v13 - 1.0L/9.0L*a[10]*u13*v23 - 1.0L/9.0L*a[10]*u23*v13 + (1.0L/9.0L)*a[10]*u23*v23 +
                    (1.0L/12.0L)*a[11]*u14*v13 - 1.0L/12.0L*a[11]*u14*v23 - 1.0L/12.0L*a[11]*u24*v13 + (1.0L/12.0L)*a[11]*u24*v23 +
                    (1.0L/4.0L)*a[12]*u1*v14 - 1.0L/4.0L*a[12]*u1*v24 - 1.0L/4.0L*a[12]*u2*v14 + (1.0L/4.0L)*a[12]*u2*v24 +
                    (1.0L/8.0L)*a[13]*u12*v14 - 1.0L/8.0L*a[13]*u12*v24 - 1.0L/8.0L*a[13]*u22*v14 + (1.0L/8.0L)*a[13]*u22*v24 +
                    (1.0L/12.0L)*a[14]*u13*v14 - 1.0L/12.0L*a[14]*u13*v24 - 1.0L/12.0L*a[14]*u23*v14 + (1.0L/12.0L)*a[14]*u23*v24 +
                    (1.0L/16.0L)*a[15]*u14*v14 - 1.0L/16.0L*a[15]*u14*v24 - 1.0L/16.0L*a[15]*u24*v14 + (1.0L/16.0L)*a[15]*u24*v24 +
                    (1.0L/3.0L)*a[2]*u13*v1 - 1.0L/3.0L*a[2]*u13*v2 - 1.0L/3.0L*a[2]*u23*v1 + (1.0L/3.0L)*a[2]*u23*v2 +
                    (1.0L/4.0L)*a[3]*u14*v1 - 1.0L/4.0L*a[3]*u14*v2 - 1.0L/4.0L*a[3]*u24*v1 + (1.0L/4.0L)*a[3]*u24*v2 +
                    (1.0L/2.0L)*a[4]*u1*v12 - 1.0L/2.0L*a[4]*u1*v22 - 1.0L/2.0L*a[4]*u2*v12 + (1.0L/2.0L)*a[4]*u2*v22 +
                    (1.0L/4.0L)*a[5]*u12*v12 - 1.0L/4.0L*a[5]*u12*v22 - 1.0L/4.0L*a[5]*u22*v12 + (1.0L/4.0L)*a[5]*u22*v22 +
                    (1.0L/6.0L)*a[6]*u13*v12 - 1.0L/6.0L*a[6]*u13*v22 - 1.0L/6.0L*a[6]*u23*v12 + (1.0L/6.0L)*a[6]*u23*v22 +
                    (1.0L/8.0L)*a[7]*u14*v12 - 1.0L/8.0L*a[7]*u14*v22 - 1.0L/8.0L*a[7]*u24*v12 + (1.0L/8.0L)*a[7]*u24*v22 +
                    (1.0L/3.0L)*a[8]*u1*v13 - 1.0L/3.0L*a[8]*u1*v23 - 1.0L/3.0L*a[8]*u2*v13 + (1.0L/3.0L)*a[8]*u2*v23 +
                    (1.0L/6.0L)*a[9]*u12*v13 - 1.0L/6.0L*a[9]*u12*v23 - 1.0L/6.0L*a[9]*u22*v13 + (1.0L/6.0L)*a[9]*u22*v23)
                   + w24 *
                   ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
                    (1.0L/8.0L)*a[49]*u12*v1 - 1.0L/8.0L*a[49]*u12*v2 - 1.0L/8.0L*a[49]*u22*v1 + (1.0L/8.0L)*a[49]*u22*v2 +
                    (1.0L/12.0L)*a[50]*u13*v1 - 1.0L/12.0L*a[50]*u13*v2 - 1.0L/12.0L*a[50]*u23*v1 + (1.0L/12.0L)*a[50]*u23*v2 +
                    (1.0L/16.0L)*a[51]*u14*v1 - 1.0L/16.0L*a[51]*u14*v2 - 1.0L/16.0L*a[51]*u24*v1 + (1.0L/16.0L)*a[51]*u24*v2 +
                    (1.0L/8.0L)*a[52]*u1*v12 - 1.0L/8.0L*a[52]*u1*v22 - 1.0L/8.0L*a[52]*u2*v12 + (1.0L/8.0L)*a[52]*u2*v22 +
                    (1.0L/16.0L)*a[53]*u12*v12 - 1.0L/16.0L*a[53]*u12*v22 - 1.0L/16.0L*a[53]*u22*v12 + (1.0L/16.0L)*a[53]*u22*v22 +
                    (1.0L/24.0L)*a[54]*u13*v12 - 1.0L/24.0L*a[54]*u13*v22 - 1.0L/24.0L*a[54]*u23*v12 + (1.0L/24.0L)*a[54]*u23*v22 +
                    (1.0L/32.0L)*a[55]*u14*v12 - 1.0L/32.0L*a[55]*u14*v22 - 1.0L/32.0L*a[55]*u24*v12 + (1.0L/32.0L)*a[55]*u24*v22 +
                    (1.0L/12.0L)*a[56]*u1*v13 - 1.0L/12.0L*a[56]*u1*v23 - 1.0L/12.0L*a[56]*u2*v13 + (1.0L/12.0L)*a[56]*u2*v23 +
                    (1.0L/24.0L)*a[57]*u12*v13 - 1.0L/24.0L*a[57]*u12*v23 - 1.0L/24.0L*a[57]*u22*v13 + (1.0L/24.0L)*a[57]*u22*v23 +
                    (1.0L/36.0L)*a[58]*u13*v13 - 1.0L/36.0L*a[58]*u13*v23 - 1.0L/36.0L*a[58]*u23*v13 + (1.0L/36.0L)*a[58]*u23*v23 +
                    (1.0L/48.0L)*a[59]*u14*v13 - 1.0L/48.0L*a[59]*u14*v23 - 1.0L/48.0L*a[59]*u24*v13 + (1.0L/48.0L)*a[59]*u24*v23 +
                    (1.0L/16.0L)*a[60]*u1*v14 - 1.0L/16.0L*a[60]*u1*v24 - 1.0L/16.0L*a[60]*u2*v14 + (1.0L/16.0L)*a[60]*u2*v24 +
                    (1.0L/32.0L)*a[61]*u12*v14 - 1.0L/32.0L*a[61]*u12*v24 - 1.0L/32.0L*a[61]*u22*v14 + (1.0L/32.0L)*a[61]*u22*v24 +
                    (1.0L/48.0L)*a[62]*u13*v14 - 1.0L/48.0L*a[62]*u13*v24 - 1.0L/48.0L*a[62]*u23*v14 + (1.0L/48.0L)*a[62]*u23*v24 +
                    (1.0L/64.0L)*a[63]*u14*v14 - 1.0L/64.0L*a[63]*u14*v24 - 1.0L/64.0L*a[63]*u24*v14 + (1.0L/64.0L)*a[63]*u24*v24)
                   + w23 *
                   ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
                    (1.0L/6.0L)*a[33]*u12*v1 - 1.0L/6.0L*a[33]*u12*v2 - 1.0L/6.0L*a[33]*u22*v1 + (1.0L/6.0L)*a[33]*u22*v2 +
                    (1.0L/9.0L)*a[34]*u13*v1 - 1.0L/9.0L*a[34]*u13*v2 - 1.0L/9.0L*a[34]*u23*v1 + (1.0L/9.0L)*a[34]*u23*v2 +
                    (1.0L/12.0L)*a[35]*u14*v1 - 1.0L/12.0L*a[35]*u14*v2 - 1.0L/12.0L*a[35]*u24*v1 + (1.0L/12.0L)*a[35]*u24*v2 +
                    (1.0L/6.0L)*a[36]*u1*v12 - 1.0L/6.0L*a[36]*u1*v22 - 1.0L/6.0L*a[36]*u2*v12 + (1.0L/6.0L)*a[36]*u2*v22 +
                    (1.0L/12.0L)*a[37]*u12*v12 - 1.0L/12.0L*a[37]*u12*v22 - 1.0L/12.0L*a[37]*u22*v12 + (1.0L/12.0L)*a[37]*u22*v22 +
                    (1.0L/18.0L)*a[38]*u13*v12 - 1.0L/18.0L*a[38]*u13*v22 - 1.0L/18.0L*a[38]*u23*v12 + (1.0L/18.0L)*a[38]*u23*v22 +
                    (1.0L/24.0L)*a[39]*u14*v12 - 1.0L/24.0L*a[39]*u14*v22 - 1.0L/24.0L*a[39]*u24*v12 + (1.0L/24.0L)*a[39]*u24*v22 +
                    (1.0L/9.0L)*a[40]*u1*v13 - 1.0L/9.0L*a[40]*u1*v23 - 1.0L/9.0L*a[40]*u2*v13 + (1.0L/9.0L)*a[40]*u2*v23 +
                    (1.0L/18.0L)*a[41]*u12*v13 - 1.0L/18.0L*a[41]*u12*v23 - 1.0L/18.0L*a[41]*u22*v13 + (1.0L/18.0L)*a[41]*u22*v23 +
                    (1.0L/27.0L)*a[42]*u13*v13 - 1.0L/27.0L*a[42]*u13*v23 - 1.0L/27.0L*a[42]*u23*v13 + (1.0L/27.0L)*a[42]*u23*v23 +
                    (1.0L/36.0L)*a[43]*u14*v13 - 1.0L/36.0L*a[43]*u14*v23 - 1.0L/36.0L*a[43]*u24*v13 + (1.0L/36.0L)*a[43]*u24*v23 +
                    (1.0L/12.0L)*a[44]*u1*v14 - 1.0L/12.0L*a[44]*u1*v24 - 1.0L/12.0L*a[44]*u2*v14 + (1.0L/12.0L)*a[44]*u2*v24 +
                    (1.0L/24.0L)*a[45]*u12*v14 - 1.0L/24.0L*a[45]*u12*v24 - 1.0L/24.0L*a[45]*u22*v14 + (1.0L/24.0L)*a[45]*u22*v24 +
                    (1.0L/36.0L)*a[46]*u13*v14 - 1.0L/36.0L*a[46]*u13*v24 - 1.0L/36.0L*a[46]*u23*v14 + (1.0L/36.0L)*a[46]*u23*v24 +
                    (1.0L/48.0L)*a[47]*u14*v14 - 1.0L/48.0L*a[47]*u14*v24 - 1.0L/48.0L*a[47]*u24*v14 + (1.0L/48.0L)*a[47]*u24*v24)
                   + w22 *
                   ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
                    (1.0L/4.0L)*a[17]*u12*v1 - 1.0L/4.0L*a[17]*u12*v2 - 1.0L/4.0L*a[17]*u22*v1 + (1.0L/4.0L)*a[17]*u22*v2 +
                    (1.0L/6.0L)*a[18]*u13*v1 - 1.0L/6.0L*a[18]*u13*v2 - 1.0L/6.0L*a[18]*u23*v1 + (1.0L/6.0L)*a[18]*u23*v2 +
                    (1.0L/8.0L)*a[19]*u14*v1 - 1.0L/8.0L*a[19]*u14*v2 - 1.0L/8.0L*a[19]*u24*v1 + (1.0L/8.0L)*a[19]*u24*v2 +
                    (1.0L/4.0L)*a[20]*u1*v12 - 1.0L/4.0L*a[20]*u1*v22 - 1.0L/4.0L*a[20]*u2*v12 + (1.0L/4.0L)*a[20]*u2*v22 +
                    (1.0L/8.0L)*a[21]*u12*v12 - 1.0L/8.0L*a[21]*u12*v22 - 1.0L/8.0L*a[21]*u22*v12 + (1.0L/8.0L)*a[21]*u22*v22 +
                    (1.0L/12.0L)*a[22]*u13*v12 - 1.0L/12.0L*a[22]*u13*v22 - 1.0L/12.0L*a[22]*u23*v12 + (1.0L/12.0L)*a[22]*u23*v22 +
                    (1.0L/16.0L)*a[23]*u14*v12 - 1.0L/16.0L*a[23]*u14*v22 - 1.0L/16.0L*a[23]*u24*v12 + (1.0L/16.0L)*a[23]*u24*v22 +
                    (1.0L/6.0L)*a[24]*u1*v13 - 1.0L/6.0L*a[24]*u1*v23 - 1.0L/6.0L*a[24]*u2*v13 + (1.0L/6.0L)*a[24]*u2*v23 +
                    (1.0L/12.0L)*a[25]*u12*v13 - 1.0L/12.0L*a[25]*u12*v23 - 1.0L/12.0L*a[25]*u22*v13 + (1.0L/12.0L)*a[25]*u22*v23 +
                    (1.0L/18.0L)*a[26]*u13*v13 - 1.0L/18.0L*a[26]*u13*v23 - 1.0L/18.0L*a[26]*u23*v13 + (1.0L/18.0L)*a[26]*u23*v23 +
                    (1.0L/24.0L)*a[27]*u14*v13 - 1.0L/24.0L*a[27]*u14*v23 - 1.0L/24.0L*a[27]*u24*v13 + (1.0L/24.0L)*a[27]*u24*v23 +
                    (1.0L/8.0L)*a[28]*u1*v14 - 1.0L/8.0L*a[28]*u1*v24 - 1.0L/8.0L*a[28]*u2*v14 + (1.0L/8.0L)*a[28]*u2*v24 +
                    (1.0L/16.0L)*a[29]*u12*v14 - 1.0L/16.0L*a[29]*u12*v24 - 1.0L/16.0L*a[29]*u22*v14 + (1.0L/16.0L)*a[29]*u22*v24 +
                    (1.0L/24.0L)*a[30]*u13*v14 - 1.0L/24.0L*a[30]*u13*v24 - 1.0L/24.0L*a[30]*u23*v14 + (1.0L/24.0L)*a[30]*u23*v24 +
                    (1.0L/32.0L)*a[31]*u14*v14 - 1.0L/32.0L*a[31]*u14*v24 - 1.0L/32.0L*a[31]*u24*v14 + (1.0L/32.0L)*a[31]*u24*v24)
                   + w2 *
                   (a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 +
                    (1.0L/2.0L)*a[1]*u12*v1 - 1.0L/2.0L*a[1]*u12*v2 - 1.0L/2.0L*a[1]*u22*v1 + (1.0L/2.0L)*a[1]*u22*v2 +
                    (1.0L/9.0L)*a[10]*u13*v13 - 1.0L/9.0L*a[10]*u13*v23 - 1.0L/9.0L*a[10]*u23*v13 + (1.0L/9.0L)*a[10]*u23*v23 +
                    (1.0L/12.0L)*a[11]*u14*v13 - 1.0L/12.0L*a[11]*u14*v23 - 1.0L/12.0L*a[11]*u24*v13 + (1.0L/12.0L)*a[11]*u24*v23 +
                    (1.0L/4.0L)*a[12]*u1*v14 - 1.0L/4.0L*a[12]*u1*v24 - 1.0L/4.0L*a[12]*u2*v14 + (1.0L/4.0L)*a[12]*u2*v24 +
                    (1.0L/8.0L)*a[13]*u12*v14 - 1.0L/8.0L*a[13]*u12*v24 - 1.0L/8.0L*a[13]*u22*v14 + (1.0L/8.0L)*a[13]*u22*v24 +
                    (1.0L/12.0L)*a[14]*u13*v14 - 1.0L/12.0L*a[14]*u13*v24 - 1.0L/12.0L*a[14]*u23*v14 + (1.0L/12.0L)*a[14]*u23*v24 +
                    (1.0L/16.0L)*a[15]*u14*v14 - 1.0L/16.0L*a[15]*u14*v24 - 1.0L/16.0L*a[15]*u24*v14 + (1.0L/16.0L)*a[15]*u24*v24 +
                    (1.0L/3.0L)*a[2]*u13*v1 - 1.0L/3.0L*a[2]*u13*v2 - 1.0L/3.0L*a[2]*u23*v1 + (1.0L/3.0L)*a[2]*u23*v2 +
                    (1.0L/4.0L)*a[3]*u14*v1 - 1.0L/4.0L*a[3]*u14*v2 - 1.0L/4.0L*a[3]*u24*v1 + (1.0L/4.0L)*a[3]*u24*v2 +
                    (1.0L/2.0L)*a[4]*u1*v12 - 1.0L/2.0L*a[4]*u1*v22 - 1.0L/2.0L*a[4]*u2*v12 + (1.0L/2.0L)*a[4]*u2*v22 +
                    (1.0L/4.0L)*a[5]*u12*v12 - 1.0L/4.0L*a[5]*u12*v22 - 1.0L/4.0L*a[5]*u22*v12 + (1.0L/4.0L)*a[5]*u22*v22 +
                    (1.0L/6.0L)*a[6]*u13*v12 - 1.0L/6.0L*a[6]*u13*v22 - 1.0L/6.0L*a[6]*u23*v12 + (1.0L/6.0L)*a[6]*u23*v22 +
                    (1.0L/8.0L)*a[7]*u14*v12 - 1.0L/8.0L*a[7]*u14*v22 - 1.0L/8.0L*a[7]*u24*v12 + (1.0L/8.0L)*a[7]*u24*v22 +
                    (1.0L/3.0L)*a[8]*u1*v13 - 1.0L/3.0L*a[8]*u1*v23 - 1.0L/3.0L*a[8]*u2*v13 + (1.0L/3.0L)*a[8]*u2*v23 +
                    (1.0L/6.0L)*a[9]*u12*v13 - 1.0L/6.0L*a[9]*u12*v23 - 1.0L/6.0L*a[9]*u22*v13 + (1.0L/6.0L)*a[9]*u22*v23);
    return C_result;
}

double Energy::integralTricubicInterpolationEnergy(const Pointd& bmin, const Pointd& bmax) const {
    Eigen::VectorXd x(6);
    x << bmin.x(), bmin.y(), bmin.z(), bmax.x(), bmax.y(), bmax.z();
    return integralTricubicInterpolationEnergy(x);

}

double Energy::integralTricubicInterpolationEnergy(const Eigen::VectorXd& x) const {
    //g->deleteCubes();
    double unit = g->getUnit();

    Pointd min, max;
    initializeMinMax(min, max, x);

    /*
    const Pointd &c = bb.getMin();
    Pointd d = c-Pointd(x(0), x(1), x(2));
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-Pointd(x(3), x(4), x(5));
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());*/


    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = x(0), minby = x(1), minbz = x(2);
    double maxbx = x(3), maxby = x(4), maxbz = x(5);
    double energy = 0;

    double x1, y1, z1;
    for (x1 = firstX; x1 <= lastX; x1+=unit){
        double x2 = x1 + unit;
        for (y1 = firstY; y1 <= lastY; y1+=unit){
            double y2 = y1 + unit;
            for (z1 = firstZ; z1<=lastZ; z1+=unit){
                double z2 = z1 + unit;

                if (maxbx <= x1 || minbx >= x2 ||
                    maxby <= y1 || minby >= y2 ||
                    maxbz <= z1 || minbz >= z2 ) // not contained
                    energy += 0;
                else {
                    const gridreal* coeffs;
                    //g->getCoefficients(coeffs, Pointd(x1,y1,z1));
                    if (minbx <= x1 && maxbx >= x2 &&
                        minby <= y1 && maxby >= y2 &&
                        minbz <= z1 && maxbz >= z2 ) { // completly contained
                        energy += g->getFullBoxValue(Pointd(x1,y1,z1));
                        //energy += integralTricubicInterpolation(coeffs, 0,0,0,1,1,1);
                        //g->addCube(BoundingBox(Pointd(x1,y1,z1), Pointd(x2,y2,z2)));
                    }
                    else { //partially contained
                        g->getCoefficients(coeffs, Pointd(x1,y1,z1));
                        double u1 = minbx < x1 ? x1 : minbx;
                        double v1 = minby < y1 ? y1 : minby;
                        double w1 = minbz < z1 ? z1 : minbz;
                        double u2 = maxbx > x2 ? x2 : maxbx;
                        double v2 = maxby > y2 ? y2 : maxby;
                        double w2 = maxbz > z2 ? z2 : maxbz;
                        //g->addCube(BoundingBox(Pointd(u1,v1,w1), Pointd(u2,v2,w2)));
                        u1 = (u1-x1)/unit;
                        u2 = (u2-x1)/unit;
                        v1 = (v1-y1)/unit;
                        v2 = (v2-y1)/unit;
                        w1 = (w1-z1)/unit;
                        w2 = (w2-z1)/unit;
                        energy += integralTricubicInterpolation(coeffs, u1,v1,w1,u2,v2,w2);
                    }
                }

            }
        }
    }

    return energy;
}
