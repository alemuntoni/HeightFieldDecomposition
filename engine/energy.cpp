#include "energy.h"


Energy::Energy() {
}

Energy::Energy(DrawableGrid& g) : g(&g){
}

double Energy::gradientDiscend(Box3D& b, BoxList& iterations) const {
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
    Eigen::VectorXd diff(6); diff = x;
    do{
        objValue = energy(x, c1, c2, c3);
        //Eigen::Vector4d gradient;
        //Eigen::VectorXd gradientFiniteDiffernece(6);

        //e.gradientEnergy(gradient, x, c.x(), c.y());
        //gradientEnergyFiniteDifference(gradientFiniteDiffernece, x, c1, c2, c3);
        gradientEnergy(gradient, x, c1, c2, c3);
        //std::cerr << "Diff norm: \n" << (gradient-gradientFiniteDiffernece).norm() << "\n\n";
        //new_x = x - alfa * gradientFiniteDiffernece;
        new_x = x - alfa * gradient;
        newObjValue = energy(new_x, c1, c2, c3);
        if (newObjValue < objValue) {
            nIterations++;
            diff = new_x - x;
            //std::cerr << "It: " << nIterations << "; Diff norm: " << diff.norm() << "\n";
            x = new_x;
            b.setMin(Pointd(x(0), x(1), x(2)));
            b.setMax(Pointd(x(3), x(4), x(5)));
            iterations.addBox(b);
            alfa *= 2;
        }
        else{
            alfa /= 10;
            //std::cerr << "alfa: " << alfa << "\n";
        }
    } while (diff.norm() > 0.005 && nIterations < 1000);
    std::cerr << "Gradient norm: " << gradient.norm() << "\n";
    b.setMin(Pointd(x(0), x(1), x(2)));
    b.setMax(Pointd(x(3), x(4), x(5)));
    iterations.addBox(b);
    std::cerr << gradient;
    return nIterations;
}

double Energy::gradientXMinComponent(const double*& a, double u1, double v1, double w1, double v2, double w2) {
    double G_result;
    G_result = -pow(w1, 4)*((1.0L/4.0L)*a[48]*v1 - 1.0L/4.0L*a[48]*v2 + (1.0L/4.0L)*a[49]*u1*v1 - 1.0L/4.0L*a[49]*u1*v2 + (1.0L/4.0L)*a[50]*pow(u1, 2)*v1 - 1.0L/4.0L*a[50]*pow(u1, 2)*v2 + (1.0L/4.0L)*a[51]*pow(u1, 3)*v1 - 1.0L/4.0L*a[51]*pow(u1, 3)*v2 + (1.0L/8.0L)*a[52]*pow(v1, 2) - 1.0L/8.0L*a[52]*pow(v2, 2) + (1.0L/8.0L)*a[53]*u1*pow(v1, 2) - 1.0L/8.0L*a[53]*u1*pow(v2, 2) + (1.0L/8.0L)*a[54]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[54]*pow(u1, 2)*pow(v2, 2) + (1.0L/8.0L)*a[55]*pow(u1, 3)*pow(v1, 2) - 1.0L/8.0L*a[55]*pow(u1, 3)*pow(v2, 2) + (1.0L/12.0L)*a[56]*pow(v1, 3) - 1.0L/12.0L*a[56]*pow(v2, 3) + (1.0L/12.0L)*a[57]*u1*pow(v1, 3) - 1.0L/12.0L*a[57]*u1*pow(v2, 3) + (1.0L/12.0L)*a[58]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[58]*pow(u1, 2)*pow(v2, 3) + (1.0L/12.0L)*a[59]*pow(u1, 3)*pow(v1, 3) - 1.0L/12.0L*a[59]*pow(u1, 3)*pow(v2, 3) + (1.0L/16.0L)*a[60]*pow(v1, 4) - 1.0L/16.0L*a[60]*pow(v2, 4) + (1.0L/16.0L)*a[61]*u1*pow(v1, 4) - 1.0L/16.0L*a[61]*u1*pow(v2, 4) + (1.0L/16.0L)*a[62]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[62]*pow(u1, 2)*pow(v2, 4) + (1.0L/16.0L)*a[63]*pow(u1, 3)*pow(v1, 4) - 1.0L/16.0L*a[63]*pow(u1, 3)*pow(v2, 4)) - pow(w1, 3)*((1.0L/3.0L)*a[32]*v1 - 1.0L/3.0L*a[32]*v2 + (1.0L/3.0L)*a[33]*u1*v1 - 1.0L/3.0L*a[33]*u1*v2 + (1.0L/3.0L)*a[34]*pow(u1, 2)*v1 - 1.0L/3.0L*a[34]*pow(u1, 2)*v2 + (1.0L/3.0L)*a[35]*pow(u1, 3)*v1 - 1.0L/3.0L*a[35]*pow(u1, 3)*v2 + (1.0L/6.0L)*a[36]*pow(v1, 2) - 1.0L/6.0L*a[36]*pow(v2, 2) + (1.0L/6.0L)*a[37]*u1*pow(v1, 2) - 1.0L/6.0L*a[37]*u1*pow(v2, 2) + (1.0L/6.0L)*a[38]*pow(u1, 2)*pow(v1, 2) - 1.0L/6.0L*a[38]*pow(u1, 2)*pow(v2, 2) + (1.0L/6.0L)*a[39]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[39]*pow(u1, 3)*pow(v2, 2) + (1.0L/9.0L)*a[40]*pow(v1, 3) - 1.0L/9.0L*a[40]*pow(v2, 3) + (1.0L/9.0L)*a[41]*u1*pow(v1, 3) - 1.0L/9.0L*a[41]*u1*pow(v2, 3) + (1.0L/9.0L)*a[42]*pow(u1, 2)*pow(v1, 3) - 1.0L/9.0L*a[42]*pow(u1, 2)*pow(v2, 3) + (1.0L/9.0L)*a[43]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[43]*pow(u1, 3)*pow(v2, 3) + (1.0L/12.0L)*a[44]*pow(v1, 4) - 1.0L/12.0L*a[44]*pow(v2, 4) + (1.0L/12.0L)*a[45]*u1*pow(v1, 4) - 1.0L/12.0L*a[45]*u1*pow(v2, 4) + (1.0L/12.0L)*a[46]*pow(u1, 2)*pow(v1, 4) - 1.0L/12.0L*a[46]*pow(u1, 2)*pow(v2, 4) + (1.0L/12.0L)*a[47]*pow(u1, 3)*pow(v1, 4) - 1.0L/12.0L*a[47]*pow(u1, 3)*pow(v2, 4)) - pow(w1, 2)*((1.0L/2.0L)*a[16]*v1 - 1.0L/2.0L*a[16]*v2 + (1.0L/2.0L)*a[17]*u1*v1 - 1.0L/2.0L*a[17]*u1*v2 + (1.0L/2.0L)*a[18]*pow(u1, 2)*v1 - 1.0L/2.0L*a[18]*pow(u1, 2)*v2 + (1.0L/2.0L)*a[19]*pow(u1, 3)*v1 - 1.0L/2.0L*a[19]*pow(u1, 3)*v2 + (1.0L/4.0L)*a[20]*pow(v1, 2) - 1.0L/4.0L*a[20]*pow(v2, 2) + (1.0L/4.0L)*a[21]*u1*pow(v1, 2) - 1.0L/4.0L*a[21]*u1*pow(v2, 2) + (1.0L/4.0L)*a[22]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[22]*pow(u1, 2)*pow(v2, 2) + (1.0L/4.0L)*a[23]*pow(u1, 3)*pow(v1, 2) - 1.0L/4.0L*a[23]*pow(u1, 3)*pow(v2, 2) + (1.0L/6.0L)*a[24]*pow(v1, 3) - 1.0L/6.0L*a[24]*pow(v2, 3) + (1.0L/6.0L)*a[25]*u1*pow(v1, 3) - 1.0L/6.0L*a[25]*u1*pow(v2, 3) + (1.0L/6.0L)*a[26]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[26]*pow(u1, 2)*pow(v2, 3) + (1.0L/6.0L)*a[27]*pow(u1, 3)*pow(v1, 3) - 1.0L/6.0L*a[27]*pow(u1, 3)*pow(v2, 3) + (1.0L/8.0L)*a[28]*pow(v1, 4) - 1.0L/8.0L*a[28]*pow(v2, 4) + (1.0L/8.0L)*a[29]*u1*pow(v1, 4) - 1.0L/8.0L*a[29]*u1*pow(v2, 4) + (1.0L/8.0L)*a[30]*pow(u1, 2)*pow(v1, 4) - 1.0L/8.0L*a[30]*pow(u1, 2)*pow(v2, 4) + (1.0L/8.0L)*a[31]*pow(u1, 3)*pow(v1, 4) - 1.0L/8.0L*a[31]*pow(u1, 3)*pow(v2, 4)) - w1*(a[0]*v1 - a[0]*v2 + a[1]*u1*v1 - a[1]*u1*v2 + (1.0L/3.0L)*a[10]*pow(u1, 2)*pow(v1, 3) - 1.0L/3.0L*a[10]*pow(u1, 2)*pow(v2, 3) + (1.0L/3.0L)*a[11]*pow(u1, 3)*pow(v1, 3) - 1.0L/3.0L*a[11]*pow(u1, 3)*pow(v2, 3) + (1.0L/4.0L)*a[12]*pow(v1, 4) - 1.0L/4.0L*a[12]*pow(v2, 4) + (1.0L/4.0L)*a[13]*u1*pow(v1, 4) - 1.0L/4.0L*a[13]*u1*pow(v2, 4) + (1.0L/4.0L)*a[14]*pow(u1, 2)*pow(v1, 4) - 1.0L/4.0L*a[14]*pow(u1, 2)*pow(v2, 4) + (1.0L/4.0L)*a[15]*pow(u1, 3)*pow(v1, 4) - 1.0L/4.0L*a[15]*pow(u1, 3)*pow(v2, 4) + a[2]*pow(u1, 2)*v1 - a[2]*pow(u1, 2)*v2 + a[3]*pow(u1, 3)*v1 - a[3]*pow(u1, 3)*v2 + (1.0L/2.0L)*a[4]*pow(v1, 2) - 1.0L/2.0L*a[4]*pow(v2, 2) + (1.0L/2.0L)*a[5]*u1*pow(v1, 2) - 1.0L/2.0L*a[5]*u1*pow(v2, 2) + (1.0L/2.0L)*a[6]*pow(u1, 2)*pow(v1, 2) - 1.0L/2.0L*a[6]*pow(u1, 2)*pow(v2, 2) + (1.0L/2.0L)*a[7]*pow(u1, 3)*pow(v1, 2) - 1.0L/2.0L*a[7]*pow(u1, 3)*pow(v2, 2) + (1.0L/3.0L)*a[8]*pow(v1, 3) - 1.0L/3.0L*a[8]*pow(v2, 3) + (1.0L/3.0L)*a[9]*u1*pow(v1, 3) - 1.0L/3.0L*a[9]*u1*pow(v2, 3)) + pow(w2, 4)*((1.0L/4.0L)*a[48]*v1 - 1.0L/4.0L*a[48]*v2 + (1.0L/4.0L)*a[49]*u1*v1 - 1.0L/4.0L*a[49]*u1*v2 + (1.0L/4.0L)*a[50]*pow(u1, 2)*v1 - 1.0L/4.0L*a[50]*pow(u1, 2)*v2 + (1.0L/4.0L)*a[51]*pow(u1, 3)*v1 - 1.0L/4.0L*a[51]*pow(u1, 3)*v2 + (1.0L/8.0L)*a[52]*pow(v1, 2) - 1.0L/8.0L*a[52]*pow(v2, 2) + (1.0L/8.0L)*a[53]*u1*pow(v1, 2) - 1.0L/8.0L*a[53]*u1*pow(v2, 2) + (1.0L/8.0L)*a[54]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[54]*pow(u1, 2)*pow(v2, 2) + (1.0L/8.0L)*a[55]*pow(u1, 3)*pow(v1, 2) - 1.0L/8.0L*a[55]*pow(u1, 3)*pow(v2, 2) + (1.0L/12.0L)*a[56]*pow(v1, 3) - 1.0L/12.0L*a[56]*pow(v2, 3) + (1.0L/12.0L)*a[57]*u1*pow(v1, 3) - 1.0L/12.0L*a[57]*u1*pow(v2, 3) + (1.0L/12.0L)*a[58]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[58]*pow(u1, 2)*pow(v2, 3) + (1.0L/12.0L)*a[59]*pow(u1, 3)*pow(v1, 3) - 1.0L/12.0L*a[59]*pow(u1, 3)*pow(v2, 3) + (1.0L/16.0L)*a[60]*pow(v1, 4) - 1.0L/16.0L*a[60]*pow(v2, 4) + (1.0L/16.0L)*a[61]*u1*pow(v1, 4) - 1.0L/16.0L*a[61]*u1*pow(v2, 4) + (1.0L/16.0L)*a[62]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[62]*pow(u1, 2)*pow(v2, 4) + (1.0L/16.0L)*a[63]*pow(u1, 3)*pow(v1, 4) - 1.0L/16.0L*a[63]*pow(u1, 3)*pow(v2, 4)) + pow(w2, 3)*((1.0L/3.0L)*a[32]*v1 - 1.0L/3.0L*a[32]*v2 + (1.0L/3.0L)*a[33]*u1*v1 - 1.0L/3.0L*a[33]*u1*v2 + (1.0L/3.0L)*a[34]*pow(u1, 2)*v1 - 1.0L/3.0L*a[34]*pow(u1, 2)*v2 + (1.0L/3.0L)*a[35]*pow(u1, 3)*v1 - 1.0L/3.0L*a[35]*pow(u1, 3)*v2 + (1.0L/6.0L)*a[36]*pow(v1, 2) - 1.0L/6.0L*a[36]*pow(v2, 2) + (1.0L/6.0L)*a[37]*u1*pow(v1, 2) - 1.0L/6.0L*a[37]*u1*pow(v2, 2) + (1.0L/6.0L)*a[38]*pow(u1, 2)*pow(v1, 2) - 1.0L/6.0L*a[38]*pow(u1, 2)*pow(v2, 2) + (1.0L/6.0L)*a[39]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[39]*pow(u1, 3)*pow(v2, 2) + (1.0L/9.0L)*a[40]*pow(v1, 3) - 1.0L/9.0L*a[40]*pow(v2, 3) + (1.0L/9.0L)*a[41]*u1*pow(v1, 3) - 1.0L/9.0L*a[41]*u1*pow(v2, 3) + (1.0L/9.0L)*a[42]*pow(u1, 2)*pow(v1, 3) - 1.0L/9.0L*a[42]*pow(u1, 2)*pow(v2, 3) + (1.0L/9.0L)*a[43]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[43]*pow(u1, 3)*pow(v2, 3) + (1.0L/12.0L)*a[44]*pow(v1, 4) - 1.0L/12.0L*a[44]*pow(v2, 4) + (1.0L/12.0L)*a[45]*u1*pow(v1, 4) - 1.0L/12.0L*a[45]*u1*pow(v2, 4) + (1.0L/12.0L)*a[46]*pow(u1, 2)*pow(v1, 4) - 1.0L/12.0L*a[46]*pow(u1, 2)*pow(v2, 4) + (1.0L/12.0L)*a[47]*pow(u1, 3)*pow(v1, 4) - 1.0L/12.0L*a[47]*pow(u1, 3)*pow(v2, 4)) + pow(w2, 2)*((1.0L/2.0L)*a[16]*v1 - 1.0L/2.0L*a[16]*v2 + (1.0L/2.0L)*a[17]*u1*v1 - 1.0L/2.0L*a[17]*u1*v2 + (1.0L/2.0L)*a[18]*pow(u1, 2)*v1 - 1.0L/2.0L*a[18]*pow(u1, 2)*v2 + (1.0L/2.0L)*a[19]*pow(u1, 3)*v1 - 1.0L/2.0L*a[19]*pow(u1, 3)*v2 + (1.0L/4.0L)*a[20]*pow(v1, 2) - 1.0L/4.0L*a[20]*pow(v2, 2) + (1.0L/4.0L)*a[21]*u1*pow(v1, 2) - 1.0L/4.0L*a[21]*u1*pow(v2, 2) + (1.0L/4.0L)*a[22]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[22]*pow(u1, 2)*pow(v2, 2) + (1.0L/4.0L)*a[23]*pow(u1, 3)*pow(v1, 2) - 1.0L/4.0L*a[23]*pow(u1, 3)*pow(v2, 2) + (1.0L/6.0L)*a[24]*pow(v1, 3) - 1.0L/6.0L*a[24]*pow(v2, 3) + (1.0L/6.0L)*a[25]*u1*pow(v1, 3) - 1.0L/6.0L*a[25]*u1*pow(v2, 3) + (1.0L/6.0L)*a[26]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[26]*pow(u1, 2)*pow(v2, 3) + (1.0L/6.0L)*a[27]*pow(u1, 3)*pow(v1, 3) - 1.0L/6.0L*a[27]*pow(u1, 3)*pow(v2, 3) + (1.0L/8.0L)*a[28]*pow(v1, 4) - 1.0L/8.0L*a[28]*pow(v2, 4) + (1.0L/8.0L)*a[29]*u1*pow(v1, 4) - 1.0L/8.0L*a[29]*u1*pow(v2, 4) + (1.0L/8.0L)*a[30]*pow(u1, 2)*pow(v1, 4) - 1.0L/8.0L*a[30]*pow(u1, 2)*pow(v2, 4) + (1.0L/8.0L)*a[31]*pow(u1, 3)*pow(v1, 4) - 1.0L/8.0L*a[31]*pow(u1, 3)*pow(v2, 4)) + w2*(a[0]*v1 - a[0]*v2 + a[1]*u1*v1 - a[1]*u1*v2 + (1.0L/3.0L)*a[10]*pow(u1, 2)*pow(v1, 3) - 1.0L/3.0L*a[10]*pow(u1, 2)*pow(v2, 3) + (1.0L/3.0L)*a[11]*pow(u1, 3)*pow(v1, 3) - 1.0L/3.0L*a[11]*pow(u1, 3)*pow(v2, 3) + (1.0L/4.0L)*a[12]*pow(v1, 4) - 1.0L/4.0L*a[12]*pow(v2, 4) + (1.0L/4.0L)*a[13]*u1*pow(v1, 4) - 1.0L/4.0L*a[13]*u1*pow(v2, 4) + (1.0L/4.0L)*a[14]*pow(u1, 2)*pow(v1, 4) - 1.0L/4.0L*a[14]*pow(u1, 2)*pow(v2, 4) + (1.0L/4.0L)*a[15]*pow(u1, 3)*pow(v1, 4) - 1.0L/4.0L*a[15]*pow(u1, 3)*pow(v2, 4) + a[2]*pow(u1, 2)*v1 - a[2]*pow(u1, 2)*v2 + a[3]*pow(u1, 3)*v1 - a[3]*pow(u1, 3)*v2 + (1.0L/2.0L)*a[4]*pow(v1, 2) - 1.0L/2.0L*a[4]*pow(v2, 2) + (1.0L/2.0L)*a[5]*u1*pow(v1, 2) - 1.0L/2.0L*a[5]*u1*pow(v2, 2) + (1.0L/2.0L)*a[6]*pow(u1, 2)*pow(v1, 2) - 1.0L/2.0L*a[6]*pow(u1, 2)*pow(v2, 2) + (1.0L/2.0L)*a[7]*pow(u1, 3)*pow(v1, 2) - 1.0L/2.0L*a[7]*pow(u1, 3)*pow(v2, 2) + (1.0L/3.0L)*a[8]*pow(v1, 3) - 1.0L/3.0L*a[8]*pow(v2, 3) + (1.0L/3.0L)*a[9]*u1*pow(v1, 3) - 1.0L/3.0L*a[9]*u1*pow(v2, 3));
    return G_result;
}

double Energy::gradientYMinComponent(const double*& a, double u1, double v1, double w1, double u2, double w2) {
    double G_result;
    G_result = -pow(w1, 4)*((1.0L/4.0L)*a[48]*u1 - 1.0L/4.0L*a[48]*u2 + (1.0L/8.0L)*a[49]*pow(u1, 2) - 1.0L/8.0L*a[49]*pow(u2, 2) + (1.0L/12.0L)*a[50]*pow(u1, 3) - 1.0L/12.0L*a[50]*pow(u2, 3) + (1.0L/16.0L)*a[51]*pow(u1, 4) - 1.0L/16.0L*a[51]*pow(u2, 4) + (1.0L/4.0L)*a[52]*u1*v1 - 1.0L/4.0L*a[52]*u2*v1 + (1.0L/8.0L)*a[53]*pow(u1, 2)*v1 - 1.0L/8.0L*a[53]*pow(u2, 2)*v1 + (1.0L/12.0L)*a[54]*pow(u1, 3)*v1 - 1.0L/12.0L*a[54]*pow(u2, 3)*v1 + (1.0L/16.0L)*a[55]*pow(u1, 4)*v1 - 1.0L/16.0L*a[55]*pow(u2, 4)*v1 + (1.0L/4.0L)*a[56]*u1*pow(v1, 2) - 1.0L/4.0L*a[56]*u2*pow(v1, 2) + (1.0L/8.0L)*a[57]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[57]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[58]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[58]*pow(u2, 3)*pow(v1, 2) + (1.0L/16.0L)*a[59]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[59]*pow(u2, 4)*pow(v1, 2) + (1.0L/4.0L)*a[60]*u1*pow(v1, 3) - 1.0L/4.0L*a[60]*u2*pow(v1, 3) + (1.0L/8.0L)*a[61]*pow(u1, 2)*pow(v1, 3) - 1.0L/8.0L*a[61]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[62]*pow(u1, 3)*pow(v1, 3) - 1.0L/12.0L*a[62]*pow(u2, 3)*pow(v1, 3) + (1.0L/16.0L)*a[63]*pow(u1, 4)*pow(v1, 3) - 1.0L/16.0L*a[63]*pow(u2, 4)*pow(v1, 3)) - pow(w1, 3)*((1.0L/3.0L)*a[32]*u1 - 1.0L/3.0L*a[32]*u2 + (1.0L/6.0L)*a[33]*pow(u1, 2) - 1.0L/6.0L*a[33]*pow(u2, 2) + (1.0L/9.0L)*a[34]*pow(u1, 3) - 1.0L/9.0L*a[34]*pow(u2, 3) + (1.0L/12.0L)*a[35]*pow(u1, 4) - 1.0L/12.0L*a[35]*pow(u2, 4) + (1.0L/3.0L)*a[36]*u1*v1 - 1.0L/3.0L*a[36]*u2*v1 + (1.0L/6.0L)*a[37]*pow(u1, 2)*v1 - 1.0L/6.0L*a[37]*pow(u2, 2)*v1 + (1.0L/9.0L)*a[38]*pow(u1, 3)*v1 - 1.0L/9.0L*a[38]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[39]*pow(u1, 4)*v1 - 1.0L/12.0L*a[39]*pow(u2, 4)*v1 + (1.0L/3.0L)*a[40]*u1*pow(v1, 2) - 1.0L/3.0L*a[40]*u2*pow(v1, 2) + (1.0L/6.0L)*a[41]*pow(u1, 2)*pow(v1, 2) - 1.0L/6.0L*a[41]*pow(u2, 2)*pow(v1, 2) + (1.0L/9.0L)*a[42]*pow(u1, 3)*pow(v1, 2) - 1.0L/9.0L*a[42]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[43]*pow(u1, 4)*pow(v1, 2) - 1.0L/12.0L*a[43]*pow(u2, 4)*pow(v1, 2) + (1.0L/3.0L)*a[44]*u1*pow(v1, 3) - 1.0L/3.0L*a[44]*u2*pow(v1, 3) + (1.0L/6.0L)*a[45]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[45]*pow(u2, 2)*pow(v1, 3) + (1.0L/9.0L)*a[46]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[46]*pow(u2, 3)*pow(v1, 3) + (1.0L/12.0L)*a[47]*pow(u1, 4)*pow(v1, 3) - 1.0L/12.0L*a[47]*pow(u2, 4)*pow(v1, 3)) - pow(w1, 2)*((1.0L/2.0L)*a[16]*u1 - 1.0L/2.0L*a[16]*u2 + (1.0L/4.0L)*a[17]*pow(u1, 2) - 1.0L/4.0L*a[17]*pow(u2, 2) + (1.0L/6.0L)*a[18]*pow(u1, 3) - 1.0L/6.0L*a[18]*pow(u2, 3) + (1.0L/8.0L)*a[19]*pow(u1, 4) - 1.0L/8.0L*a[19]*pow(u2, 4) + (1.0L/2.0L)*a[20]*u1*v1 - 1.0L/2.0L*a[20]*u2*v1 + (1.0L/4.0L)*a[21]*pow(u1, 2)*v1 - 1.0L/4.0L*a[21]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[22]*pow(u1, 3)*v1 - 1.0L/6.0L*a[22]*pow(u2, 3)*v1 + (1.0L/8.0L)*a[23]*pow(u1, 4)*v1 - 1.0L/8.0L*a[23]*pow(u2, 4)*v1 + (1.0L/2.0L)*a[24]*u1*pow(v1, 2) - 1.0L/2.0L*a[24]*u2*pow(v1, 2) + (1.0L/4.0L)*a[25]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[25]*pow(u2, 2)*pow(v1, 2) + (1.0L/6.0L)*a[26]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[26]*pow(u2, 3)*pow(v1, 2) + (1.0L/8.0L)*a[27]*pow(u1, 4)*pow(v1, 2) - 1.0L/8.0L*a[27]*pow(u2, 4)*pow(v1, 2) + (1.0L/2.0L)*a[28]*u1*pow(v1, 3) - 1.0L/2.0L*a[28]*u2*pow(v1, 3) + (1.0L/4.0L)*a[29]*pow(u1, 2)*pow(v1, 3) - 1.0L/4.0L*a[29]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[30]*pow(u1, 3)*pow(v1, 3) - 1.0L/6.0L*a[30]*pow(u2, 3)*pow(v1, 3) + (1.0L/8.0L)*a[31]*pow(u1, 4)*pow(v1, 3) - 1.0L/8.0L*a[31]*pow(u2, 4)*pow(v1, 3)) - w1*(a[0]*u1 - a[0]*u2 + (1.0L/2.0L)*a[1]*pow(u1, 2) - 1.0L/2.0L*a[1]*pow(u2, 2) + (1.0L/3.0L)*a[10]*pow(u1, 3)*pow(v1, 2) - 1.0L/3.0L*a[10]*pow(u2, 3)*pow(v1, 2) + (1.0L/4.0L)*a[11]*pow(u1, 4)*pow(v1, 2) - 1.0L/4.0L*a[11]*pow(u2, 4)*pow(v1, 2) + a[12]*u1*pow(v1, 3) - a[12]*u2*pow(v1, 3) + (1.0L/2.0L)*a[13]*pow(u1, 2)*pow(v1, 3) - 1.0L/2.0L*a[13]*pow(u2, 2)*pow(v1, 3) + (1.0L/3.0L)*a[14]*pow(u1, 3)*pow(v1, 3) - 1.0L/3.0L*a[14]*pow(u2, 3)*pow(v1, 3) + (1.0L/4.0L)*a[15]*pow(u1, 4)*pow(v1, 3) - 1.0L/4.0L*a[15]*pow(u2, 4)*pow(v1, 3) + (1.0L/3.0L)*a[2]*pow(u1, 3) - 1.0L/3.0L*a[2]*pow(u2, 3) + (1.0L/4.0L)*a[3]*pow(u1, 4) - 1.0L/4.0L*a[3]*pow(u2, 4) + a[4]*u1*v1 - a[4]*u2*v1 + (1.0L/2.0L)*a[5]*pow(u1, 2)*v1 - 1.0L/2.0L*a[5]*pow(u2, 2)*v1 + (1.0L/3.0L)*a[6]*pow(u1, 3)*v1 - 1.0L/3.0L*a[6]*pow(u2, 3)*v1 + (1.0L/4.0L)*a[7]*pow(u1, 4)*v1 - 1.0L/4.0L*a[7]*pow(u2, 4)*v1 + a[8]*u1*pow(v1, 2) - a[8]*u2*pow(v1, 2) + (1.0L/2.0L)*a[9]*pow(u1, 2)*pow(v1, 2) - 1.0L/2.0L*a[9]*pow(u2, 2)*pow(v1, 2)) + pow(w2, 4)*((1.0L/4.0L)*a[48]*u1 - 1.0L/4.0L*a[48]*u2 + (1.0L/8.0L)*a[49]*pow(u1, 2) - 1.0L/8.0L*a[49]*pow(u2, 2) + (1.0L/12.0L)*a[50]*pow(u1, 3) - 1.0L/12.0L*a[50]*pow(u2, 3) + (1.0L/16.0L)*a[51]*pow(u1, 4) - 1.0L/16.0L*a[51]*pow(u2, 4) + (1.0L/4.0L)*a[52]*u1*v1 - 1.0L/4.0L*a[52]*u2*v1 + (1.0L/8.0L)*a[53]*pow(u1, 2)*v1 - 1.0L/8.0L*a[53]*pow(u2, 2)*v1 + (1.0L/12.0L)*a[54]*pow(u1, 3)*v1 - 1.0L/12.0L*a[54]*pow(u2, 3)*v1 + (1.0L/16.0L)*a[55]*pow(u1, 4)*v1 - 1.0L/16.0L*a[55]*pow(u2, 4)*v1 + (1.0L/4.0L)*a[56]*u1*pow(v1, 2) - 1.0L/4.0L*a[56]*u2*pow(v1, 2) + (1.0L/8.0L)*a[57]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[57]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[58]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[58]*pow(u2, 3)*pow(v1, 2) + (1.0L/16.0L)*a[59]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[59]*pow(u2, 4)*pow(v1, 2) + (1.0L/4.0L)*a[60]*u1*pow(v1, 3) - 1.0L/4.0L*a[60]*u2*pow(v1, 3) + (1.0L/8.0L)*a[61]*pow(u1, 2)*pow(v1, 3) - 1.0L/8.0L*a[61]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[62]*pow(u1, 3)*pow(v1, 3) - 1.0L/12.0L*a[62]*pow(u2, 3)*pow(v1, 3) + (1.0L/16.0L)*a[63]*pow(u1, 4)*pow(v1, 3) - 1.0L/16.0L*a[63]*pow(u2, 4)*pow(v1, 3)) + pow(w2, 3)*((1.0L/3.0L)*a[32]*u1 - 1.0L/3.0L*a[32]*u2 + (1.0L/6.0L)*a[33]*pow(u1, 2) - 1.0L/6.0L*a[33]*pow(u2, 2) + (1.0L/9.0L)*a[34]*pow(u1, 3) - 1.0L/9.0L*a[34]*pow(u2, 3) + (1.0L/12.0L)*a[35]*pow(u1, 4) - 1.0L/12.0L*a[35]*pow(u2, 4) + (1.0L/3.0L)*a[36]*u1*v1 - 1.0L/3.0L*a[36]*u2*v1 + (1.0L/6.0L)*a[37]*pow(u1, 2)*v1 - 1.0L/6.0L*a[37]*pow(u2, 2)*v1 + (1.0L/9.0L)*a[38]*pow(u1, 3)*v1 - 1.0L/9.0L*a[38]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[39]*pow(u1, 4)*v1 - 1.0L/12.0L*a[39]*pow(u2, 4)*v1 + (1.0L/3.0L)*a[40]*u1*pow(v1, 2) - 1.0L/3.0L*a[40]*u2*pow(v1, 2) + (1.0L/6.0L)*a[41]*pow(u1, 2)*pow(v1, 2) - 1.0L/6.0L*a[41]*pow(u2, 2)*pow(v1, 2) + (1.0L/9.0L)*a[42]*pow(u1, 3)*pow(v1, 2) - 1.0L/9.0L*a[42]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[43]*pow(u1, 4)*pow(v1, 2) - 1.0L/12.0L*a[43]*pow(u2, 4)*pow(v1, 2) + (1.0L/3.0L)*a[44]*u1*pow(v1, 3) - 1.0L/3.0L*a[44]*u2*pow(v1, 3) + (1.0L/6.0L)*a[45]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[45]*pow(u2, 2)*pow(v1, 3) + (1.0L/9.0L)*a[46]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[46]*pow(u2, 3)*pow(v1, 3) + (1.0L/12.0L)*a[47]*pow(u1, 4)*pow(v1, 3) - 1.0L/12.0L*a[47]*pow(u2, 4)*pow(v1, 3)) + pow(w2, 2)*((1.0L/2.0L)*a[16]*u1 - 1.0L/2.0L*a[16]*u2 + (1.0L/4.0L)*a[17]*pow(u1, 2) - 1.0L/4.0L*a[17]*pow(u2, 2) + (1.0L/6.0L)*a[18]*pow(u1, 3) - 1.0L/6.0L*a[18]*pow(u2, 3) + (1.0L/8.0L)*a[19]*pow(u1, 4) - 1.0L/8.0L*a[19]*pow(u2, 4) + (1.0L/2.0L)*a[20]*u1*v1 - 1.0L/2.0L*a[20]*u2*v1 + (1.0L/4.0L)*a[21]*pow(u1, 2)*v1 - 1.0L/4.0L*a[21]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[22]*pow(u1, 3)*v1 - 1.0L/6.0L*a[22]*pow(u2, 3)*v1 + (1.0L/8.0L)*a[23]*pow(u1, 4)*v1 - 1.0L/8.0L*a[23]*pow(u2, 4)*v1 + (1.0L/2.0L)*a[24]*u1*pow(v1, 2) - 1.0L/2.0L*a[24]*u2*pow(v1, 2) + (1.0L/4.0L)*a[25]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[25]*pow(u2, 2)*pow(v1, 2) + (1.0L/6.0L)*a[26]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[26]*pow(u2, 3)*pow(v1, 2) + (1.0L/8.0L)*a[27]*pow(u1, 4)*pow(v1, 2) - 1.0L/8.0L*a[27]*pow(u2, 4)*pow(v1, 2) + (1.0L/2.0L)*a[28]*u1*pow(v1, 3) - 1.0L/2.0L*a[28]*u2*pow(v1, 3) + (1.0L/4.0L)*a[29]*pow(u1, 2)*pow(v1, 3) - 1.0L/4.0L*a[29]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[30]*pow(u1, 3)*pow(v1, 3) - 1.0L/6.0L*a[30]*pow(u2, 3)*pow(v1, 3) + (1.0L/8.0L)*a[31]*pow(u1, 4)*pow(v1, 3) - 1.0L/8.0L*a[31]*pow(u2, 4)*pow(v1, 3)) + w2*(a[0]*u1 - a[0]*u2 + (1.0L/2.0L)*a[1]*pow(u1, 2) - 1.0L/2.0L*a[1]*pow(u2, 2) + (1.0L/3.0L)*a[10]*pow(u1, 3)*pow(v1, 2) - 1.0L/3.0L*a[10]*pow(u2, 3)*pow(v1, 2) + (1.0L/4.0L)*a[11]*pow(u1, 4)*pow(v1, 2) - 1.0L/4.0L*a[11]*pow(u2, 4)*pow(v1, 2) + a[12]*u1*pow(v1, 3) - a[12]*u2*pow(v1, 3) + (1.0L/2.0L)*a[13]*pow(u1, 2)*pow(v1, 3) - 1.0L/2.0L*a[13]*pow(u2, 2)*pow(v1, 3) + (1.0L/3.0L)*a[14]*pow(u1, 3)*pow(v1, 3) - 1.0L/3.0L*a[14]*pow(u2, 3)*pow(v1, 3) + (1.0L/4.0L)*a[15]*pow(u1, 4)*pow(v1, 3) - 1.0L/4.0L*a[15]*pow(u2, 4)*pow(v1, 3) + (1.0L/3.0L)*a[2]*pow(u1, 3) - 1.0L/3.0L*a[2]*pow(u2, 3) + (1.0L/4.0L)*a[3]*pow(u1, 4) - 1.0L/4.0L*a[3]*pow(u2, 4) + a[4]*u1*v1 - a[4]*u2*v1 + (1.0L/2.0L)*a[5]*pow(u1, 2)*v1 - 1.0L/2.0L*a[5]*pow(u2, 2)*v1 + (1.0L/3.0L)*a[6]*pow(u1, 3)*v1 - 1.0L/3.0L*a[6]*pow(u2, 3)*v1 + (1.0L/4.0L)*a[7]*pow(u1, 4)*v1 - 1.0L/4.0L*a[7]*pow(u2, 4)*v1 + a[8]*u1*pow(v1, 2) - a[8]*u2*pow(v1, 2) + (1.0L/2.0L)*a[9]*pow(u1, 2)*pow(v1, 2) - 1.0L/2.0L*a[9]*pow(u2, 2)*pow(v1, 2));
    return G_result;
}

double Energy::gradientZMinComponent(const double*& a, double u1, double v1, double w1, double u2, double v2) {
    double G_result;
    G_result = -a[0]*u1*v1 + a[0]*u1*v2 + a[0]*u2*v1 - a[0]*u2*v2 - 1.0L/2.0L*a[1]*pow(u1, 2)*v1 + (1.0L/2.0L)*a[1]*pow(u1, 2)*v2 + (1.0L/2.0L)*a[1]*pow(u2, 2)*v1 - 1.0L/2.0L*a[1]*pow(u2, 2)*v2 - 1.0L/9.0L*a[10]*pow(u1, 3)*pow(v1, 3) + (1.0L/9.0L)*a[10]*pow(u1, 3)*pow(v2, 3) + (1.0L/9.0L)*a[10]*pow(u2, 3)*pow(v1, 3) - 1.0L/9.0L*a[10]*pow(u2, 3)*pow(v2, 3) - 1.0L/12.0L*a[11]*pow(u1, 4)*pow(v1, 3) + (1.0L/12.0L)*a[11]*pow(u1, 4)*pow(v2, 3) + (1.0L/12.0L)*a[11]*pow(u2, 4)*pow(v1, 3) - 1.0L/12.0L*a[11]*pow(u2, 4)*pow(v2, 3) - 1.0L/4.0L*a[12]*u1*pow(v1, 4) + (1.0L/4.0L)*a[12]*u1*pow(v2, 4) + (1.0L/4.0L)*a[12]*u2*pow(v1, 4) - 1.0L/4.0L*a[12]*u2*pow(v2, 4) - 1.0L/8.0L*a[13]*pow(u1, 2)*pow(v1, 4) + (1.0L/8.0L)*a[13]*pow(u1, 2)*pow(v2, 4) + (1.0L/8.0L)*a[13]*pow(u2, 2)*pow(v1, 4) - 1.0L/8.0L*a[13]*pow(u2, 2)*pow(v2, 4) - 1.0L/12.0L*a[14]*pow(u1, 3)*pow(v1, 4) + (1.0L/12.0L)*a[14]*pow(u1, 3)*pow(v2, 4) + (1.0L/12.0L)*a[14]*pow(u2, 3)*pow(v1, 4) - 1.0L/12.0L*a[14]*pow(u2, 3)*pow(v2, 4) - 1.0L/16.0L*a[15]*pow(u1, 4)*pow(v1, 4) + (1.0L/16.0L)*a[15]*pow(u1, 4)*pow(v2, 4) + (1.0L/16.0L)*a[15]*pow(u2, 4)*pow(v1, 4) - 1.0L/16.0L*a[15]*pow(u2, 4)*pow(v2, 4) - 1.0L/3.0L*a[2]*pow(u1, 3)*v1 + (1.0L/3.0L)*a[2]*pow(u1, 3)*v2 + (1.0L/3.0L)*a[2]*pow(u2, 3)*v1 - 1.0L/3.0L*a[2]*pow(u2, 3)*v2 - 1.0L/4.0L*a[3]*pow(u1, 4)*v1 + (1.0L/4.0L)*a[3]*pow(u1, 4)*v2 + (1.0L/4.0L)*a[3]*pow(u2, 4)*v1 - 1.0L/4.0L*a[3]*pow(u2, 4)*v2 - 1.0L/2.0L*a[4]*u1*pow(v1, 2) + (1.0L/2.0L)*a[4]*u1*pow(v2, 2) + (1.0L/2.0L)*a[4]*u2*pow(v1, 2) - 1.0L/2.0L*a[4]*u2*pow(v2, 2) - 1.0L/4.0L*a[5]*pow(u1, 2)*pow(v1, 2) + (1.0L/4.0L)*a[5]*pow(u1, 2)*pow(v2, 2) + (1.0L/4.0L)*a[5]*pow(u2, 2)*pow(v1, 2) - 1.0L/4.0L*a[5]*pow(u2, 2)*pow(v2, 2) - 1.0L/6.0L*a[6]*pow(u1, 3)*pow(v1, 2) + (1.0L/6.0L)*a[6]*pow(u1, 3)*pow(v2, 2) + (1.0L/6.0L)*a[6]*pow(u2, 3)*pow(v1, 2) - 1.0L/6.0L*a[6]*pow(u2, 3)*pow(v2, 2) - 1.0L/8.0L*a[7]*pow(u1, 4)*pow(v1, 2) + (1.0L/8.0L)*a[7]*pow(u1, 4)*pow(v2, 2) + (1.0L/8.0L)*a[7]*pow(u2, 4)*pow(v1, 2) - 1.0L/8.0L*a[7]*pow(u2, 4)*pow(v2, 2) - 1.0L/3.0L*a[8]*u1*pow(v1, 3) + (1.0L/3.0L)*a[8]*u1*pow(v2, 3) + (1.0L/3.0L)*a[8]*u2*pow(v1, 3) - 1.0L/3.0L*a[8]*u2*pow(v2, 3) - 1.0L/6.0L*a[9]*pow(u1, 2)*pow(v1, 3) + (1.0L/6.0L)*a[9]*pow(u1, 2)*pow(v2, 3) + (1.0L/6.0L)*a[9]*pow(u2, 2)*pow(v1, 3) - 1.0L/6.0L*a[9]*pow(u2, 2)*pow(v2, 3) - 4*pow(w1, 3)*((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 + (1.0L/8.0L)*a[49]*pow(u1, 2)*v1 - 1.0L/8.0L*a[49]*pow(u1, 2)*v2 - 1.0L/8.0L*a[49]*pow(u2, 2)*v1 + (1.0L/8.0L)*a[49]*pow(u2, 2)*v2 + (1.0L/12.0L)*a[50]*pow(u1, 3)*v1 - 1.0L/12.0L*a[50]*pow(u1, 3)*v2 - 1.0L/12.0L*a[50]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[50]*pow(u2, 3)*v2 + (1.0L/16.0L)*a[51]*pow(u1, 4)*v1 - 1.0L/16.0L*a[51]*pow(u1, 4)*v2 - 1.0L/16.0L*a[51]*pow(u2, 4)*v1 + (1.0L/16.0L)*a[51]*pow(u2, 4)*v2 + (1.0L/8.0L)*a[52]*u1*pow(v1, 2) - 1.0L/8.0L*a[52]*u1*pow(v2, 2) - 1.0L/8.0L*a[52]*u2*pow(v1, 2) + (1.0L/8.0L)*a[52]*u2*pow(v2, 2) + (1.0L/16.0L)*a[53]*pow(u1, 2)*pow(v1, 2) - 1.0L/16.0L*a[53]*pow(u1, 2)*pow(v2, 2) - 1.0L/16.0L*a[53]*pow(u2, 2)*pow(v1, 2) + (1.0L/16.0L)*a[53]*pow(u2, 2)*pow(v2, 2) + (1.0L/24.0L)*a[54]*pow(u1, 3)*pow(v1, 2) - 1.0L/24.0L*a[54]*pow(u1, 3)*pow(v2, 2) - 1.0L/24.0L*a[54]*pow(u2, 3)*pow(v1, 2) + (1.0L/24.0L)*a[54]*pow(u2, 3)*pow(v2, 2) + (1.0L/32.0L)*a[55]*pow(u1, 4)*pow(v1, 2) - 1.0L/32.0L*a[55]*pow(u1, 4)*pow(v2, 2) - 1.0L/32.0L*a[55]*pow(u2, 4)*pow(v1, 2) + (1.0L/32.0L)*a[55]*pow(u2, 4)*pow(v2, 2) + (1.0L/12.0L)*a[56]*u1*pow(v1, 3) - 1.0L/12.0L*a[56]*u1*pow(v2, 3) - 1.0L/12.0L*a[56]*u2*pow(v1, 3) + (1.0L/12.0L)*a[56]*u2*pow(v2, 3) + (1.0L/24.0L)*a[57]*pow(u1, 2)*pow(v1, 3) - 1.0L/24.0L*a[57]*pow(u1, 2)*pow(v2, 3) - 1.0L/24.0L*a[57]*pow(u2, 2)*pow(v1, 3) + (1.0L/24.0L)*a[57]*pow(u2, 2)*pow(v2, 3) + (1.0L/36.0L)*a[58]*pow(u1, 3)*pow(v1, 3) - 1.0L/36.0L*a[58]*pow(u1, 3)*pow(v2, 3) - 1.0L/36.0L*a[58]*pow(u2, 3)*pow(v1, 3) + (1.0L/36.0L)*a[58]*pow(u2, 3)*pow(v2, 3) + (1.0L/48.0L)*a[59]*pow(u1, 4)*pow(v1, 3) - 1.0L/48.0L*a[59]*pow(u1, 4)*pow(v2, 3) - 1.0L/48.0L*a[59]*pow(u2, 4)*pow(v1, 3) + (1.0L/48.0L)*a[59]*pow(u2, 4)*pow(v2, 3) + (1.0L/16.0L)*a[60]*u1*pow(v1, 4) - 1.0L/16.0L*a[60]*u1*pow(v2, 4) - 1.0L/16.0L*a[60]*u2*pow(v1, 4) + (1.0L/16.0L)*a[60]*u2*pow(v2, 4) + (1.0L/32.0L)*a[61]*pow(u1, 2)*pow(v1, 4) - 1.0L/32.0L*a[61]*pow(u1, 2)*pow(v2, 4) - 1.0L/32.0L*a[61]*pow(u2, 2)*pow(v1, 4) + (1.0L/32.0L)*a[61]*pow(u2, 2)*pow(v2, 4) + (1.0L/48.0L)*a[62]*pow(u1, 3)*pow(v1, 4) - 1.0L/48.0L*a[62]*pow(u1, 3)*pow(v2, 4) - 1.0L/48.0L*a[62]*pow(u2, 3)*pow(v1, 4) + (1.0L/48.0L)*a[62]*pow(u2, 3)*pow(v2, 4) + (1.0L/64.0L)*a[63]*pow(u1, 4)*pow(v1, 4) - 1.0L/64.0L*a[63]*pow(u1, 4)*pow(v2, 4) - 1.0L/64.0L*a[63]*pow(u2, 4)*pow(v1, 4) + (1.0L/64.0L)*a[63]*pow(u2, 4)*pow(v2, 4)) - 3*pow(w1, 2)*((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 + (1.0L/6.0L)*a[33]*pow(u1, 2)*v1 - 1.0L/6.0L*a[33]*pow(u1, 2)*v2 - 1.0L/6.0L*a[33]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[33]*pow(u2, 2)*v2 + (1.0L/9.0L)*a[34]*pow(u1, 3)*v1 - 1.0L/9.0L*a[34]*pow(u1, 3)*v2 - 1.0L/9.0L*a[34]*pow(u2, 3)*v1 + (1.0L/9.0L)*a[34]*pow(u2, 3)*v2 + (1.0L/12.0L)*a[35]*pow(u1, 4)*v1 - 1.0L/12.0L*a[35]*pow(u1, 4)*v2 - 1.0L/12.0L*a[35]*pow(u2, 4)*v1 + (1.0L/12.0L)*a[35]*pow(u2, 4)*v2 + (1.0L/6.0L)*a[36]*u1*pow(v1, 2) - 1.0L/6.0L*a[36]*u1*pow(v2, 2) - 1.0L/6.0L*a[36]*u2*pow(v1, 2) + (1.0L/6.0L)*a[36]*u2*pow(v2, 2) + (1.0L/12.0L)*a[37]*pow(u1, 2)*pow(v1, 2) - 1.0L/12.0L*a[37]*pow(u1, 2)*pow(v2, 2) - 1.0L/12.0L*a[37]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[37]*pow(u2, 2)*pow(v2, 2) + (1.0L/18.0L)*a[38]*pow(u1, 3)*pow(v1, 2) - 1.0L/18.0L*a[38]*pow(u1, 3)*pow(v2, 2) - 1.0L/18.0L*a[38]*pow(u2, 3)*pow(v1, 2) + (1.0L/18.0L)*a[38]*pow(u2, 3)*pow(v2, 2) + (1.0L/24.0L)*a[39]*pow(u1, 4)*pow(v1, 2) - 1.0L/24.0L*a[39]*pow(u1, 4)*pow(v2, 2) - 1.0L/24.0L*a[39]*pow(u2, 4)*pow(v1, 2) + (1.0L/24.0L)*a[39]*pow(u2, 4)*pow(v2, 2) + (1.0L/9.0L)*a[40]*u1*pow(v1, 3) - 1.0L/9.0L*a[40]*u1*pow(v2, 3) - 1.0L/9.0L*a[40]*u2*pow(v1, 3) + (1.0L/9.0L)*a[40]*u2*pow(v2, 3) + (1.0L/18.0L)*a[41]*pow(u1, 2)*pow(v1, 3) - 1.0L/18.0L*a[41]*pow(u1, 2)*pow(v2, 3) - 1.0L/18.0L*a[41]*pow(u2, 2)*pow(v1, 3) + (1.0L/18.0L)*a[41]*pow(u2, 2)*pow(v2, 3) + (1.0L/27.0L)*a[42]*pow(u1, 3)*pow(v1, 3) - 1.0L/27.0L*a[42]*pow(u1, 3)*pow(v2, 3) - 1.0L/27.0L*a[42]*pow(u2, 3)*pow(v1, 3) + (1.0L/27.0L)*a[42]*pow(u2, 3)*pow(v2, 3) + (1.0L/36.0L)*a[43]*pow(u1, 4)*pow(v1, 3) - 1.0L/36.0L*a[43]*pow(u1, 4)*pow(v2, 3) - 1.0L/36.0L*a[43]*pow(u2, 4)*pow(v1, 3) + (1.0L/36.0L)*a[43]*pow(u2, 4)*pow(v2, 3) + (1.0L/12.0L)*a[44]*u1*pow(v1, 4) - 1.0L/12.0L*a[44]*u1*pow(v2, 4) - 1.0L/12.0L*a[44]*u2*pow(v1, 4) + (1.0L/12.0L)*a[44]*u2*pow(v2, 4) + (1.0L/24.0L)*a[45]*pow(u1, 2)*pow(v1, 4) - 1.0L/24.0L*a[45]*pow(u1, 2)*pow(v2, 4) - 1.0L/24.0L*a[45]*pow(u2, 2)*pow(v1, 4) + (1.0L/24.0L)*a[45]*pow(u2, 2)*pow(v2, 4) + (1.0L/36.0L)*a[46]*pow(u1, 3)*pow(v1, 4) - 1.0L/36.0L*a[46]*pow(u1, 3)*pow(v2, 4) - 1.0L/36.0L*a[46]*pow(u2, 3)*pow(v1, 4) + (1.0L/36.0L)*a[46]*pow(u2, 3)*pow(v2, 4) + (1.0L/48.0L)*a[47]*pow(u1, 4)*pow(v1, 4) - 1.0L/48.0L*a[47]*pow(u1, 4)*pow(v2, 4) - 1.0L/48.0L*a[47]*pow(u2, 4)*pow(v1, 4) + (1.0L/48.0L)*a[47]*pow(u2, 4)*pow(v2, 4)) - 2*w1*((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 + (1.0L/4.0L)*a[17]*pow(u1, 2)*v1 - 1.0L/4.0L*a[17]*pow(u1, 2)*v2 - 1.0L/4.0L*a[17]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[17]*pow(u2, 2)*v2 + (1.0L/6.0L)*a[18]*pow(u1, 3)*v1 - 1.0L/6.0L*a[18]*pow(u1, 3)*v2 - 1.0L/6.0L*a[18]*pow(u2, 3)*v1 + (1.0L/6.0L)*a[18]*pow(u2, 3)*v2 + (1.0L/8.0L)*a[19]*pow(u1, 4)*v1 - 1.0L/8.0L*a[19]*pow(u1, 4)*v2 - 1.0L/8.0L*a[19]*pow(u2, 4)*v1 + (1.0L/8.0L)*a[19]*pow(u2, 4)*v2 + (1.0L/4.0L)*a[20]*u1*pow(v1, 2) - 1.0L/4.0L*a[20]*u1*pow(v2, 2) - 1.0L/4.0L*a[20]*u2*pow(v1, 2) + (1.0L/4.0L)*a[20]*u2*pow(v2, 2) + (1.0L/8.0L)*a[21]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[21]*pow(u1, 2)*pow(v2, 2) - 1.0L/8.0L*a[21]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[21]*pow(u2, 2)*pow(v2, 2) + (1.0L/12.0L)*a[22]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[22]*pow(u1, 3)*pow(v2, 2) - 1.0L/12.0L*a[22]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[22]*pow(u2, 3)*pow(v2, 2) + (1.0L/16.0L)*a[23]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[23]*pow(u1, 4)*pow(v2, 2) - 1.0L/16.0L*a[23]*pow(u2, 4)*pow(v1, 2) + (1.0L/16.0L)*a[23]*pow(u2, 4)*pow(v2, 2) + (1.0L/6.0L)*a[24]*u1*pow(v1, 3) - 1.0L/6.0L*a[24]*u1*pow(v2, 3) - 1.0L/6.0L*a[24]*u2*pow(v1, 3) + (1.0L/6.0L)*a[24]*u2*pow(v2, 3) + (1.0L/12.0L)*a[25]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[25]*pow(u1, 2)*pow(v2, 3) - 1.0L/12.0L*a[25]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[25]*pow(u2, 2)*pow(v2, 3) + (1.0L/18.0L)*a[26]*pow(u1, 3)*pow(v1, 3) - 1.0L/18.0L*a[26]*pow(u1, 3)*pow(v2, 3) - 1.0L/18.0L*a[26]*pow(u2, 3)*pow(v1, 3) + (1.0L/18.0L)*a[26]*pow(u2, 3)*pow(v2, 3) + (1.0L/24.0L)*a[27]*pow(u1, 4)*pow(v1, 3) - 1.0L/24.0L*a[27]*pow(u1, 4)*pow(v2, 3) - 1.0L/24.0L*a[27]*pow(u2, 4)*pow(v1, 3) + (1.0L/24.0L)*a[27]*pow(u2, 4)*pow(v2, 3) + (1.0L/8.0L)*a[28]*u1*pow(v1, 4) - 1.0L/8.0L*a[28]*u1*pow(v2, 4) - 1.0L/8.0L*a[28]*u2*pow(v1, 4) + (1.0L/8.0L)*a[28]*u2*pow(v2, 4) + (1.0L/16.0L)*a[29]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[29]*pow(u1, 2)*pow(v2, 4) - 1.0L/16.0L*a[29]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[29]*pow(u2, 2)*pow(v2, 4) + (1.0L/24.0L)*a[30]*pow(u1, 3)*pow(v1, 4) - 1.0L/24.0L*a[30]*pow(u1, 3)*pow(v2, 4) - 1.0L/24.0L*a[30]*pow(u2, 3)*pow(v1, 4) + (1.0L/24.0L)*a[30]*pow(u2, 3)*pow(v2, 4) + (1.0L/32.0L)*a[31]*pow(u1, 4)*pow(v1, 4) - 1.0L/32.0L*a[31]*pow(u1, 4)*pow(v2, 4) - 1.0L/32.0L*a[31]*pow(u2, 4)*pow(v1, 4) + (1.0L/32.0L)*a[31]*pow(u2, 4)*pow(v2, 4));
    return G_result;
}

double Energy::gradientXMaxComponent(const double*& a, double v1, double w1, double u2, double v2, double w2) {
    double G_result;
    G_result = -pow(w1, 4)*(-1.0L/4.0L*a[48]*v1 + (1.0L/4.0L)*a[48]*v2 - 1.0L/4.0L*a[49]*u2*v1 + (1.0L/4.0L)*a[49]*u2*v2 - 1.0L/4.0L*a[50]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[50]*pow(u2, 2)*v2 - 1.0L/4.0L*a[51]*pow(u2, 3)*v1 + (1.0L/4.0L)*a[51]*pow(u2, 3)*v2 - 1.0L/8.0L*a[52]*pow(v1, 2) + (1.0L/8.0L)*a[52]*pow(v2, 2) - 1.0L/8.0L*a[53]*u2*pow(v1, 2) + (1.0L/8.0L)*a[53]*u2*pow(v2, 2) - 1.0L/8.0L*a[54]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[54]*pow(u2, 2)*pow(v2, 2) - 1.0L/8.0L*a[55]*pow(u2, 3)*pow(v1, 2) + (1.0L/8.0L)*a[55]*pow(u2, 3)*pow(v2, 2) - 1.0L/12.0L*a[56]*pow(v1, 3) + (1.0L/12.0L)*a[56]*pow(v2, 3) - 1.0L/12.0L*a[57]*u2*pow(v1, 3) + (1.0L/12.0L)*a[57]*u2*pow(v2, 3) - 1.0L/12.0L*a[58]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[58]*pow(u2, 2)*pow(v2, 3) - 1.0L/12.0L*a[59]*pow(u2, 3)*pow(v1, 3) + (1.0L/12.0L)*a[59]*pow(u2, 3)*pow(v2, 3) - 1.0L/16.0L*a[60]*pow(v1, 4) + (1.0L/16.0L)*a[60]*pow(v2, 4) - 1.0L/16.0L*a[61]*u2*pow(v1, 4) + (1.0L/16.0L)*a[61]*u2*pow(v2, 4) - 1.0L/16.0L*a[62]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[62]*pow(u2, 2)*pow(v2, 4) - 1.0L/16.0L*a[63]*pow(u2, 3)*pow(v1, 4) + (1.0L/16.0L)*a[63]*pow(u2, 3)*pow(v2, 4)) - pow(w1, 3)*(-1.0L/3.0L*a[32]*v1 + (1.0L/3.0L)*a[32]*v2 - 1.0L/3.0L*a[33]*u2*v1 + (1.0L/3.0L)*a[33]*u2*v2 - 1.0L/3.0L*a[34]*pow(u2, 2)*v1 + (1.0L/3.0L)*a[34]*pow(u2, 2)*v2 - 1.0L/3.0L*a[35]*pow(u2, 3)*v1 + (1.0L/3.0L)*a[35]*pow(u2, 3)*v2 - 1.0L/6.0L*a[36]*pow(v1, 2) + (1.0L/6.0L)*a[36]*pow(v2, 2) - 1.0L/6.0L*a[37]*u2*pow(v1, 2) + (1.0L/6.0L)*a[37]*u2*pow(v2, 2) - 1.0L/6.0L*a[38]*pow(u2, 2)*pow(v1, 2) + (1.0L/6.0L)*a[38]*pow(u2, 2)*pow(v2, 2) - 1.0L/6.0L*a[39]*pow(u2, 3)*pow(v1, 2) + (1.0L/6.0L)*a[39]*pow(u2, 3)*pow(v2, 2) - 1.0L/9.0L*a[40]*pow(v1, 3) + (1.0L/9.0L)*a[40]*pow(v2, 3) - 1.0L/9.0L*a[41]*u2*pow(v1, 3) + (1.0L/9.0L)*a[41]*u2*pow(v2, 3) - 1.0L/9.0L*a[42]*pow(u2, 2)*pow(v1, 3) + (1.0L/9.0L)*a[42]*pow(u2, 2)*pow(v2, 3) - 1.0L/9.0L*a[43]*pow(u2, 3)*pow(v1, 3) + (1.0L/9.0L)*a[43]*pow(u2, 3)*pow(v2, 3) - 1.0L/12.0L*a[44]*pow(v1, 4) + (1.0L/12.0L)*a[44]*pow(v2, 4) - 1.0L/12.0L*a[45]*u2*pow(v1, 4) + (1.0L/12.0L)*a[45]*u2*pow(v2, 4) - 1.0L/12.0L*a[46]*pow(u2, 2)*pow(v1, 4) + (1.0L/12.0L)*a[46]*pow(u2, 2)*pow(v2, 4) - 1.0L/12.0L*a[47]*pow(u2, 3)*pow(v1, 4) + (1.0L/12.0L)*a[47]*pow(u2, 3)*pow(v2, 4)) - pow(w1, 2)*(-1.0L/2.0L*a[16]*v1 + (1.0L/2.0L)*a[16]*v2 - 1.0L/2.0L*a[17]*u2*v1 + (1.0L/2.0L)*a[17]*u2*v2 - 1.0L/2.0L*a[18]*pow(u2, 2)*v1 + (1.0L/2.0L)*a[18]*pow(u2, 2)*v2 - 1.0L/2.0L*a[19]*pow(u2, 3)*v1 + (1.0L/2.0L)*a[19]*pow(u2, 3)*v2 - 1.0L/4.0L*a[20]*pow(v1, 2) + (1.0L/4.0L)*a[20]*pow(v2, 2) - 1.0L/4.0L*a[21]*u2*pow(v1, 2) + (1.0L/4.0L)*a[21]*u2*pow(v2, 2) - 1.0L/4.0L*a[22]*pow(u2, 2)*pow(v1, 2) + (1.0L/4.0L)*a[22]*pow(u2, 2)*pow(v2, 2) - 1.0L/4.0L*a[23]*pow(u2, 3)*pow(v1, 2) + (1.0L/4.0L)*a[23]*pow(u2, 3)*pow(v2, 2) - 1.0L/6.0L*a[24]*pow(v1, 3) + (1.0L/6.0L)*a[24]*pow(v2, 3) - 1.0L/6.0L*a[25]*u2*pow(v1, 3) + (1.0L/6.0L)*a[25]*u2*pow(v2, 3) - 1.0L/6.0L*a[26]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[26]*pow(u2, 2)*pow(v2, 3) - 1.0L/6.0L*a[27]*pow(u2, 3)*pow(v1, 3) + (1.0L/6.0L)*a[27]*pow(u2, 3)*pow(v2, 3) - 1.0L/8.0L*a[28]*pow(v1, 4) + (1.0L/8.0L)*a[28]*pow(v2, 4) - 1.0L/8.0L*a[29]*u2*pow(v1, 4) + (1.0L/8.0L)*a[29]*u2*pow(v2, 4) - 1.0L/8.0L*a[30]*pow(u2, 2)*pow(v1, 4) + (1.0L/8.0L)*a[30]*pow(u2, 2)*pow(v2, 4) - 1.0L/8.0L*a[31]*pow(u2, 3)*pow(v1, 4) + (1.0L/8.0L)*a[31]*pow(u2, 3)*pow(v2, 4)) - w1*(-a[0]*v1 + a[0]*v2 - a[1]*u2*v1 + a[1]*u2*v2 - 1.0L/3.0L*a[10]*pow(u2, 2)*pow(v1, 3) + (1.0L/3.0L)*a[10]*pow(u2, 2)*pow(v2, 3) - 1.0L/3.0L*a[11]*pow(u2, 3)*pow(v1, 3) + (1.0L/3.0L)*a[11]*pow(u2, 3)*pow(v2, 3) - 1.0L/4.0L*a[12]*pow(v1, 4) + (1.0L/4.0L)*a[12]*pow(v2, 4) - 1.0L/4.0L*a[13]*u2*pow(v1, 4) + (1.0L/4.0L)*a[13]*u2*pow(v2, 4) - 1.0L/4.0L*a[14]*pow(u2, 2)*pow(v1, 4) + (1.0L/4.0L)*a[14]*pow(u2, 2)*pow(v2, 4) - 1.0L/4.0L*a[15]*pow(u2, 3)*pow(v1, 4) + (1.0L/4.0L)*a[15]*pow(u2, 3)*pow(v2, 4) - a[2]*pow(u2, 2)*v1 + a[2]*pow(u2, 2)*v2 - a[3]*pow(u2, 3)*v1 + a[3]*pow(u2, 3)*v2 - 1.0L/2.0L*a[4]*pow(v1, 2) + (1.0L/2.0L)*a[4]*pow(v2, 2) - 1.0L/2.0L*a[5]*u2*pow(v1, 2) + (1.0L/2.0L)*a[5]*u2*pow(v2, 2) - 1.0L/2.0L*a[6]*pow(u2, 2)*pow(v1, 2) + (1.0L/2.0L)*a[6]*pow(u2, 2)*pow(v2, 2) - 1.0L/2.0L*a[7]*pow(u2, 3)*pow(v1, 2) + (1.0L/2.0L)*a[7]*pow(u2, 3)*pow(v2, 2) - 1.0L/3.0L*a[8]*pow(v1, 3) + (1.0L/3.0L)*a[8]*pow(v2, 3) - 1.0L/3.0L*a[9]*u2*pow(v1, 3) + (1.0L/3.0L)*a[9]*u2*pow(v2, 3)) + pow(w2, 4)*(-1.0L/4.0L*a[48]*v1 + (1.0L/4.0L)*a[48]*v2 - 1.0L/4.0L*a[49]*u2*v1 + (1.0L/4.0L)*a[49]*u2*v2 - 1.0L/4.0L*a[50]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[50]*pow(u2, 2)*v2 - 1.0L/4.0L*a[51]*pow(u2, 3)*v1 + (1.0L/4.0L)*a[51]*pow(u2, 3)*v2 - 1.0L/8.0L*a[52]*pow(v1, 2) + (1.0L/8.0L)*a[52]*pow(v2, 2) - 1.0L/8.0L*a[53]*u2*pow(v1, 2) + (1.0L/8.0L)*a[53]*u2*pow(v2, 2) - 1.0L/8.0L*a[54]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[54]*pow(u2, 2)*pow(v2, 2) - 1.0L/8.0L*a[55]*pow(u2, 3)*pow(v1, 2) + (1.0L/8.0L)*a[55]*pow(u2, 3)*pow(v2, 2) - 1.0L/12.0L*a[56]*pow(v1, 3) + (1.0L/12.0L)*a[56]*pow(v2, 3) - 1.0L/12.0L*a[57]*u2*pow(v1, 3) + (1.0L/12.0L)*a[57]*u2*pow(v2, 3) - 1.0L/12.0L*a[58]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[58]*pow(u2, 2)*pow(v2, 3) - 1.0L/12.0L*a[59]*pow(u2, 3)*pow(v1, 3) + (1.0L/12.0L)*a[59]*pow(u2, 3)*pow(v2, 3) - 1.0L/16.0L*a[60]*pow(v1, 4) + (1.0L/16.0L)*a[60]*pow(v2, 4) - 1.0L/16.0L*a[61]*u2*pow(v1, 4) + (1.0L/16.0L)*a[61]*u2*pow(v2, 4) - 1.0L/16.0L*a[62]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[62]*pow(u2, 2)*pow(v2, 4) - 1.0L/16.0L*a[63]*pow(u2, 3)*pow(v1, 4) + (1.0L/16.0L)*a[63]*pow(u2, 3)*pow(v2, 4)) + pow(w2, 3)*(-1.0L/3.0L*a[32]*v1 + (1.0L/3.0L)*a[32]*v2 - 1.0L/3.0L*a[33]*u2*v1 + (1.0L/3.0L)*a[33]*u2*v2 - 1.0L/3.0L*a[34]*pow(u2, 2)*v1 + (1.0L/3.0L)*a[34]*pow(u2, 2)*v2 - 1.0L/3.0L*a[35]*pow(u2, 3)*v1 + (1.0L/3.0L)*a[35]*pow(u2, 3)*v2 - 1.0L/6.0L*a[36]*pow(v1, 2) + (1.0L/6.0L)*a[36]*pow(v2, 2) - 1.0L/6.0L*a[37]*u2*pow(v1, 2) + (1.0L/6.0L)*a[37]*u2*pow(v2, 2) - 1.0L/6.0L*a[38]*pow(u2, 2)*pow(v1, 2) + (1.0L/6.0L)*a[38]*pow(u2, 2)*pow(v2, 2) - 1.0L/6.0L*a[39]*pow(u2, 3)*pow(v1, 2) + (1.0L/6.0L)*a[39]*pow(u2, 3)*pow(v2, 2) - 1.0L/9.0L*a[40]*pow(v1, 3) + (1.0L/9.0L)*a[40]*pow(v2, 3) - 1.0L/9.0L*a[41]*u2*pow(v1, 3) + (1.0L/9.0L)*a[41]*u2*pow(v2, 3) - 1.0L/9.0L*a[42]*pow(u2, 2)*pow(v1, 3) + (1.0L/9.0L)*a[42]*pow(u2, 2)*pow(v2, 3) - 1.0L/9.0L*a[43]*pow(u2, 3)*pow(v1, 3) + (1.0L/9.0L)*a[43]*pow(u2, 3)*pow(v2, 3) - 1.0L/12.0L*a[44]*pow(v1, 4) + (1.0L/12.0L)*a[44]*pow(v2, 4) - 1.0L/12.0L*a[45]*u2*pow(v1, 4) + (1.0L/12.0L)*a[45]*u2*pow(v2, 4) - 1.0L/12.0L*a[46]*pow(u2, 2)*pow(v1, 4) + (1.0L/12.0L)*a[46]*pow(u2, 2)*pow(v2, 4) - 1.0L/12.0L*a[47]*pow(u2, 3)*pow(v1, 4) + (1.0L/12.0L)*a[47]*pow(u2, 3)*pow(v2, 4)) + pow(w2, 2)*(-1.0L/2.0L*a[16]*v1 + (1.0L/2.0L)*a[16]*v2 - 1.0L/2.0L*a[17]*u2*v1 + (1.0L/2.0L)*a[17]*u2*v2 - 1.0L/2.0L*a[18]*pow(u2, 2)*v1 + (1.0L/2.0L)*a[18]*pow(u2, 2)*v2 - 1.0L/2.0L*a[19]*pow(u2, 3)*v1 + (1.0L/2.0L)*a[19]*pow(u2, 3)*v2 - 1.0L/4.0L*a[20]*pow(v1, 2) + (1.0L/4.0L)*a[20]*pow(v2, 2) - 1.0L/4.0L*a[21]*u2*pow(v1, 2) + (1.0L/4.0L)*a[21]*u2*pow(v2, 2) - 1.0L/4.0L*a[22]*pow(u2, 2)*pow(v1, 2) + (1.0L/4.0L)*a[22]*pow(u2, 2)*pow(v2, 2) - 1.0L/4.0L*a[23]*pow(u2, 3)*pow(v1, 2) + (1.0L/4.0L)*a[23]*pow(u2, 3)*pow(v2, 2) - 1.0L/6.0L*a[24]*pow(v1, 3) + (1.0L/6.0L)*a[24]*pow(v2, 3) - 1.0L/6.0L*a[25]*u2*pow(v1, 3) + (1.0L/6.0L)*a[25]*u2*pow(v2, 3) - 1.0L/6.0L*a[26]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[26]*pow(u2, 2)*pow(v2, 3) - 1.0L/6.0L*a[27]*pow(u2, 3)*pow(v1, 3) + (1.0L/6.0L)*a[27]*pow(u2, 3)*pow(v2, 3) - 1.0L/8.0L*a[28]*pow(v1, 4) + (1.0L/8.0L)*a[28]*pow(v2, 4) - 1.0L/8.0L*a[29]*u2*pow(v1, 4) + (1.0L/8.0L)*a[29]*u2*pow(v2, 4) - 1.0L/8.0L*a[30]*pow(u2, 2)*pow(v1, 4) + (1.0L/8.0L)*a[30]*pow(u2, 2)*pow(v2, 4) - 1.0L/8.0L*a[31]*pow(u2, 3)*pow(v1, 4) + (1.0L/8.0L)*a[31]*pow(u2, 3)*pow(v2, 4)) + w2*(-a[0]*v1 + a[0]*v2 - a[1]*u2*v1 + a[1]*u2*v2 - 1.0L/3.0L*a[10]*pow(u2, 2)*pow(v1, 3) + (1.0L/3.0L)*a[10]*pow(u2, 2)*pow(v2, 3) - 1.0L/3.0L*a[11]*pow(u2, 3)*pow(v1, 3) + (1.0L/3.0L)*a[11]*pow(u2, 3)*pow(v2, 3) - 1.0L/4.0L*a[12]*pow(v1, 4) + (1.0L/4.0L)*a[12]*pow(v2, 4) - 1.0L/4.0L*a[13]*u2*pow(v1, 4) + (1.0L/4.0L)*a[13]*u2*pow(v2, 4) - 1.0L/4.0L*a[14]*pow(u2, 2)*pow(v1, 4) + (1.0L/4.0L)*a[14]*pow(u2, 2)*pow(v2, 4) - 1.0L/4.0L*a[15]*pow(u2, 3)*pow(v1, 4) + (1.0L/4.0L)*a[15]*pow(u2, 3)*pow(v2, 4) - a[2]*pow(u2, 2)*v1 + a[2]*pow(u2, 2)*v2 - a[3]*pow(u2, 3)*v1 + a[3]*pow(u2, 3)*v2 - 1.0L/2.0L*a[4]*pow(v1, 2) + (1.0L/2.0L)*a[4]*pow(v2, 2) - 1.0L/2.0L*a[5]*u2*pow(v1, 2) + (1.0L/2.0L)*a[5]*u2*pow(v2, 2) - 1.0L/2.0L*a[6]*pow(u2, 2)*pow(v1, 2) + (1.0L/2.0L)*a[6]*pow(u2, 2)*pow(v2, 2) - 1.0L/2.0L*a[7]*pow(u2, 3)*pow(v1, 2) + (1.0L/2.0L)*a[7]*pow(u2, 3)*pow(v2, 2) - 1.0L/3.0L*a[8]*pow(v1, 3) + (1.0L/3.0L)*a[8]*pow(v2, 3) - 1.0L/3.0L*a[9]*u2*pow(v1, 3) + (1.0L/3.0L)*a[9]*u2*pow(v2, 3));
    return G_result;
}

double Energy::gradientYMaxComponent(const double*& a, double u1, double w1, double u2, double v2, double w2) {
    double G_result;
    G_result = -pow(w1, 4)*(-1.0L/4.0L*a[48]*u1 + (1.0L/4.0L)*a[48]*u2 - 1.0L/8.0L*a[49]*pow(u1, 2) + (1.0L/8.0L)*a[49]*pow(u2, 2) - 1.0L/12.0L*a[50]*pow(u1, 3) + (1.0L/12.0L)*a[50]*pow(u2, 3) - 1.0L/16.0L*a[51]*pow(u1, 4) + (1.0L/16.0L)*a[51]*pow(u2, 4) - 1.0L/4.0L*a[52]*u1*v2 + (1.0L/4.0L)*a[52]*u2*v2 - 1.0L/8.0L*a[53]*pow(u1, 2)*v2 + (1.0L/8.0L)*a[53]*pow(u2, 2)*v2 - 1.0L/12.0L*a[54]*pow(u1, 3)*v2 + (1.0L/12.0L)*a[54]*pow(u2, 3)*v2 - 1.0L/16.0L*a[55]*pow(u1, 4)*v2 + (1.0L/16.0L)*a[55]*pow(u2, 4)*v2 - 1.0L/4.0L*a[56]*u1*pow(v2, 2) + (1.0L/4.0L)*a[56]*u2*pow(v2, 2) - 1.0L/8.0L*a[57]*pow(u1, 2)*pow(v2, 2) + (1.0L/8.0L)*a[57]*pow(u2, 2)*pow(v2, 2) - 1.0L/12.0L*a[58]*pow(u1, 3)*pow(v2, 2) + (1.0L/12.0L)*a[58]*pow(u2, 3)*pow(v2, 2) - 1.0L/16.0L*a[59]*pow(u1, 4)*pow(v2, 2) + (1.0L/16.0L)*a[59]*pow(u2, 4)*pow(v2, 2) - 1.0L/4.0L*a[60]*u1*pow(v2, 3) + (1.0L/4.0L)*a[60]*u2*pow(v2, 3) - 1.0L/8.0L*a[61]*pow(u1, 2)*pow(v2, 3) + (1.0L/8.0L)*a[61]*pow(u2, 2)*pow(v2, 3) - 1.0L/12.0L*a[62]*pow(u1, 3)*pow(v2, 3) + (1.0L/12.0L)*a[62]*pow(u2, 3)*pow(v2, 3) - 1.0L/16.0L*a[63]*pow(u1, 4)*pow(v2, 3) + (1.0L/16.0L)*a[63]*pow(u2, 4)*pow(v2, 3)) - pow(w1, 3)*(-1.0L/3.0L*a[32]*u1 + (1.0L/3.0L)*a[32]*u2 - 1.0L/6.0L*a[33]*pow(u1, 2) + (1.0L/6.0L)*a[33]*pow(u2, 2) - 1.0L/9.0L*a[34]*pow(u1, 3) + (1.0L/9.0L)*a[34]*pow(u2, 3) - 1.0L/12.0L*a[35]*pow(u1, 4) + (1.0L/12.0L)*a[35]*pow(u2, 4) - 1.0L/3.0L*a[36]*u1*v2 + (1.0L/3.0L)*a[36]*u2*v2 - 1.0L/6.0L*a[37]*pow(u1, 2)*v2 + (1.0L/6.0L)*a[37]*pow(u2, 2)*v2 - 1.0L/9.0L*a[38]*pow(u1, 3)*v2 + (1.0L/9.0L)*a[38]*pow(u2, 3)*v2 - 1.0L/12.0L*a[39]*pow(u1, 4)*v2 + (1.0L/12.0L)*a[39]*pow(u2, 4)*v2 - 1.0L/3.0L*a[40]*u1*pow(v2, 2) + (1.0L/3.0L)*a[40]*u2*pow(v2, 2) - 1.0L/6.0L*a[41]*pow(u1, 2)*pow(v2, 2) + (1.0L/6.0L)*a[41]*pow(u2, 2)*pow(v2, 2) - 1.0L/9.0L*a[42]*pow(u1, 3)*pow(v2, 2) + (1.0L/9.0L)*a[42]*pow(u2, 3)*pow(v2, 2) - 1.0L/12.0L*a[43]*pow(u1, 4)*pow(v2, 2) + (1.0L/12.0L)*a[43]*pow(u2, 4)*pow(v2, 2) - 1.0L/3.0L*a[44]*u1*pow(v2, 3) + (1.0L/3.0L)*a[44]*u2*pow(v2, 3) - 1.0L/6.0L*a[45]*pow(u1, 2)*pow(v2, 3) + (1.0L/6.0L)*a[45]*pow(u2, 2)*pow(v2, 3) - 1.0L/9.0L*a[46]*pow(u1, 3)*pow(v2, 3) + (1.0L/9.0L)*a[46]*pow(u2, 3)*pow(v2, 3) - 1.0L/12.0L*a[47]*pow(u1, 4)*pow(v2, 3) + (1.0L/12.0L)*a[47]*pow(u2, 4)*pow(v2, 3)) - pow(w1, 2)*(-1.0L/2.0L*a[16]*u1 + (1.0L/2.0L)*a[16]*u2 - 1.0L/4.0L*a[17]*pow(u1, 2) + (1.0L/4.0L)*a[17]*pow(u2, 2) - 1.0L/6.0L*a[18]*pow(u1, 3) + (1.0L/6.0L)*a[18]*pow(u2, 3) - 1.0L/8.0L*a[19]*pow(u1, 4) + (1.0L/8.0L)*a[19]*pow(u2, 4) - 1.0L/2.0L*a[20]*u1*v2 + (1.0L/2.0L)*a[20]*u2*v2 - 1.0L/4.0L*a[21]*pow(u1, 2)*v2 + (1.0L/4.0L)*a[21]*pow(u2, 2)*v2 - 1.0L/6.0L*a[22]*pow(u1, 3)*v2 + (1.0L/6.0L)*a[22]*pow(u2, 3)*v2 - 1.0L/8.0L*a[23]*pow(u1, 4)*v2 + (1.0L/8.0L)*a[23]*pow(u2, 4)*v2 - 1.0L/2.0L*a[24]*u1*pow(v2, 2) + (1.0L/2.0L)*a[24]*u2*pow(v2, 2) - 1.0L/4.0L*a[25]*pow(u1, 2)*pow(v2, 2) + (1.0L/4.0L)*a[25]*pow(u2, 2)*pow(v2, 2) - 1.0L/6.0L*a[26]*pow(u1, 3)*pow(v2, 2) + (1.0L/6.0L)*a[26]*pow(u2, 3)*pow(v2, 2) - 1.0L/8.0L*a[27]*pow(u1, 4)*pow(v2, 2) + (1.0L/8.0L)*a[27]*pow(u2, 4)*pow(v2, 2) - 1.0L/2.0L*a[28]*u1*pow(v2, 3) + (1.0L/2.0L)*a[28]*u2*pow(v2, 3) - 1.0L/4.0L*a[29]*pow(u1, 2)*pow(v2, 3) + (1.0L/4.0L)*a[29]*pow(u2, 2)*pow(v2, 3) - 1.0L/6.0L*a[30]*pow(u1, 3)*pow(v2, 3) + (1.0L/6.0L)*a[30]*pow(u2, 3)*pow(v2, 3) - 1.0L/8.0L*a[31]*pow(u1, 4)*pow(v2, 3) + (1.0L/8.0L)*a[31]*pow(u2, 4)*pow(v2, 3)) - w1*(-a[0]*u1 + a[0]*u2 - 1.0L/2.0L*a[1]*pow(u1, 2) + (1.0L/2.0L)*a[1]*pow(u2, 2) - 1.0L/3.0L*a[10]*pow(u1, 3)*pow(v2, 2) + (1.0L/3.0L)*a[10]*pow(u2, 3)*pow(v2, 2) - 1.0L/4.0L*a[11]*pow(u1, 4)*pow(v2, 2) + (1.0L/4.0L)*a[11]*pow(u2, 4)*pow(v2, 2) - a[12]*u1*pow(v2, 3) + a[12]*u2*pow(v2, 3) - 1.0L/2.0L*a[13]*pow(u1, 2)*pow(v2, 3) + (1.0L/2.0L)*a[13]*pow(u2, 2)*pow(v2, 3) - 1.0L/3.0L*a[14]*pow(u1, 3)*pow(v2, 3) + (1.0L/3.0L)*a[14]*pow(u2, 3)*pow(v2, 3) - 1.0L/4.0L*a[15]*pow(u1, 4)*pow(v2, 3) + (1.0L/4.0L)*a[15]*pow(u2, 4)*pow(v2, 3) - 1.0L/3.0L*a[2]*pow(u1, 3) + (1.0L/3.0L)*a[2]*pow(u2, 3) - 1.0L/4.0L*a[3]*pow(u1, 4) + (1.0L/4.0L)*a[3]*pow(u2, 4) - a[4]*u1*v2 + a[4]*u2*v2 - 1.0L/2.0L*a[5]*pow(u1, 2)*v2 + (1.0L/2.0L)*a[5]*pow(u2, 2)*v2 - 1.0L/3.0L*a[6]*pow(u1, 3)*v2 + (1.0L/3.0L)*a[6]*pow(u2, 3)*v2 - 1.0L/4.0L*a[7]*pow(u1, 4)*v2 + (1.0L/4.0L)*a[7]*pow(u2, 4)*v2 - a[8]*u1*pow(v2, 2) + a[8]*u2*pow(v2, 2) - 1.0L/2.0L*a[9]*pow(u1, 2)*pow(v2, 2) + (1.0L/2.0L)*a[9]*pow(u2, 2)*pow(v2, 2)) + pow(w2, 4)*(-1.0L/4.0L*a[48]*u1 + (1.0L/4.0L)*a[48]*u2 - 1.0L/8.0L*a[49]*pow(u1, 2) + (1.0L/8.0L)*a[49]*pow(u2, 2) - 1.0L/12.0L*a[50]*pow(u1, 3) + (1.0L/12.0L)*a[50]*pow(u2, 3) - 1.0L/16.0L*a[51]*pow(u1, 4) + (1.0L/16.0L)*a[51]*pow(u2, 4) - 1.0L/4.0L*a[52]*u1*v2 + (1.0L/4.0L)*a[52]*u2*v2 - 1.0L/8.0L*a[53]*pow(u1, 2)*v2 + (1.0L/8.0L)*a[53]*pow(u2, 2)*v2 - 1.0L/12.0L*a[54]*pow(u1, 3)*v2 + (1.0L/12.0L)*a[54]*pow(u2, 3)*v2 - 1.0L/16.0L*a[55]*pow(u1, 4)*v2 + (1.0L/16.0L)*a[55]*pow(u2, 4)*v2 - 1.0L/4.0L*a[56]*u1*pow(v2, 2) + (1.0L/4.0L)*a[56]*u2*pow(v2, 2) - 1.0L/8.0L*a[57]*pow(u1, 2)*pow(v2, 2) + (1.0L/8.0L)*a[57]*pow(u2, 2)*pow(v2, 2) - 1.0L/12.0L*a[58]*pow(u1, 3)*pow(v2, 2) + (1.0L/12.0L)*a[58]*pow(u2, 3)*pow(v2, 2) - 1.0L/16.0L*a[59]*pow(u1, 4)*pow(v2, 2) + (1.0L/16.0L)*a[59]*pow(u2, 4)*pow(v2, 2) - 1.0L/4.0L*a[60]*u1*pow(v2, 3) + (1.0L/4.0L)*a[60]*u2*pow(v2, 3) - 1.0L/8.0L*a[61]*pow(u1, 2)*pow(v2, 3) + (1.0L/8.0L)*a[61]*pow(u2, 2)*pow(v2, 3) - 1.0L/12.0L*a[62]*pow(u1, 3)*pow(v2, 3) + (1.0L/12.0L)*a[62]*pow(u2, 3)*pow(v2, 3) - 1.0L/16.0L*a[63]*pow(u1, 4)*pow(v2, 3) + (1.0L/16.0L)*a[63]*pow(u2, 4)*pow(v2, 3)) + pow(w2, 3)*(-1.0L/3.0L*a[32]*u1 + (1.0L/3.0L)*a[32]*u2 - 1.0L/6.0L*a[33]*pow(u1, 2) + (1.0L/6.0L)*a[33]*pow(u2, 2) - 1.0L/9.0L*a[34]*pow(u1, 3) + (1.0L/9.0L)*a[34]*pow(u2, 3) - 1.0L/12.0L*a[35]*pow(u1, 4) + (1.0L/12.0L)*a[35]*pow(u2, 4) - 1.0L/3.0L*a[36]*u1*v2 + (1.0L/3.0L)*a[36]*u2*v2 - 1.0L/6.0L*a[37]*pow(u1, 2)*v2 + (1.0L/6.0L)*a[37]*pow(u2, 2)*v2 - 1.0L/9.0L*a[38]*pow(u1, 3)*v2 + (1.0L/9.0L)*a[38]*pow(u2, 3)*v2 - 1.0L/12.0L*a[39]*pow(u1, 4)*v2 + (1.0L/12.0L)*a[39]*pow(u2, 4)*v2 - 1.0L/3.0L*a[40]*u1*pow(v2, 2) + (1.0L/3.0L)*a[40]*u2*pow(v2, 2) - 1.0L/6.0L*a[41]*pow(u1, 2)*pow(v2, 2) + (1.0L/6.0L)*a[41]*pow(u2, 2)*pow(v2, 2) - 1.0L/9.0L*a[42]*pow(u1, 3)*pow(v2, 2) + (1.0L/9.0L)*a[42]*pow(u2, 3)*pow(v2, 2) - 1.0L/12.0L*a[43]*pow(u1, 4)*pow(v2, 2) + (1.0L/12.0L)*a[43]*pow(u2, 4)*pow(v2, 2) - 1.0L/3.0L*a[44]*u1*pow(v2, 3) + (1.0L/3.0L)*a[44]*u2*pow(v2, 3) - 1.0L/6.0L*a[45]*pow(u1, 2)*pow(v2, 3) + (1.0L/6.0L)*a[45]*pow(u2, 2)*pow(v2, 3) - 1.0L/9.0L*a[46]*pow(u1, 3)*pow(v2, 3) + (1.0L/9.0L)*a[46]*pow(u2, 3)*pow(v2, 3) - 1.0L/12.0L*a[47]*pow(u1, 4)*pow(v2, 3) + (1.0L/12.0L)*a[47]*pow(u2, 4)*pow(v2, 3)) + pow(w2, 2)*(-1.0L/2.0L*a[16]*u1 + (1.0L/2.0L)*a[16]*u2 - 1.0L/4.0L*a[17]*pow(u1, 2) + (1.0L/4.0L)*a[17]*pow(u2, 2) - 1.0L/6.0L*a[18]*pow(u1, 3) + (1.0L/6.0L)*a[18]*pow(u2, 3) - 1.0L/8.0L*a[19]*pow(u1, 4) + (1.0L/8.0L)*a[19]*pow(u2, 4) - 1.0L/2.0L*a[20]*u1*v2 + (1.0L/2.0L)*a[20]*u2*v2 - 1.0L/4.0L*a[21]*pow(u1, 2)*v2 + (1.0L/4.0L)*a[21]*pow(u2, 2)*v2 - 1.0L/6.0L*a[22]*pow(u1, 3)*v2 + (1.0L/6.0L)*a[22]*pow(u2, 3)*v2 - 1.0L/8.0L*a[23]*pow(u1, 4)*v2 + (1.0L/8.0L)*a[23]*pow(u2, 4)*v2 - 1.0L/2.0L*a[24]*u1*pow(v2, 2) + (1.0L/2.0L)*a[24]*u2*pow(v2, 2) - 1.0L/4.0L*a[25]*pow(u1, 2)*pow(v2, 2) + (1.0L/4.0L)*a[25]*pow(u2, 2)*pow(v2, 2) - 1.0L/6.0L*a[26]*pow(u1, 3)*pow(v2, 2) + (1.0L/6.0L)*a[26]*pow(u2, 3)*pow(v2, 2) - 1.0L/8.0L*a[27]*pow(u1, 4)*pow(v2, 2) + (1.0L/8.0L)*a[27]*pow(u2, 4)*pow(v2, 2) - 1.0L/2.0L*a[28]*u1*pow(v2, 3) + (1.0L/2.0L)*a[28]*u2*pow(v2, 3) - 1.0L/4.0L*a[29]*pow(u1, 2)*pow(v2, 3) + (1.0L/4.0L)*a[29]*pow(u2, 2)*pow(v2, 3) - 1.0L/6.0L*a[30]*pow(u1, 3)*pow(v2, 3) + (1.0L/6.0L)*a[30]*pow(u2, 3)*pow(v2, 3) - 1.0L/8.0L*a[31]*pow(u1, 4)*pow(v2, 3) + (1.0L/8.0L)*a[31]*pow(u2, 4)*pow(v2, 3)) + w2*(-a[0]*u1 + a[0]*u2 - 1.0L/2.0L*a[1]*pow(u1, 2) + (1.0L/2.0L)*a[1]*pow(u2, 2) - 1.0L/3.0L*a[10]*pow(u1, 3)*pow(v2, 2) + (1.0L/3.0L)*a[10]*pow(u2, 3)*pow(v2, 2) - 1.0L/4.0L*a[11]*pow(u1, 4)*pow(v2, 2) + (1.0L/4.0L)*a[11]*pow(u2, 4)*pow(v2, 2) - a[12]*u1*pow(v2, 3) + a[12]*u2*pow(v2, 3) - 1.0L/2.0L*a[13]*pow(u1, 2)*pow(v2, 3) + (1.0L/2.0L)*a[13]*pow(u2, 2)*pow(v2, 3) - 1.0L/3.0L*a[14]*pow(u1, 3)*pow(v2, 3) + (1.0L/3.0L)*a[14]*pow(u2, 3)*pow(v2, 3) - 1.0L/4.0L*a[15]*pow(u1, 4)*pow(v2, 3) + (1.0L/4.0L)*a[15]*pow(u2, 4)*pow(v2, 3) - 1.0L/3.0L*a[2]*pow(u1, 3) + (1.0L/3.0L)*a[2]*pow(u2, 3) - 1.0L/4.0L*a[3]*pow(u1, 4) + (1.0L/4.0L)*a[3]*pow(u2, 4) - a[4]*u1*v2 + a[4]*u2*v2 - 1.0L/2.0L*a[5]*pow(u1, 2)*v2 + (1.0L/2.0L)*a[5]*pow(u2, 2)*v2 - 1.0L/3.0L*a[6]*pow(u1, 3)*v2 + (1.0L/3.0L)*a[6]*pow(u2, 3)*v2 - 1.0L/4.0L*a[7]*pow(u1, 4)*v2 + (1.0L/4.0L)*a[7]*pow(u2, 4)*v2 - a[8]*u1*pow(v2, 2) + a[8]*u2*pow(v2, 2) - 1.0L/2.0L*a[9]*pow(u1, 2)*pow(v2, 2) + (1.0L/2.0L)*a[9]*pow(u2, 2)*pow(v2, 2));
    return G_result;
}

double Energy::gradientZMaxComponent(const double*& a, double u1, double v1, double u2, double v2, double w2) {
    double G_result;
    G_result = a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 + (1.0L/2.0L)*a[1]*pow(u1, 2)*v1 - 1.0L/2.0L*a[1]*pow(u1, 2)*v2 - 1.0L/2.0L*a[1]*pow(u2, 2)*v1 + (1.0L/2.0L)*a[1]*pow(u2, 2)*v2 + (1.0L/9.0L)*a[10]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[10]*pow(u1, 3)*pow(v2, 3) - 1.0L/9.0L*a[10]*pow(u2, 3)*pow(v1, 3) + (1.0L/9.0L)*a[10]*pow(u2, 3)*pow(v2, 3) + (1.0L/12.0L)*a[11]*pow(u1, 4)*pow(v1, 3) - 1.0L/12.0L*a[11]*pow(u1, 4)*pow(v2, 3) - 1.0L/12.0L*a[11]*pow(u2, 4)*pow(v1, 3) + (1.0L/12.0L)*a[11]*pow(u2, 4)*pow(v2, 3) + (1.0L/4.0L)*a[12]*u1*pow(v1, 4) - 1.0L/4.0L*a[12]*u1*pow(v2, 4) - 1.0L/4.0L*a[12]*u2*pow(v1, 4) + (1.0L/4.0L)*a[12]*u2*pow(v2, 4) + (1.0L/8.0L)*a[13]*pow(u1, 2)*pow(v1, 4) - 1.0L/8.0L*a[13]*pow(u1, 2)*pow(v2, 4) - 1.0L/8.0L*a[13]*pow(u2, 2)*pow(v1, 4) + (1.0L/8.0L)*a[13]*pow(u2, 2)*pow(v2, 4) + (1.0L/12.0L)*a[14]*pow(u1, 3)*pow(v1, 4) - 1.0L/12.0L*a[14]*pow(u1, 3)*pow(v2, 4) - 1.0L/12.0L*a[14]*pow(u2, 3)*pow(v1, 4) + (1.0L/12.0L)*a[14]*pow(u2, 3)*pow(v2, 4) + (1.0L/16.0L)*a[15]*pow(u1, 4)*pow(v1, 4) - 1.0L/16.0L*a[15]*pow(u1, 4)*pow(v2, 4) - 1.0L/16.0L*a[15]*pow(u2, 4)*pow(v1, 4) + (1.0L/16.0L)*a[15]*pow(u2, 4)*pow(v2, 4) + (1.0L/3.0L)*a[2]*pow(u1, 3)*v1 - 1.0L/3.0L*a[2]*pow(u1, 3)*v2 - 1.0L/3.0L*a[2]*pow(u2, 3)*v1 + (1.0L/3.0L)*a[2]*pow(u2, 3)*v2 + (1.0L/4.0L)*a[3]*pow(u1, 4)*v1 - 1.0L/4.0L*a[3]*pow(u1, 4)*v2 - 1.0L/4.0L*a[3]*pow(u2, 4)*v1 + (1.0L/4.0L)*a[3]*pow(u2, 4)*v2 + (1.0L/2.0L)*a[4]*u1*pow(v1, 2) - 1.0L/2.0L*a[4]*u1*pow(v2, 2) - 1.0L/2.0L*a[4]*u2*pow(v1, 2) + (1.0L/2.0L)*a[4]*u2*pow(v2, 2) + (1.0L/4.0L)*a[5]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[5]*pow(u1, 2)*pow(v2, 2) - 1.0L/4.0L*a[5]*pow(u2, 2)*pow(v1, 2) + (1.0L/4.0L)*a[5]*pow(u2, 2)*pow(v2, 2) + (1.0L/6.0L)*a[6]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[6]*pow(u1, 3)*pow(v2, 2) - 1.0L/6.0L*a[6]*pow(u2, 3)*pow(v1, 2) + (1.0L/6.0L)*a[6]*pow(u2, 3)*pow(v2, 2) + (1.0L/8.0L)*a[7]*pow(u1, 4)*pow(v1, 2) - 1.0L/8.0L*a[7]*pow(u1, 4)*pow(v2, 2) - 1.0L/8.0L*a[7]*pow(u2, 4)*pow(v1, 2) + (1.0L/8.0L)*a[7]*pow(u2, 4)*pow(v2, 2) + (1.0L/3.0L)*a[8]*u1*pow(v1, 3) - 1.0L/3.0L*a[8]*u1*pow(v2, 3) - 1.0L/3.0L*a[8]*u2*pow(v1, 3) + (1.0L/3.0L)*a[8]*u2*pow(v2, 3) + (1.0L/6.0L)*a[9]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[9]*pow(u1, 2)*pow(v2, 3) - 1.0L/6.0L*a[9]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[9]*pow(u2, 2)*pow(v2, 3) + 4*pow(w2, 3)*((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 + (1.0L/8.0L)*a[49]*pow(u1, 2)*v1 - 1.0L/8.0L*a[49]*pow(u1, 2)*v2 - 1.0L/8.0L*a[49]*pow(u2, 2)*v1 + (1.0L/8.0L)*a[49]*pow(u2, 2)*v2 + (1.0L/12.0L)*a[50]*pow(u1, 3)*v1 - 1.0L/12.0L*a[50]*pow(u1, 3)*v2 - 1.0L/12.0L*a[50]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[50]*pow(u2, 3)*v2 + (1.0L/16.0L)*a[51]*pow(u1, 4)*v1 - 1.0L/16.0L*a[51]*pow(u1, 4)*v2 - 1.0L/16.0L*a[51]*pow(u2, 4)*v1 + (1.0L/16.0L)*a[51]*pow(u2, 4)*v2 + (1.0L/8.0L)*a[52]*u1*pow(v1, 2) - 1.0L/8.0L*a[52]*u1*pow(v2, 2) - 1.0L/8.0L*a[52]*u2*pow(v1, 2) + (1.0L/8.0L)*a[52]*u2*pow(v2, 2) + (1.0L/16.0L)*a[53]*pow(u1, 2)*pow(v1, 2) - 1.0L/16.0L*a[53]*pow(u1, 2)*pow(v2, 2) - 1.0L/16.0L*a[53]*pow(u2, 2)*pow(v1, 2) + (1.0L/16.0L)*a[53]*pow(u2, 2)*pow(v2, 2) + (1.0L/24.0L)*a[54]*pow(u1, 3)*pow(v1, 2) - 1.0L/24.0L*a[54]*pow(u1, 3)*pow(v2, 2) - 1.0L/24.0L*a[54]*pow(u2, 3)*pow(v1, 2) + (1.0L/24.0L)*a[54]*pow(u2, 3)*pow(v2, 2) + (1.0L/32.0L)*a[55]*pow(u1, 4)*pow(v1, 2) - 1.0L/32.0L*a[55]*pow(u1, 4)*pow(v2, 2) - 1.0L/32.0L*a[55]*pow(u2, 4)*pow(v1, 2) + (1.0L/32.0L)*a[55]*pow(u2, 4)*pow(v2, 2) + (1.0L/12.0L)*a[56]*u1*pow(v1, 3) - 1.0L/12.0L*a[56]*u1*pow(v2, 3) - 1.0L/12.0L*a[56]*u2*pow(v1, 3) + (1.0L/12.0L)*a[56]*u2*pow(v2, 3) + (1.0L/24.0L)*a[57]*pow(u1, 2)*pow(v1, 3) - 1.0L/24.0L*a[57]*pow(u1, 2)*pow(v2, 3) - 1.0L/24.0L*a[57]*pow(u2, 2)*pow(v1, 3) + (1.0L/24.0L)*a[57]*pow(u2, 2)*pow(v2, 3) + (1.0L/36.0L)*a[58]*pow(u1, 3)*pow(v1, 3) - 1.0L/36.0L*a[58]*pow(u1, 3)*pow(v2, 3) - 1.0L/36.0L*a[58]*pow(u2, 3)*pow(v1, 3) + (1.0L/36.0L)*a[58]*pow(u2, 3)*pow(v2, 3) + (1.0L/48.0L)*a[59]*pow(u1, 4)*pow(v1, 3) - 1.0L/48.0L*a[59]*pow(u1, 4)*pow(v2, 3) - 1.0L/48.0L*a[59]*pow(u2, 4)*pow(v1, 3) + (1.0L/48.0L)*a[59]*pow(u2, 4)*pow(v2, 3) + (1.0L/16.0L)*a[60]*u1*pow(v1, 4) - 1.0L/16.0L*a[60]*u1*pow(v2, 4) - 1.0L/16.0L*a[60]*u2*pow(v1, 4) + (1.0L/16.0L)*a[60]*u2*pow(v2, 4) + (1.0L/32.0L)*a[61]*pow(u1, 2)*pow(v1, 4) - 1.0L/32.0L*a[61]*pow(u1, 2)*pow(v2, 4) - 1.0L/32.0L*a[61]*pow(u2, 2)*pow(v1, 4) + (1.0L/32.0L)*a[61]*pow(u2, 2)*pow(v2, 4) + (1.0L/48.0L)*a[62]*pow(u1, 3)*pow(v1, 4) - 1.0L/48.0L*a[62]*pow(u1, 3)*pow(v2, 4) - 1.0L/48.0L*a[62]*pow(u2, 3)*pow(v1, 4) + (1.0L/48.0L)*a[62]*pow(u2, 3)*pow(v2, 4) + (1.0L/64.0L)*a[63]*pow(u1, 4)*pow(v1, 4) - 1.0L/64.0L*a[63]*pow(u1, 4)*pow(v2, 4) - 1.0L/64.0L*a[63]*pow(u2, 4)*pow(v1, 4) + (1.0L/64.0L)*a[63]*pow(u2, 4)*pow(v2, 4)) + 3*pow(w2, 2)*((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 + (1.0L/6.0L)*a[33]*pow(u1, 2)*v1 - 1.0L/6.0L*a[33]*pow(u1, 2)*v2 - 1.0L/6.0L*a[33]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[33]*pow(u2, 2)*v2 + (1.0L/9.0L)*a[34]*pow(u1, 3)*v1 - 1.0L/9.0L*a[34]*pow(u1, 3)*v2 - 1.0L/9.0L*a[34]*pow(u2, 3)*v1 + (1.0L/9.0L)*a[34]*pow(u2, 3)*v2 + (1.0L/12.0L)*a[35]*pow(u1, 4)*v1 - 1.0L/12.0L*a[35]*pow(u1, 4)*v2 - 1.0L/12.0L*a[35]*pow(u2, 4)*v1 + (1.0L/12.0L)*a[35]*pow(u2, 4)*v2 + (1.0L/6.0L)*a[36]*u1*pow(v1, 2) - 1.0L/6.0L*a[36]*u1*pow(v2, 2) - 1.0L/6.0L*a[36]*u2*pow(v1, 2) + (1.0L/6.0L)*a[36]*u2*pow(v2, 2) + (1.0L/12.0L)*a[37]*pow(u1, 2)*pow(v1, 2) - 1.0L/12.0L*a[37]*pow(u1, 2)*pow(v2, 2) - 1.0L/12.0L*a[37]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[37]*pow(u2, 2)*pow(v2, 2) + (1.0L/18.0L)*a[38]*pow(u1, 3)*pow(v1, 2) - 1.0L/18.0L*a[38]*pow(u1, 3)*pow(v2, 2) - 1.0L/18.0L*a[38]*pow(u2, 3)*pow(v1, 2) + (1.0L/18.0L)*a[38]*pow(u2, 3)*pow(v2, 2) + (1.0L/24.0L)*a[39]*pow(u1, 4)*pow(v1, 2) - 1.0L/24.0L*a[39]*pow(u1, 4)*pow(v2, 2) - 1.0L/24.0L*a[39]*pow(u2, 4)*pow(v1, 2) + (1.0L/24.0L)*a[39]*pow(u2, 4)*pow(v2, 2) + (1.0L/9.0L)*a[40]*u1*pow(v1, 3) - 1.0L/9.0L*a[40]*u1*pow(v2, 3) - 1.0L/9.0L*a[40]*u2*pow(v1, 3) + (1.0L/9.0L)*a[40]*u2*pow(v2, 3) + (1.0L/18.0L)*a[41]*pow(u1, 2)*pow(v1, 3) - 1.0L/18.0L*a[41]*pow(u1, 2)*pow(v2, 3) - 1.0L/18.0L*a[41]*pow(u2, 2)*pow(v1, 3) + (1.0L/18.0L)*a[41]*pow(u2, 2)*pow(v2, 3) + (1.0L/27.0L)*a[42]*pow(u1, 3)*pow(v1, 3) - 1.0L/27.0L*a[42]*pow(u1, 3)*pow(v2, 3) - 1.0L/27.0L*a[42]*pow(u2, 3)*pow(v1, 3) + (1.0L/27.0L)*a[42]*pow(u2, 3)*pow(v2, 3) + (1.0L/36.0L)*a[43]*pow(u1, 4)*pow(v1, 3) - 1.0L/36.0L*a[43]*pow(u1, 4)*pow(v2, 3) - 1.0L/36.0L*a[43]*pow(u2, 4)*pow(v1, 3) + (1.0L/36.0L)*a[43]*pow(u2, 4)*pow(v2, 3) + (1.0L/12.0L)*a[44]*u1*pow(v1, 4) - 1.0L/12.0L*a[44]*u1*pow(v2, 4) - 1.0L/12.0L*a[44]*u2*pow(v1, 4) + (1.0L/12.0L)*a[44]*u2*pow(v2, 4) + (1.0L/24.0L)*a[45]*pow(u1, 2)*pow(v1, 4) - 1.0L/24.0L*a[45]*pow(u1, 2)*pow(v2, 4) - 1.0L/24.0L*a[45]*pow(u2, 2)*pow(v1, 4) + (1.0L/24.0L)*a[45]*pow(u2, 2)*pow(v2, 4) + (1.0L/36.0L)*a[46]*pow(u1, 3)*pow(v1, 4) - 1.0L/36.0L*a[46]*pow(u1, 3)*pow(v2, 4) - 1.0L/36.0L*a[46]*pow(u2, 3)*pow(v1, 4) + (1.0L/36.0L)*a[46]*pow(u2, 3)*pow(v2, 4) + (1.0L/48.0L)*a[47]*pow(u1, 4)*pow(v1, 4) - 1.0L/48.0L*a[47]*pow(u1, 4)*pow(v2, 4) - 1.0L/48.0L*a[47]*pow(u2, 4)*pow(v1, 4) + (1.0L/48.0L)*a[47]*pow(u2, 4)*pow(v2, 4)) + 2*w2*((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 + (1.0L/4.0L)*a[17]*pow(u1, 2)*v1 - 1.0L/4.0L*a[17]*pow(u1, 2)*v2 - 1.0L/4.0L*a[17]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[17]*pow(u2, 2)*v2 + (1.0L/6.0L)*a[18]*pow(u1, 3)*v1 - 1.0L/6.0L*a[18]*pow(u1, 3)*v2 - 1.0L/6.0L*a[18]*pow(u2, 3)*v1 + (1.0L/6.0L)*a[18]*pow(u2, 3)*v2 + (1.0L/8.0L)*a[19]*pow(u1, 4)*v1 - 1.0L/8.0L*a[19]*pow(u1, 4)*v2 - 1.0L/8.0L*a[19]*pow(u2, 4)*v1 + (1.0L/8.0L)*a[19]*pow(u2, 4)*v2 + (1.0L/4.0L)*a[20]*u1*pow(v1, 2) - 1.0L/4.0L*a[20]*u1*pow(v2, 2) - 1.0L/4.0L*a[20]*u2*pow(v1, 2) + (1.0L/4.0L)*a[20]*u2*pow(v2, 2) + (1.0L/8.0L)*a[21]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[21]*pow(u1, 2)*pow(v2, 2) - 1.0L/8.0L*a[21]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[21]*pow(u2, 2)*pow(v2, 2) + (1.0L/12.0L)*a[22]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[22]*pow(u1, 3)*pow(v2, 2) - 1.0L/12.0L*a[22]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[22]*pow(u2, 3)*pow(v2, 2) + (1.0L/16.0L)*a[23]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[23]*pow(u1, 4)*pow(v2, 2) - 1.0L/16.0L*a[23]*pow(u2, 4)*pow(v1, 2) + (1.0L/16.0L)*a[23]*pow(u2, 4)*pow(v2, 2) + (1.0L/6.0L)*a[24]*u1*pow(v1, 3) - 1.0L/6.0L*a[24]*u1*pow(v2, 3) - 1.0L/6.0L*a[24]*u2*pow(v1, 3) + (1.0L/6.0L)*a[24]*u2*pow(v2, 3) + (1.0L/12.0L)*a[25]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[25]*pow(u1, 2)*pow(v2, 3) - 1.0L/12.0L*a[25]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[25]*pow(u2, 2)*pow(v2, 3) + (1.0L/18.0L)*a[26]*pow(u1, 3)*pow(v1, 3) - 1.0L/18.0L*a[26]*pow(u1, 3)*pow(v2, 3) - 1.0L/18.0L*a[26]*pow(u2, 3)*pow(v1, 3) + (1.0L/18.0L)*a[26]*pow(u2, 3)*pow(v2, 3) + (1.0L/24.0L)*a[27]*pow(u1, 4)*pow(v1, 3) - 1.0L/24.0L*a[27]*pow(u1, 4)*pow(v2, 3) - 1.0L/24.0L*a[27]*pow(u2, 4)*pow(v1, 3) + (1.0L/24.0L)*a[27]*pow(u2, 4)*pow(v2, 3) + (1.0L/8.0L)*a[28]*u1*pow(v1, 4) - 1.0L/8.0L*a[28]*u1*pow(v2, 4) - 1.0L/8.0L*a[28]*u2*pow(v1, 4) + (1.0L/8.0L)*a[28]*u2*pow(v2, 4) + (1.0L/16.0L)*a[29]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[29]*pow(u1, 2)*pow(v2, 4) - 1.0L/16.0L*a[29]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[29]*pow(u2, 2)*pow(v2, 4) + (1.0L/24.0L)*a[30]*pow(u1, 3)*pow(v1, 4) - 1.0L/24.0L*a[30]*pow(u1, 3)*pow(v2, 4) - 1.0L/24.0L*a[30]*pow(u2, 3)*pow(v1, 4) + (1.0L/24.0L)*a[30]*pow(u2, 3)*pow(v2, 4) + (1.0L/32.0L)*a[31]*pow(u1, 4)*pow(v1, 4) - 1.0L/32.0L*a[31]*pow(u1, 4)*pow(v2, 4) - 1.0L/32.0L*a[31]*pow(u2, 4)*pow(v1, 4) + (1.0L/32.0L)*a[31]*pow(u2, 4)*pow(v2, 4));
    return G_result;
}

double Energy::gradientEvaluateComponent(const Pointd& bmin, const Pointd& bmax, double (*f)(const double* &, double, double, double, double, double, double)) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                    const double* coeffs;
                    g->getCoefficients(coeffs, Pointd(x1,y1,z1));
                    if (minbx <= x1 && maxbx >= x2 &&
                        minby <= y1 && maxby >= y2 &&
                        minbz <= z1 && maxbz >= z2 ) { // completly contained
                        energy += f(coeffs, 0,0,0,1,1,1);
                        //g->addCube(BoundingBox(Pointd(x1,y1,z1), Pointd(x2,y2,z2)));
                    }
                    else { //partially contained
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
                        energy += f(coeffs, u1,v1,w1,u2,v2,w2);
                    }
                }

            }
        }
    }

    return energy;
}

double Energy::gradientEvaluateXMinComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

double Energy::gradientEvaluateYMinComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

double Energy::gradientEvaluateZMinComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

double Energy::gradientEvaluateXMaxComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

double Energy::gradientEvaluateYMaxComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

double Energy::gradientEvaluateZMaxComponent(const Pointd& bmin, const Pointd& bmax) const {
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                const double* coeffs;
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

void Energy::gradientTricubicInterpolationEnergy(Eigen::VectorXd& gradient, const Pointd& min, const Pointd& max) const {
    gradient(0) = gradientEvaluateXMinComponent(min, max);
    gradient(1) = gradientEvaluateYMinComponent(min, max);
    gradient(2) = gradientEvaluateZMinComponent(min, max);
    gradient(3) = gradientEvaluateXMaxComponent(min, max);
    gradient(4) = gradientEvaluateYMaxComponent(min, max);
    gradient(5) = gradientEvaluateZMaxComponent(min, max);
    for (int i = 0; i < 6; i++) gradient(i) /= 2;
}

void Energy::gradientEnergy(Eigen::VectorXd& gradient, const Eigen::VectorXd& x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    assert(x.rows() == 6);
    Pointd min(x(0), x(1), x(2)), max(x(3), x(4), x(5));
    Box3D b(min, max, c1, c2, c3);
    gradientTricubicInterpolationEnergy(gradient, min, max);
    //gradient += gradientBarrierEnergy(b);
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

double Energy::lowConstraint(const Pointd& min, const Pointd& c, double s) const {
    return fi(c.x()-min.x(),s) + fi(c.y()-min.y(),s)+ fi(c.z()-min.z(),s);
}

double Energy::highConstraint(const Pointd& max, const Pointd& c, double s) const {
    return fi(max.x()-c.x(),s) + fi(max.y()-c.y(),s) + fi(max.z()-c.z(),s);
}

double Energy::gBarrier(double x, double s) const {
    return (1/(pow(s,3)))*pow(x,3) - (3/(pow(s,2)))*pow(x,2) + (3/(s))*x;
}

double Energy::fi(double x, double s) const {
    return x <= 0 ? std::numeric_limits<double>::max() :
                    x > s ?
                        0 : (1 / gBarrier(x, s) - 1);
}

double Energy::barrierEnergy(const Box3D& b, double s) const {
    Pointd c1 = b.getConstraint1(), c2 = b.getConstraint2(), c3 = b.getConstraint3();
    Pointd min = b.getMin(), max = b.getMax();
    return lowConstraint(min, c1, s) + lowConstraint(min, c2, s) + lowConstraint(min, c3, s) + highConstraint(max, c1, s) + highConstraint(max, c2, s) + highConstraint(max, c3, s);
}

//sia i coefficienti che la sotto-box integrata devono essere nell'intervallo 0-1
double Energy::integralTricubicInterpolation(const double*& a, double u1, double v1, double w1, double u2, double v2, double w2) {
    //Simpy generated code
    //assert (a.size() == 64);
    double C_result;
    C_result = - pow(w1, 4) *
               ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
                (1.0L/8.0L)*a[49]*pow(u1, 2)*v1 - 1.0L/8.0L*a[49]*pow(u1, 2)*v2 - 1.0L/8.0L*a[49]*pow(u2, 2)*v1 + (1.0L/8.0L)*a[49]*pow(u2, 2)*v2 +
                (1.0L/12.0L)*a[50]*pow(u1, 3)*v1 - 1.0L/12.0L*a[50]*pow(u1, 3)*v2 - 1.0L/12.0L*a[50]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[50]*pow(u2, 3)*v2 +
                (1.0L/16.0L)*a[51]*pow(u1, 4)*v1 - 1.0L/16.0L*a[51]*pow(u1, 4)*v2 - 1.0L/16.0L*a[51]*pow(u2, 4)*v1 + (1.0L/16.0L)*a[51]*pow(u2, 4)*v2 +
                (1.0L/8.0L)*a[52]*u1*pow(v1, 2) - 1.0L/8.0L*a[52]*u1*pow(v2, 2) - 1.0L/8.0L*a[52]*u2*pow(v1, 2) + (1.0L/8.0L)*a[52]*u2*pow(v2, 2) +
                (1.0L/16.0L)*a[53]*pow(u1, 2)*pow(v1, 2) - 1.0L/16.0L*a[53]*pow(u1, 2)*pow(v2, 2) - 1.0L/16.0L*a[53]*pow(u2, 2)*pow(v1, 2) + (1.0L/16.0L)*a[53]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/24.0L)*a[54]*pow(u1, 3)*pow(v1, 2) - 1.0L/24.0L*a[54]*pow(u1, 3)*pow(v2, 2) - 1.0L/24.0L*a[54]*pow(u2, 3)*pow(v1, 2) + (1.0L/24.0L)*a[54]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/32.0L)*a[55]*pow(u1, 4)*pow(v1, 2) - 1.0L/32.0L*a[55]*pow(u1, 4)*pow(v2, 2) - 1.0L/32.0L*a[55]*pow(u2, 4)*pow(v1, 2) + (1.0L/32.0L)*a[55]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/12.0L)*a[56]*u1*pow(v1, 3) - 1.0L/12.0L*a[56]*u1*pow(v2, 3) - 1.0L/12.0L*a[56]*u2*pow(v1, 3) + (1.0L/12.0L)*a[56]*u2*pow(v2, 3) +
                (1.0L/24.0L)*a[57]*pow(u1, 2)*pow(v1, 3) - 1.0L/24.0L*a[57]*pow(u1, 2)*pow(v2, 3) - 1.0L/24.0L*a[57]*pow(u2, 2)*pow(v1, 3) + (1.0L/24.0L)*a[57]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/36.0L)*a[58]*pow(u1, 3)*pow(v1, 3) - 1.0L/36.0L*a[58]*pow(u1, 3)*pow(v2, 3) - 1.0L/36.0L*a[58]*pow(u2, 3)*pow(v1, 3) + (1.0L/36.0L)*a[58]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/48.0L)*a[59]*pow(u1, 4)*pow(v1, 3) - 1.0L/48.0L*a[59]*pow(u1, 4)*pow(v2, 3) - 1.0L/48.0L*a[59]*pow(u2, 4)*pow(v1, 3) + (1.0L/48.0L)*a[59]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/16.0L)*a[60]*u1*pow(v1, 4) - 1.0L/16.0L*a[60]*u1*pow(v2, 4) - 1.0L/16.0L*a[60]*u2*pow(v1, 4) + (1.0L/16.0L)*a[60]*u2*pow(v2, 4) +
                (1.0L/32.0L)*a[61]*pow(u1, 2)*pow(v1, 4) - 1.0L/32.0L*a[61]*pow(u1, 2)*pow(v2, 4) - 1.0L/32.0L*a[61]*pow(u2, 2)*pow(v1, 4) + (1.0L/32.0L)*a[61]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/48.0L)*a[62]*pow(u1, 3)*pow(v1, 4) - 1.0L/48.0L*a[62]*pow(u1, 3)*pow(v2, 4) - 1.0L/48.0L*a[62]*pow(u2, 3)*pow(v1, 4) + (1.0L/48.0L)*a[62]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/64.0L)*a[63]*pow(u1, 4)*pow(v1, 4) - 1.0L/64.0L*a[63]*pow(u1, 4)*pow(v2, 4) - 1.0L/64.0L*a[63]*pow(u2, 4)*pow(v1, 4) + (1.0L/64.0L)*a[63]*pow(u2, 4)*pow(v2, 4))
               - pow(w1, 3) *
               ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
                (1.0L/6.0L)*a[33]*pow(u1, 2)*v1 - 1.0L/6.0L*a[33]*pow(u1, 2)*v2 - 1.0L/6.0L*a[33]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[33]*pow(u2, 2)*v2 +
                (1.0L/9.0L)*a[34]*pow(u1, 3)*v1 - 1.0L/9.0L*a[34]*pow(u1, 3)*v2 - 1.0L/9.0L*a[34]*pow(u2, 3)*v1 + (1.0L/9.0L)*a[34]*pow(u2, 3)*v2 +
                (1.0L/12.0L)*a[35]*pow(u1, 4)*v1 - 1.0L/12.0L*a[35]*pow(u1, 4)*v2 - 1.0L/12.0L*a[35]*pow(u2, 4)*v1 + (1.0L/12.0L)*a[35]*pow(u2, 4)*v2 +
                (1.0L/6.0L)*a[36]*u1*pow(v1, 2) - 1.0L/6.0L*a[36]*u1*pow(v2, 2) - 1.0L/6.0L*a[36]*u2*pow(v1, 2) + (1.0L/6.0L)*a[36]*u2*pow(v2, 2) +
                (1.0L/12.0L)*a[37]*pow(u1, 2)*pow(v1, 2) - 1.0L/12.0L*a[37]*pow(u1, 2)*pow(v2, 2) - 1.0L/12.0L*a[37]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[37]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/18.0L)*a[38]*pow(u1, 3)*pow(v1, 2) - 1.0L/18.0L*a[38]*pow(u1, 3)*pow(v2, 2) - 1.0L/18.0L*a[38]*pow(u2, 3)*pow(v1, 2) + (1.0L/18.0L)*a[38]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/24.0L)*a[39]*pow(u1, 4)*pow(v1, 2) - 1.0L/24.0L*a[39]*pow(u1, 4)*pow(v2, 2) - 1.0L/24.0L*a[39]*pow(u2, 4)*pow(v1, 2) + (1.0L/24.0L)*a[39]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/9.0L)*a[40]*u1*pow(v1, 3) - 1.0L/9.0L*a[40]*u1*pow(v2, 3) - 1.0L/9.0L*a[40]*u2*pow(v1, 3) + (1.0L/9.0L)*a[40]*u2*pow(v2, 3) +
                (1.0L/18.0L)*a[41]*pow(u1, 2)*pow(v1, 3) - 1.0L/18.0L*a[41]*pow(u1, 2)*pow(v2, 3) - 1.0L/18.0L*a[41]*pow(u2, 2)*pow(v1, 3) + (1.0L/18.0L)*a[41]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/27.0L)*a[42]*pow(u1, 3)*pow(v1, 3) - 1.0L/27.0L*a[42]*pow(u1, 3)*pow(v2, 3) - 1.0L/27.0L*a[42]*pow(u2, 3)*pow(v1, 3) + (1.0L/27.0L)*a[42]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/36.0L)*a[43]*pow(u1, 4)*pow(v1, 3) - 1.0L/36.0L*a[43]*pow(u1, 4)*pow(v2, 3) - 1.0L/36.0L*a[43]*pow(u2, 4)*pow(v1, 3) + (1.0L/36.0L)*a[43]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/12.0L)*a[44]*u1*pow(v1, 4) - 1.0L/12.0L*a[44]*u1*pow(v2, 4) - 1.0L/12.0L*a[44]*u2*pow(v1, 4) + (1.0L/12.0L)*a[44]*u2*pow(v2, 4) +
                (1.0L/24.0L)*a[45]*pow(u1, 2)*pow(v1, 4) - 1.0L/24.0L*a[45]*pow(u1, 2)*pow(v2, 4) - 1.0L/24.0L*a[45]*pow(u2, 2)*pow(v1, 4) + (1.0L/24.0L)*a[45]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/36.0L)*a[46]*pow(u1, 3)*pow(v1, 4) - 1.0L/36.0L*a[46]*pow(u1, 3)*pow(v2, 4) - 1.0L/36.0L*a[46]*pow(u2, 3)*pow(v1, 4) + (1.0L/36.0L)*a[46]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/48.0L)*a[47]*pow(u1, 4)*pow(v1, 4) - 1.0L/48.0L*a[47]*pow(u1, 4)*pow(v2, 4) - 1.0L/48.0L*a[47]*pow(u2, 4)*pow(v1, 4) + (1.0L/48.0L)*a[47]*pow(u2, 4)*pow(v2, 4))
               - pow(w1, 2) *
               ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
                (1.0L/4.0L)*a[17]*pow(u1, 2)*v1 - 1.0L/4.0L*a[17]*pow(u1, 2)*v2 - 1.0L/4.0L*a[17]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[17]*pow(u2, 2)*v2 +
                (1.0L/6.0L)*a[18]*pow(u1, 3)*v1 - 1.0L/6.0L*a[18]*pow(u1, 3)*v2 - 1.0L/6.0L*a[18]*pow(u2, 3)*v1 + (1.0L/6.0L)*a[18]*pow(u2, 3)*v2 +
                (1.0L/8.0L)*a[19]*pow(u1, 4)*v1 - 1.0L/8.0L*a[19]*pow(u1, 4)*v2 - 1.0L/8.0L*a[19]*pow(u2, 4)*v1 + (1.0L/8.0L)*a[19]*pow(u2, 4)*v2 +
                (1.0L/4.0L)*a[20]*u1*pow(v1, 2) - 1.0L/4.0L*a[20]*u1*pow(v2, 2) - 1.0L/4.0L*a[20]*u2*pow(v1, 2) + (1.0L/4.0L)*a[20]*u2*pow(v2, 2) +
                (1.0L/8.0L)*a[21]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[21]*pow(u1, 2)*pow(v2, 2) - 1.0L/8.0L*a[21]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[21]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/12.0L)*a[22]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[22]*pow(u1, 3)*pow(v2, 2) - 1.0L/12.0L*a[22]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[22]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/16.0L)*a[23]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[23]*pow(u1, 4)*pow(v2, 2) - 1.0L/16.0L*a[23]*pow(u2, 4)*pow(v1, 2) + (1.0L/16.0L)*a[23]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/6.0L)*a[24]*u1*pow(v1, 3) - 1.0L/6.0L*a[24]*u1*pow(v2, 3) - 1.0L/6.0L*a[24]*u2*pow(v1, 3) + (1.0L/6.0L)*a[24]*u2*pow(v2, 3) +
                (1.0L/12.0L)*a[25]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[25]*pow(u1, 2)*pow(v2, 3) - 1.0L/12.0L*a[25]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[25]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/18.0L)*a[26]*pow(u1, 3)*pow(v1, 3) - 1.0L/18.0L*a[26]*pow(u1, 3)*pow(v2, 3) - 1.0L/18.0L*a[26]*pow(u2, 3)*pow(v1, 3) + (1.0L/18.0L)*a[26]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/24.0L)*a[27]*pow(u1, 4)*pow(v1, 3) - 1.0L/24.0L*a[27]*pow(u1, 4)*pow(v2, 3) - 1.0L/24.0L*a[27]*pow(u2, 4)*pow(v1, 3) + (1.0L/24.0L)*a[27]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/8.0L)*a[28]*u1*pow(v1, 4) - 1.0L/8.0L*a[28]*u1*pow(v2, 4) - 1.0L/8.0L*a[28]*u2*pow(v1, 4) + (1.0L/8.0L)*a[28]*u2*pow(v2, 4) +
                (1.0L/16.0L)*a[29]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[29]*pow(u1, 2)*pow(v2, 4) - 1.0L/16.0L*a[29]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[29]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/24.0L)*a[30]*pow(u1, 3)*pow(v1, 4) - 1.0L/24.0L*a[30]*pow(u1, 3)*pow(v2, 4) - 1.0L/24.0L*a[30]*pow(u2, 3)*pow(v1, 4) + (1.0L/24.0L)*a[30]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/32.0L)*a[31]*pow(u1, 4)*pow(v1, 4) - 1.0L/32.0L*a[31]*pow(u1, 4)*pow(v2, 4) - 1.0L/32.0L*a[31]*pow(u2, 4)*pow(v1, 4) + (1.0L/32.0L)*a[31]*pow(u2, 4)*pow(v2, 4))
               - w1 *
               (a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 +
                (1.0L/2.0L)*a[1]*pow(u1, 2)*v1 - 1.0L/2.0L*a[1]*pow(u1, 2)*v2 - 1.0L/2.0L*a[1]*pow(u2, 2)*v1 + (1.0L/2.0L)*a[1]*pow(u2, 2)*v2 +
                (1.0L/9.0L)*a[10]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[10]*pow(u1, 3)*pow(v2, 3) - 1.0L/9.0L*a[10]*pow(u2, 3)*pow(v1, 3) + (1.0L/9.0L)*a[10]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/12.0L)*a[11]*pow(u1, 4)*pow(v1, 3) - 1.0L/12.0L*a[11]*pow(u1, 4)*pow(v2, 3) - 1.0L/12.0L*a[11]*pow(u2, 4)*pow(v1, 3) + (1.0L/12.0L)*a[11]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/4.0L)*a[12]*u1*pow(v1, 4) - 1.0L/4.0L*a[12]*u1*pow(v2, 4) - 1.0L/4.0L*a[12]*u2*pow(v1, 4) + (1.0L/4.0L)*a[12]*u2*pow(v2, 4) +
                (1.0L/8.0L)*a[13]*pow(u1, 2)*pow(v1, 4) - 1.0L/8.0L*a[13]*pow(u1, 2)*pow(v2, 4) - 1.0L/8.0L*a[13]*pow(u2, 2)*pow(v1, 4) + (1.0L/8.0L)*a[13]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/12.0L)*a[14]*pow(u1, 3)*pow(v1, 4) - 1.0L/12.0L*a[14]*pow(u1, 3)*pow(v2, 4) - 1.0L/12.0L*a[14]*pow(u2, 3)*pow(v1, 4) + (1.0L/12.0L)*a[14]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/16.0L)*a[15]*pow(u1, 4)*pow(v1, 4) - 1.0L/16.0L*a[15]*pow(u1, 4)*pow(v2, 4) - 1.0L/16.0L*a[15]*pow(u2, 4)*pow(v1, 4) + (1.0L/16.0L)*a[15]*pow(u2, 4)*pow(v2, 4) +
                (1.0L/3.0L)*a[2]*pow(u1, 3)*v1 - 1.0L/3.0L*a[2]*pow(u1, 3)*v2 - 1.0L/3.0L*a[2]*pow(u2, 3)*v1 + (1.0L/3.0L)*a[2]*pow(u2, 3)*v2 +
                (1.0L/4.0L)*a[3]*pow(u1, 4)*v1 - 1.0L/4.0L*a[3]*pow(u1, 4)*v2 - 1.0L/4.0L*a[3]*pow(u2, 4)*v1 + (1.0L/4.0L)*a[3]*pow(u2, 4)*v2 +
                (1.0L/2.0L)*a[4]*u1*pow(v1, 2) - 1.0L/2.0L*a[4]*u1*pow(v2, 2) - 1.0L/2.0L*a[4]*u2*pow(v1, 2) + (1.0L/2.0L)*a[4]*u2*pow(v2, 2) +
                (1.0L/4.0L)*a[5]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[5]*pow(u1, 2)*pow(v2, 2) - 1.0L/4.0L*a[5]*pow(u2, 2)*pow(v1, 2) + (1.0L/4.0L)*a[5]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/6.0L)*a[6]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[6]*pow(u1, 3)*pow(v2, 2) - 1.0L/6.0L*a[6]*pow(u2, 3)*pow(v1, 2) + (1.0L/6.0L)*a[6]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/8.0L)*a[7]*pow(u1, 4)*pow(v1, 2) - 1.0L/8.0L*a[7]*pow(u1, 4)*pow(v2, 2) - 1.0L/8.0L*a[7]*pow(u2, 4)*pow(v1, 2) + (1.0L/8.0L)*a[7]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/3.0L)*a[8]*u1*pow(v1, 3) - 1.0L/3.0L*a[8]*u1*pow(v2, 3) - 1.0L/3.0L*a[8]*u2*pow(v1, 3) + (1.0L/3.0L)*a[8]*u2*pow(v2, 3) +
                (1.0L/6.0L)*a[9]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[9]*pow(u1, 2)*pow(v2, 3) - 1.0L/6.0L*a[9]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[9]*pow(u2, 2)*pow(v2, 3))
               + pow(w2, 4) *
               ((1.0L/4.0L)*a[48]*u1*v1 - 1.0L/4.0L*a[48]*u1*v2 - 1.0L/4.0L*a[48]*u2*v1 + (1.0L/4.0L)*a[48]*u2*v2 +
                (1.0L/8.0L)*a[49]*pow(u1, 2)*v1 - 1.0L/8.0L*a[49]*pow(u1, 2)*v2 - 1.0L/8.0L*a[49]*pow(u2, 2)*v1 + (1.0L/8.0L)*a[49]*pow(u2, 2)*v2 +
                (1.0L/12.0L)*a[50]*pow(u1, 3)*v1 - 1.0L/12.0L*a[50]*pow(u1, 3)*v2 - 1.0L/12.0L*a[50]*pow(u2, 3)*v1 + (1.0L/12.0L)*a[50]*pow(u2, 3)*v2 +
                (1.0L/16.0L)*a[51]*pow(u1, 4)*v1 - 1.0L/16.0L*a[51]*pow(u1, 4)*v2 - 1.0L/16.0L*a[51]*pow(u2, 4)*v1 + (1.0L/16.0L)*a[51]*pow(u2, 4)*v2 +
                (1.0L/8.0L)*a[52]*u1*pow(v1, 2) - 1.0L/8.0L*a[52]*u1*pow(v2, 2) - 1.0L/8.0L*a[52]*u2*pow(v1, 2) + (1.0L/8.0L)*a[52]*u2*pow(v2, 2) +
                (1.0L/16.0L)*a[53]*pow(u1, 2)*pow(v1, 2) - 1.0L/16.0L*a[53]*pow(u1, 2)*pow(v2, 2) - 1.0L/16.0L*a[53]*pow(u2, 2)*pow(v1, 2) + (1.0L/16.0L)*a[53]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/24.0L)*a[54]*pow(u1, 3)*pow(v1, 2) - 1.0L/24.0L*a[54]*pow(u1, 3)*pow(v2, 2) - 1.0L/24.0L*a[54]*pow(u2, 3)*pow(v1, 2) + (1.0L/24.0L)*a[54]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/32.0L)*a[55]*pow(u1, 4)*pow(v1, 2) - 1.0L/32.0L*a[55]*pow(u1, 4)*pow(v2, 2) - 1.0L/32.0L*a[55]*pow(u2, 4)*pow(v1, 2) + (1.0L/32.0L)*a[55]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/12.0L)*a[56]*u1*pow(v1, 3) - 1.0L/12.0L*a[56]*u1*pow(v2, 3) - 1.0L/12.0L*a[56]*u2*pow(v1, 3) + (1.0L/12.0L)*a[56]*u2*pow(v2, 3) +
                (1.0L/24.0L)*a[57]*pow(u1, 2)*pow(v1, 3) - 1.0L/24.0L*a[57]*pow(u1, 2)*pow(v2, 3) - 1.0L/24.0L*a[57]*pow(u2, 2)*pow(v1, 3) + (1.0L/24.0L)*a[57]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/36.0L)*a[58]*pow(u1, 3)*pow(v1, 3) - 1.0L/36.0L*a[58]*pow(u1, 3)*pow(v2, 3) - 1.0L/36.0L*a[58]*pow(u2, 3)*pow(v1, 3) + (1.0L/36.0L)*a[58]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/48.0L)*a[59]*pow(u1, 4)*pow(v1, 3) - 1.0L/48.0L*a[59]*pow(u1, 4)*pow(v2, 3) - 1.0L/48.0L*a[59]*pow(u2, 4)*pow(v1, 3) + (1.0L/48.0L)*a[59]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/16.0L)*a[60]*u1*pow(v1, 4) - 1.0L/16.0L*a[60]*u1*pow(v2, 4) - 1.0L/16.0L*a[60]*u2*pow(v1, 4) + (1.0L/16.0L)*a[60]*u2*pow(v2, 4) +
                (1.0L/32.0L)*a[61]*pow(u1, 2)*pow(v1, 4) - 1.0L/32.0L*a[61]*pow(u1, 2)*pow(v2, 4) - 1.0L/32.0L*a[61]*pow(u2, 2)*pow(v1, 4) + (1.0L/32.0L)*a[61]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/48.0L)*a[62]*pow(u1, 3)*pow(v1, 4) - 1.0L/48.0L*a[62]*pow(u1, 3)*pow(v2, 4) - 1.0L/48.0L*a[62]*pow(u2, 3)*pow(v1, 4) + (1.0L/48.0L)*a[62]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/64.0L)*a[63]*pow(u1, 4)*pow(v1, 4) - 1.0L/64.0L*a[63]*pow(u1, 4)*pow(v2, 4) - 1.0L/64.0L*a[63]*pow(u2, 4)*pow(v1, 4) + (1.0L/64.0L)*a[63]*pow(u2, 4)*pow(v2, 4))
               + pow(w2, 3) *
               ((1.0L/3.0L)*a[32]*u1*v1 - 1.0L/3.0L*a[32]*u1*v2 - 1.0L/3.0L*a[32]*u2*v1 + (1.0L/3.0L)*a[32]*u2*v2 +
                (1.0L/6.0L)*a[33]*pow(u1, 2)*v1 - 1.0L/6.0L*a[33]*pow(u1, 2)*v2 - 1.0L/6.0L*a[33]*pow(u2, 2)*v1 + (1.0L/6.0L)*a[33]*pow(u2, 2)*v2 +
                (1.0L/9.0L)*a[34]*pow(u1, 3)*v1 - 1.0L/9.0L*a[34]*pow(u1, 3)*v2 - 1.0L/9.0L*a[34]*pow(u2, 3)*v1 + (1.0L/9.0L)*a[34]*pow(u2, 3)*v2 +
                (1.0L/12.0L)*a[35]*pow(u1, 4)*v1 - 1.0L/12.0L*a[35]*pow(u1, 4)*v2 - 1.0L/12.0L*a[35]*pow(u2, 4)*v1 + (1.0L/12.0L)*a[35]*pow(u2, 4)*v2 +
                (1.0L/6.0L)*a[36]*u1*pow(v1, 2) - 1.0L/6.0L*a[36]*u1*pow(v2, 2) - 1.0L/6.0L*a[36]*u2*pow(v1, 2) + (1.0L/6.0L)*a[36]*u2*pow(v2, 2) +
                (1.0L/12.0L)*a[37]*pow(u1, 2)*pow(v1, 2) - 1.0L/12.0L*a[37]*pow(u1, 2)*pow(v2, 2) - 1.0L/12.0L*a[37]*pow(u2, 2)*pow(v1, 2) + (1.0L/12.0L)*a[37]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/18.0L)*a[38]*pow(u1, 3)*pow(v1, 2) - 1.0L/18.0L*a[38]*pow(u1, 3)*pow(v2, 2) - 1.0L/18.0L*a[38]*pow(u2, 3)*pow(v1, 2) + (1.0L/18.0L)*a[38]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/24.0L)*a[39]*pow(u1, 4)*pow(v1, 2) - 1.0L/24.0L*a[39]*pow(u1, 4)*pow(v2, 2) - 1.0L/24.0L*a[39]*pow(u2, 4)*pow(v1, 2) + (1.0L/24.0L)*a[39]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/9.0L)*a[40]*u1*pow(v1, 3) - 1.0L/9.0L*a[40]*u1*pow(v2, 3) - 1.0L/9.0L*a[40]*u2*pow(v1, 3) + (1.0L/9.0L)*a[40]*u2*pow(v2, 3) +
                (1.0L/18.0L)*a[41]*pow(u1, 2)*pow(v1, 3) - 1.0L/18.0L*a[41]*pow(u1, 2)*pow(v2, 3) - 1.0L/18.0L*a[41]*pow(u2, 2)*pow(v1, 3) + (1.0L/18.0L)*a[41]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/27.0L)*a[42]*pow(u1, 3)*pow(v1, 3) - 1.0L/27.0L*a[42]*pow(u1, 3)*pow(v2, 3) - 1.0L/27.0L*a[42]*pow(u2, 3)*pow(v1, 3) + (1.0L/27.0L)*a[42]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/36.0L)*a[43]*pow(u1, 4)*pow(v1, 3) - 1.0L/36.0L*a[43]*pow(u1, 4)*pow(v2, 3) - 1.0L/36.0L*a[43]*pow(u2, 4)*pow(v1, 3) + (1.0L/36.0L)*a[43]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/12.0L)*a[44]*u1*pow(v1, 4) - 1.0L/12.0L*a[44]*u1*pow(v2, 4) - 1.0L/12.0L*a[44]*u2*pow(v1, 4) + (1.0L/12.0L)*a[44]*u2*pow(v2, 4) +
                (1.0L/24.0L)*a[45]*pow(u1, 2)*pow(v1, 4) - 1.0L/24.0L*a[45]*pow(u1, 2)*pow(v2, 4) - 1.0L/24.0L*a[45]*pow(u2, 2)*pow(v1, 4) + (1.0L/24.0L)*a[45]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/36.0L)*a[46]*pow(u1, 3)*pow(v1, 4) - 1.0L/36.0L*a[46]*pow(u1, 3)*pow(v2, 4) - 1.0L/36.0L*a[46]*pow(u2, 3)*pow(v1, 4) + (1.0L/36.0L)*a[46]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/48.0L)*a[47]*pow(u1, 4)*pow(v1, 4) - 1.0L/48.0L*a[47]*pow(u1, 4)*pow(v2, 4) - 1.0L/48.0L*a[47]*pow(u2, 4)*pow(v1, 4) + (1.0L/48.0L)*a[47]*pow(u2, 4)*pow(v2, 4))
               + pow(w2, 2) *
               ((1.0L/2.0L)*a[16]*u1*v1 - 1.0L/2.0L*a[16]*u1*v2 - 1.0L/2.0L*a[16]*u2*v1 + (1.0L/2.0L)*a[16]*u2*v2 +
                (1.0L/4.0L)*a[17]*pow(u1, 2)*v1 - 1.0L/4.0L*a[17]*pow(u1, 2)*v2 - 1.0L/4.0L*a[17]*pow(u2, 2)*v1 + (1.0L/4.0L)*a[17]*pow(u2, 2)*v2 +
                (1.0L/6.0L)*a[18]*pow(u1, 3)*v1 - 1.0L/6.0L*a[18]*pow(u1, 3)*v2 - 1.0L/6.0L*a[18]*pow(u2, 3)*v1 + (1.0L/6.0L)*a[18]*pow(u2, 3)*v2 +
                (1.0L/8.0L)*a[19]*pow(u1, 4)*v1 - 1.0L/8.0L*a[19]*pow(u1, 4)*v2 - 1.0L/8.0L*a[19]*pow(u2, 4)*v1 + (1.0L/8.0L)*a[19]*pow(u2, 4)*v2 +
                (1.0L/4.0L)*a[20]*u1*pow(v1, 2) - 1.0L/4.0L*a[20]*u1*pow(v2, 2) - 1.0L/4.0L*a[20]*u2*pow(v1, 2) + (1.0L/4.0L)*a[20]*u2*pow(v2, 2) +
                (1.0L/8.0L)*a[21]*pow(u1, 2)*pow(v1, 2) - 1.0L/8.0L*a[21]*pow(u1, 2)*pow(v2, 2) - 1.0L/8.0L*a[21]*pow(u2, 2)*pow(v1, 2) + (1.0L/8.0L)*a[21]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/12.0L)*a[22]*pow(u1, 3)*pow(v1, 2) - 1.0L/12.0L*a[22]*pow(u1, 3)*pow(v2, 2) - 1.0L/12.0L*a[22]*pow(u2, 3)*pow(v1, 2) + (1.0L/12.0L)*a[22]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/16.0L)*a[23]*pow(u1, 4)*pow(v1, 2) - 1.0L/16.0L*a[23]*pow(u1, 4)*pow(v2, 2) - 1.0L/16.0L*a[23]*pow(u2, 4)*pow(v1, 2) + (1.0L/16.0L)*a[23]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/6.0L)*a[24]*u1*pow(v1, 3) - 1.0L/6.0L*a[24]*u1*pow(v2, 3) - 1.0L/6.0L*a[24]*u2*pow(v1, 3) + (1.0L/6.0L)*a[24]*u2*pow(v2, 3) +
                (1.0L/12.0L)*a[25]*pow(u1, 2)*pow(v1, 3) - 1.0L/12.0L*a[25]*pow(u1, 2)*pow(v2, 3) - 1.0L/12.0L*a[25]*pow(u2, 2)*pow(v1, 3) + (1.0L/12.0L)*a[25]*pow(u2, 2)*pow(v2, 3) +
                (1.0L/18.0L)*a[26]*pow(u1, 3)*pow(v1, 3) - 1.0L/18.0L*a[26]*pow(u1, 3)*pow(v2, 3) - 1.0L/18.0L*a[26]*pow(u2, 3)*pow(v1, 3) + (1.0L/18.0L)*a[26]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/24.0L)*a[27]*pow(u1, 4)*pow(v1, 3) - 1.0L/24.0L*a[27]*pow(u1, 4)*pow(v2, 3) - 1.0L/24.0L*a[27]*pow(u2, 4)*pow(v1, 3) + (1.0L/24.0L)*a[27]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/8.0L)*a[28]*u1*pow(v1, 4) - 1.0L/8.0L*a[28]*u1*pow(v2, 4) - 1.0L/8.0L*a[28]*u2*pow(v1, 4) + (1.0L/8.0L)*a[28]*u2*pow(v2, 4) +
                (1.0L/16.0L)*a[29]*pow(u1, 2)*pow(v1, 4) - 1.0L/16.0L*a[29]*pow(u1, 2)*pow(v2, 4) - 1.0L/16.0L*a[29]*pow(u2, 2)*pow(v1, 4) + (1.0L/16.0L)*a[29]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/24.0L)*a[30]*pow(u1, 3)*pow(v1, 4) - 1.0L/24.0L*a[30]*pow(u1, 3)*pow(v2, 4) - 1.0L/24.0L*a[30]*pow(u2, 3)*pow(v1, 4) + (1.0L/24.0L)*a[30]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/32.0L)*a[31]*pow(u1, 4)*pow(v1, 4) - 1.0L/32.0L*a[31]*pow(u1, 4)*pow(v2, 4) - 1.0L/32.0L*a[31]*pow(u2, 4)*pow(v1, 4) + (1.0L/32.0L)*a[31]*pow(u2, 4)*pow(v2, 4))
               + w2 *
               (a[0]*u1*v1 - a[0]*u1*v2 - a[0]*u2*v1 + a[0]*u2*v2 +
                (1.0L/2.0L)*a[1]*pow(u1, 2)*v1 - 1.0L/2.0L*a[1]*pow(u1, 2)*v2 - 1.0L/2.0L*a[1]*pow(u2, 2)*v1 + (1.0L/2.0L)*a[1]*pow(u2, 2)*v2 +
                (1.0L/9.0L)*a[10]*pow(u1, 3)*pow(v1, 3) - 1.0L/9.0L*a[10]*pow(u1, 3)*pow(v2, 3) - 1.0L/9.0L*a[10]*pow(u2, 3)*pow(v1, 3) + (1.0L/9.0L)*a[10]*pow(u2, 3)*pow(v2, 3) +
                (1.0L/12.0L)*a[11]*pow(u1, 4)*pow(v1, 3) - 1.0L/12.0L*a[11]*pow(u1, 4)*pow(v2, 3) - 1.0L/12.0L*a[11]*pow(u2, 4)*pow(v1, 3) + (1.0L/12.0L)*a[11]*pow(u2, 4)*pow(v2, 3) +
                (1.0L/4.0L)*a[12]*u1*pow(v1, 4) - 1.0L/4.0L*a[12]*u1*pow(v2, 4) - 1.0L/4.0L*a[12]*u2*pow(v1, 4) + (1.0L/4.0L)*a[12]*u2*pow(v2, 4) +
                (1.0L/8.0L)*a[13]*pow(u1, 2)*pow(v1, 4) - 1.0L/8.0L*a[13]*pow(u1, 2)*pow(v2, 4) - 1.0L/8.0L*a[13]*pow(u2, 2)*pow(v1, 4) + (1.0L/8.0L)*a[13]*pow(u2, 2)*pow(v2, 4) +
                (1.0L/12.0L)*a[14]*pow(u1, 3)*pow(v1, 4) - 1.0L/12.0L*a[14]*pow(u1, 3)*pow(v2, 4) - 1.0L/12.0L*a[14]*pow(u2, 3)*pow(v1, 4) + (1.0L/12.0L)*a[14]*pow(u2, 3)*pow(v2, 4) +
                (1.0L/16.0L)*a[15]*pow(u1, 4)*pow(v1, 4) - 1.0L/16.0L*a[15]*pow(u1, 4)*pow(v2, 4) - 1.0L/16.0L*a[15]*pow(u2, 4)*pow(v1, 4) + (1.0L/16.0L)*a[15]*pow(u2, 4)*pow(v2, 4) +
                (1.0L/3.0L)*a[2]*pow(u1, 3)*v1 - 1.0L/3.0L*a[2]*pow(u1, 3)*v2 - 1.0L/3.0L*a[2]*pow(u2, 3)*v1 + (1.0L/3.0L)*a[2]*pow(u2, 3)*v2 +
                (1.0L/4.0L)*a[3]*pow(u1, 4)*v1 - 1.0L/4.0L*a[3]*pow(u1, 4)*v2 - 1.0L/4.0L*a[3]*pow(u2, 4)*v1 + (1.0L/4.0L)*a[3]*pow(u2, 4)*v2 +
                (1.0L/2.0L)*a[4]*u1*pow(v1, 2) - 1.0L/2.0L*a[4]*u1*pow(v2, 2) - 1.0L/2.0L*a[4]*u2*pow(v1, 2) + (1.0L/2.0L)*a[4]*u2*pow(v2, 2) +
                (1.0L/4.0L)*a[5]*pow(u1, 2)*pow(v1, 2) - 1.0L/4.0L*a[5]*pow(u1, 2)*pow(v2, 2) - 1.0L/4.0L*a[5]*pow(u2, 2)*pow(v1, 2) + (1.0L/4.0L)*a[5]*pow(u2, 2)*pow(v2, 2) +
                (1.0L/6.0L)*a[6]*pow(u1, 3)*pow(v1, 2) - 1.0L/6.0L*a[6]*pow(u1, 3)*pow(v2, 2) - 1.0L/6.0L*a[6]*pow(u2, 3)*pow(v1, 2) + (1.0L/6.0L)*a[6]*pow(u2, 3)*pow(v2, 2) +
                (1.0L/8.0L)*a[7]*pow(u1, 4)*pow(v1, 2) - 1.0L/8.0L*a[7]*pow(u1, 4)*pow(v2, 2) - 1.0L/8.0L*a[7]*pow(u2, 4)*pow(v1, 2) + (1.0L/8.0L)*a[7]*pow(u2, 4)*pow(v2, 2) +
                (1.0L/3.0L)*a[8]*u1*pow(v1, 3) - 1.0L/3.0L*a[8]*u1*pow(v2, 3) - 1.0L/3.0L*a[8]*u2*pow(v1, 3) + (1.0L/3.0L)*a[8]*u2*pow(v2, 3) +
                (1.0L/6.0L)*a[9]*pow(u1, 2)*pow(v1, 3) - 1.0L/6.0L*a[9]*pow(u1, 2)*pow(v2, 3) - 1.0L/6.0L*a[9]*pow(u2, 2)*pow(v1, 3) + (1.0L/6.0L)*a[9]*pow(u2, 2)*pow(v2, 3));
    return C_result;

}

double Energy::integralTricubicInterpolationEnergy(const Pointd& bmin, const Pointd& bmax) const {
    g->deleteCubes();
    double unit = g->getUnit();

    BoundingBox bb = g->getBoundingBox();
    Pointd c = bb.getMin();
    Pointd d = c-bmin;
    Pointd nu = d / g->getUnit();
    Pointi ni = Pointi(nu.x(), nu.y(), nu.z());
    Pointi di = ni * g->getUnit();
    Pointd min = c - Pointd(di.x(), di.y(), di.z());
    d = c-bmax;
    nu = d / g->getUnit();
    ni = Pointi(nu.x(), nu.y(), nu.z());
    di = ni * g->getUnit();
    Pointd max = c - Pointd(di.x(), di.y(), di.z());

    double firstX = min.x()-unit, firstY = min.y()-unit, firstZ = min.z()-unit, lastX = max.x()+unit, lastY = max.y()+unit, lastZ = max.z()+unit;

    double minbx = bmin.x(), minby = bmin.y(), minbz = bmin.z();
    double maxbx = bmax.x(), maxby = bmax.y(), maxbz = bmax.z();
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
                    const double* coeffs;
                    g->getCoefficients(coeffs, Pointd(x1,y1,z1));
                    if (minbx <= x1 && maxbx >= x2 &&
                        minby <= y1 && maxby >= y2 &&
                        minbz <= z1 && maxbz >= z2 ) { // completly contained
                        energy += integralTricubicInterpolation(coeffs, 0,0,0,1,1,1);
                        //g->addCube(BoundingBox(Pointd(x1,y1,z1), Pointd(x2,y2,z2)));
                    }
                    else { //partially contained
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

double Energy::energy(const Box3D& b) const {
    return integralTricubicInterpolationEnergy(b.getMin(), b.getMax()) /*+ barrierEnergy(b)*/;
}

double Energy::energy(const Eigen::VectorXd &x, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    assert(x.rows() == 6);
    Pointd min(x(0), x(1), x(2)), max(x(3), x(4), x(5));
    Box3D b(min, max, c1, c2, c3);
    return integralTricubicInterpolationEnergy(min, max) /*+ barrierEnergy(b)*/;
}

double Energy::energy(double minx, double miny, double minz, double maxx, double maxy, double maxz, const Pointd& c1, const Pointd& c2, const Pointd& c3) const {
    Box3D b(Pointd(minx, miny, minz), Pointd(maxx, maxy, maxz), c1, c2, c3);
    return integralTricubicInterpolationEnergy(b.getMin(), b.getMax()) /*+ barrierEnergy(b)*/;
}
