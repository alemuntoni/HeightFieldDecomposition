#include "energy.h"


Energy::Energy() {
}

Energy::Energy(DrawableGrid& g) : g(&g){
}

//sia i coefficienti che la sotto-box integrata devono essere nell'intervallo 0-1
double Energy::integralTricubicInterpolation(const Eigen::VectorXd& a, double u1, double v1, double w1, double u2, double v2, double w2) {
    //Simpy generated code
    assert (a.rows() == 64);
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

double Energy::integralTricubicInterpolationEnergy(const Box3D& b) {
    return evaluateTricubicInterpolationFunction(b, integralTricubicInterpolation);
}

double Energy::evaluateTricubicInterpolationFunction(const Box3D& b, double (*f)(const Eigen::VectorXd&, double, double, double, double, double, double)) {

}

double Energy::energy(const Box3D& b) {
    return integralTricubicInterpolationEnergy(b)/* + barrierEnergy(b)*/;
}
