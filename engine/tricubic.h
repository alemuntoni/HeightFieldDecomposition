#ifndef TRI_CUBIC_INTERPOLATOR_H
#define TRI_CUBIC_INTERPOLATOR_H

#include <Eigen/Core>
#include "cg3/data_structures/arrays/arrays.h"
#include "cg3/geometry/point.h"

typedef float gridreal;

namespace TricubicInterpolator {

    void getCoefficients(std::vector<std::array<gridreal, 64> >& coeffs, cg3::Array3D<int>& mapCoeffs,  const cg3::Array3D<gridreal> &weights);

    void getCoefficients(cg3::Array4D<gridreal>& coeffs, const cg3::Array3D<gridreal> &weights);

    double getValue(const cg3::Pointd &p, const gridreal* coeffs);
}

#endif
