#ifndef FOURAXISCHECKER_H
#define FOURAXISCHECKER_H
#include <cg3/data_structures/arrays/arrays.h>
#include "cg3/meshes/eigenmesh/eigenmesh.h"

namespace FourAxisChecker {

int maxYFace(std::vector<int> &list, const cg3::EigenMesh& mesh);
int minYFace(std::vector<int>& list, const cg3::EigenMesh& mesh);
void checkPlane(cg3::Array2D<int>& visibility, const cg3::EigenMesh& mesh, int indexPlane, int numberPlanes);
void checkVisibilityAllPlanes(const cg3::EigenMesh& mesh, cg3::Array2D<int> &visibility, int numberPlanes);
void minimizeNumberPlanes(std::vector<int> &survivedPlanes, cg3::Array2D<int> &visibility);
#ifdef MULTI_LABEL_OPTIMIZATION_INCLUDED
std::vector<int> getAssociation(const std::vector<int> &survivedPlanes, cg3::Array2D<int> &visibility, const cg3::EigenMesh &mesh);
#endif

}

#endif // FOURAXISCHECKER_H
