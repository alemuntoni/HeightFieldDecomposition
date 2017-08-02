#ifndef TINYFEATUREDETECTION_H
#define TINYFEATUREDETECTION_H

#include <eigenmesh/eigenmesh.h>
#include <common/arrays.h>

namespace TinyFeatureDetection
{

std::vector<unsigned int> sdf(const EigenMesh& m, double threshold);
void colorSDF(EigenMesh& m, std::vector<unsigned int>& problematicFaces);

Array3D<Pointd> generateGrid(const EigenMesh &m, double threshold);

bool tinyFeatureVoxelization(const EigenMesh &hf, const Vec3 &target, double threshold);

}

#endif // TINYFEATUREDETECTION_H
