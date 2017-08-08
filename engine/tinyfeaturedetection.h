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

std::set<unsigned int> chartExpansion(const EigenMesh &hf, unsigned int f, std::vector<bool> &seen);

std::vector<Pointd> getPolygonFromChart(const EigenMesh&hf, const std::set<unsigned int>& chart);

bool tinyFeaturePlane(const EigenMesh& hf, const Vec3 &target, double threshold, double& mindist);

}

#endif // TINYFEATUREDETECTION_H
