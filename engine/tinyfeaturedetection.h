#ifndef TINYFEATUREDETECTION_H
#define TINYFEATUREDETECTION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/data_structures/arrays/arrays.h>

namespace TinyFeatureDetection
{

std::vector<unsigned int> sdf(const cg3::EigenMesh& m, double threshold);
void colorSDF(cg3::EigenMesh& m, std::vector<unsigned int>& problematicFaces);

cg3::Array3D<cg3::Pointd> generateGrid(const cg3::EigenMesh &m, double threshold);

bool tinyFeatureVoxelization(const cg3::EigenMesh &hf, const cg3::Vec3 &target, double threshold);

std::set<unsigned int> chartExpansion(const cg3::EigenMesh &hf, unsigned int f, std::vector<bool> &seen);

std::vector<cg3::Pointd> getPolygonFromChart(const cg3::EigenMesh&hf, const std::set<unsigned int>& chart);

bool tinyFeaturePlane(const cg3::EigenMesh& hf, const cg3::Vec3 &target, double threshold, double& mindist);

}

#endif // TINYFEATUREDETECTION_H
