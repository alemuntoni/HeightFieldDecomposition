#ifndef TINYFEATUREDETECTION_H
#define TINYFEATUREDETECTION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/data_structures/arrays/arrays.h>
#include <cg3/geometry/2d/point2d.h>

namespace TinyFeatureDetection
{

void colorSDF(cg3::EigenMesh& m, std::vector<unsigned int>& problematicFaces);

std::set<unsigned int> chartExpansion(const cg3::EigenMesh &hf, unsigned int f, std::vector<bool> &seen);

std::vector<cg3::Pointd> getPolygonFromChart(const cg3::EigenMesh&hf, const std::set<unsigned int>& chart);

bool tinyFeaturePlane(const cg3::EigenMesh& hf, const cg3::Vec3 &target, double threshold, double& mindist);

}

bool int2d(const std::vector<cg3::Point2Dd> &polygon1, const std::vector<cg3::Point2Dd> &polygon2);

#endif // TINYFEATUREDETECTION_H
