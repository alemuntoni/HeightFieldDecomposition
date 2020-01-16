#ifndef TINYFEATUREDETECTION_H
#define TINYFEATUREDETECTION_H

#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/data_structures/arrays/arrays.h>
#include <cg3/geometry/point2.h>

namespace TinyFeatureDetection
{

void colorSDF(cg3::EigenMesh& m, std::vector<unsigned int>& problematicFaces);

std::set<unsigned int> chartExpansion(const cg3::EigenMesh &hf, unsigned int f, std::vector<bool> &seen);

std::vector<cg3::Point3d> getPolygonFromChart(const cg3::EigenMesh&hf, const std::set<unsigned int>& chart);

bool tinyFeaturePlane(const cg3::EigenMesh& hf, const cg3::Vec3d &target, double threshold, double& mindist);

}

bool int2d(const std::vector<cg3::Point2d> &polygon1, const std::vector<cg3::Point2d> &polygon2);

#endif // TINYFEATUREDETECTION_H
