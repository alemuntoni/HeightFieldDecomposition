//this file exists just for an ambiguity on cgal

#include "tinyfeaturedetection.h"

#include <cg3/cgal/2d/cgal_booleans2d.h>

bool int2d(const std::vector<cg3::Point2Dd> &polygon1, const std::vector<cg3::Point2Dd> &polygon2){
    return cg3::cgal::booleans2d::doIntersect(polygon1, polygon2);
}
