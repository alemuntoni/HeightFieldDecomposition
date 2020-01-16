//this file exists just for an ambiguity on cgal

#include "tinyfeaturedetection.h"

#include <cg3/cgal/booleans2.h>

bool int2d(const std::vector<cg3::Point2d> &polygon1, const std::vector<cg3::Point2d> &polygon2){
    return cg3::cgal::doIntersect(polygon1, polygon2);
}
