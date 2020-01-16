/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#ifndef UNSIGNED_DISTANCES_H
#define UNSIGNED_DISTANCES_H

#include <cg3/cgal/aabb_tree3.h>

std::vector<double> getUnsignedDistances(
        const std::vector<cg3::Point3d>& points,
        const cg3::cgal::AABBTree3& tree);

#endif // UNSIGNED_DISTANCES_H
