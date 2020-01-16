/*
 * This file is part of cg3lib: https://github.com/cg3hci/cg3lib
 * This Source Code Form is subject to the terms of the GNU GPL 3.0
 *
 * @author Alessandro Muntoni (muntoni.alessandro@gmail.com)
 */
#include "unsigned_distances.h"

std::vector<double> getUnsignedDistances(
        const std::vector<cg3::Point3d>& points,
        const cg3::cgal::AABBTree3& tree)
{
	std::vector<double> distances(points.size(), 0);
	size_t size = distances.size();

    #pragma omp parallel for
	for (unsigned int i = 0; i < size; i++){
		distances[i] = tree.squaredDistance(points[i]);
	}

	return distances;
}
