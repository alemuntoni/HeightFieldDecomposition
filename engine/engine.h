#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>

#include "lib/dcel/dcel.h"
#include "lib/grid/grid.h"
#include "lib/common/timer.h"
#include "energy.h"
#include "boxlist.h"
#include "common.h"
#include "lib/cgal_interface/aabbtree.h"

#define ORIENTATIONS 4
#define TARGETS 6

namespace Engine {

    Vec3 getClosestTarget(const Vec3 &n);

    void serializeAsEngineManager(std::ofstream &binaryfile, const Grid& g, const Dcel& d, const BoxList& bl);



    Eigen::Matrix3d scaleAndRotateDcel(Dcel& d, int resolution = 50, unsigned int rot = 0);

    void generateGrid(Grid &g, const Dcel &d, double kernelDistance = 6, const Vec3& target = Vec3(), bool heightfields = false);

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g);

    void createVectorTriples(std::vector<std::tuple<int, Box3D, std::vector<unsigned int> > >& vectorTriples, const BoxList& boxList, const Dcel &d);

    int deleteBoxes(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<unsigned int> > > &vectorTriples, unsigned int numberFaces);

    int deleteBoxes(BoxList& boxList, const Dcel &d);

    void makePreprocessingAndSave(const Dcel &input, const std::string& filename, int resolution = 50, double kernelDistance = 6, bool heightfields = false);

    void expandBoxesFromPreprocessing(const std::string &inputFile, const std::string &outputFile);

    void afterExpandedBoxes(const std::string &inputFile);

    void largeScaleFabrication(const Dcel &input, int resolution = 50, double kernelDistance = 6, bool heightfields = false);

}

#endif // ENGINE_H
