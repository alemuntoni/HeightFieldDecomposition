#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>

#include "dcel/dcel.h"
#include "lib/grid/grid.h"
#include "common/timer.h"
#include "energy.h"
#include "boxlist.h"
#include "common.h"
#ifndef SERVER_MODE
#include "cgal/aabbtree.h"
#include "igl/iglinterface.h"
#endif

#define ORIENTATIONS 4
#define TARGETS 6

namespace Engine {

    Vec3 getClosestTarget(const Vec3 &n);

    void serializeAsEngineManager(std::ofstream &binaryfile, const Grid& g, const Dcel& d, const BoxList& bl);

    Eigen::Matrix3d rotateDcelAlreadyScaled(Dcel& d, unsigned int rot);

    Eigen::Matrix3d scaleAndRotateDcel(Dcel& d, int resolution = 50, unsigned int rot = 0);
    #ifndef SERVER_MODE
    void generateGrid(Grid &g, const Dcel &d, double kernelDistance = 6, bool heightfields = false, const Vec3& target = Vec3());

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());
    #endif
    void expandBoxes(BoxList &boxList, const Grid &g);
    #ifndef SERVER_MODE
    void createVectorTriples(std::vector<std::tuple<int, Box3D, std::vector<bool> > >& vectorTriples, const BoxList& boxList, const Dcel &d);

    int deleteBoxes(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces);

    int deleteBoxes(BoxList& boxList, const Dcel &d);

    int deleteBoxesMemorySafe(BoxList& boxList, const Dcel &d);

    void makePreprocessingAndSave(const Dcel &input, const std::string& filename, int resolution = 50, double kernelDistance = 6, bool heightfields = false);

    void largeScaleFabrication(const Dcel &input, int resolution = 50, double kernelDistance = 6, bool heightfields = false);
     #endif
    namespace Server {
        void expandBoxesFromPreprocessing(const std::string &inputFile, const std::string &outputFile);

        void booleanOperationsFromSolutions(const std::string &inputFile, const std::string &outputFile);
    }

}

#endif // ENGINE_H
