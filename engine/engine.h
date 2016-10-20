#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>

#include "common/timer.h"
#include "boxlist.h"
#include "common.h"

#ifndef SERVER_MODE
#include "dcel/dcel.h"
#include "lib/grid/grid.h"
#include "energy.h"
#include "cgal/aabbtree.h"
#include "igl/iglinterface.h"
#include "heightfieldslist.h"
#include "lib/grid/irregulargrid.h"
#elif SERVER_MODE==1
#include "dcel/dcel.h"
#include "lib/grid/grid.h"
#include "energy.h"
#include "igl/iglinterface.h"
#elif SERVER_MODE==2
#include "igl/iglinterface.h"
#endif

#define ORIENTATIONS 1
#define TARGETS 6

namespace Engine {
    Vec3 getClosestTarget(const Vec3 &n);

    void serializeAsEngineManager(std::ofstream &binaryfile, const Grid& g, const Dcel& d, const BoxList& bl);

    Eigen::Matrix3d rotateDcelAlreadyScaled(Dcel& d, unsigned int rot);

    Eigen::Matrix3d scaleAndRotateDcel(Dcel& d, unsigned int rot = 0, double factor = 1);

    void getFlippedFaces(std::set<const Dcel::Face*>& flippedFaces, std::set<const Dcel::Face*>& savedFaces, const Dcel& d, const Vec3& target, double angleThreshold, double areaThreshold);

    void setTrianglesTargets(Dcel scaled[]);

    static std::set<const Dcel::Face*> dummy;
    void generateGrid(Grid &g, const Dcel &d, double kernelDistance = 6, bool heightfields = false, const Vec3& target = Vec3(), std::set<const Dcel::Face*> &savedFaces = dummy);

    void addBox(BoxList& boxList, const Vec3 target, const Dcel::Face* f, const Eigen::Matrix3d& rot);

    void calculateDecimatedBoxes(BoxList &boxList, const Dcel &d, const Eigen::VectorXi& mapping, const std::set<int> &coveredFaces, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), int orientation = -1,  bool onlyTarget = false, const Vec3& target = Vec3());

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g, bool printTimes = false);

    void createAndMinimizeAllBoxes(BoxList &solutions, const Dcel &d, double kernelDistance, bool heightfields = true, bool onlyNearestTarget = true, double areaTolerance = 0, double angleTolerance = 0);

    void compactSet(std::set<double> &set, double epsilon = 1e-6);

    void createIrregularGrid(IrregularGrid &grid, const BoxList& solutions, double epsilon = 0.01);

    static HeightfieldsList dummyhe;
    void booleanOperations(HeightfieldsList &he, IGLInterface::SimpleIGLMesh &bc, BoxList &solutions, const Dcel& inputMesh, HeightfieldsList &entirePieces = dummyhe);

    void gluePortionsToBaseComplex(HeightfieldsList &he, IGLInterface::SimpleIGLMesh &bc, BoxList &solutions, const Dcel& inputMesh);

    void createVectorTriples(std::vector<std::tuple<int, Box3D, std::vector<bool> > >& vectorTriples, const BoxList& boxList, const Dcel &d);

    int deleteBoxes(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces);

    int deleteBoxes(BoxList& boxList, const Dcel &d);

    int deleteBoxesMemorySafe(BoxList& boxList, const Dcel &d);

    void largeScaleFabrication(const Dcel &input, double kernelDistance = 6, bool heightfields = false);
}

#endif // ENGINE_H
