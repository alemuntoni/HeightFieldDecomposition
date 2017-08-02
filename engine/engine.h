#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>
#include <stack>

#include "common/timer.h"
#include "boxlist.h"
#include "common.h"

#include <dcel/dcel.h>
#include "lib/grid/grid.h"
#include "energy.h"
#include <cgal/cgalinterface.h>
#include "heightfieldslist.h"
#include "lib/grid/irregulargrid.h"

#define ORIENTATIONS 1
#define TARGETS 6
#define STARTING_NUMBER_FACES 600

#define BOOL_DEBUG

namespace Engine {
    void findOptimalOrientation(Dcel& d, EigenMesh& originalMesh);

    Vec3 getClosestTarget(const Vec3 &n);

    void serializeAsEngineManager(std::ofstream &binaryfile, const Grid& g, const Dcel& d, const BoxList& bl);

    Eigen::Matrix3d rotateDcelAlreadyScaled(Dcel& d, unsigned int rot);

    Eigen::Matrix3d scaleAndRotateDcel(Dcel& d, unsigned int rot = 0, double factor = 1);

    void getFlippedFaces(std::set<const Dcel::Face*>& flippedFaces, std::set<const Dcel::Face*>& savedFaces, const Dcel& d, const Vec3& target, double angleThreshold, double areaThreshold);

    void setTrianglesTargets(Dcel scaled[]);

    void generateGridAndDistanceField(Array3D<Pointd> &grid, Array3D<gridreal> &distanceField, const SimpleEigenMesh& m, bool generateDistanceField = true, double gridUnit = 2, bool integer = true);

    void calculateGridWeights(Grid& g, const Array3D<Pointd> &grid, const Array3D<gridreal> &distanceField, const Dcel& d, double kernelDistance, bool tolerance, const Vec3 &target, std::set<const Dcel::Face*>& savedFaces);

    static std::set<const Dcel::Face*> dummy;
    static Array3D<gridreal> ddf;
    Array3D<gridreal> generateGrid(Grid &g, const Dcel &d, double kernelDistance = 6, bool tolerance = false, const Vec3& target = Vec3(), std::set<const Dcel::Face*> &savedFaces = dummy);

    void addBox(BoxList& boxList, const Vec3 target, const Dcel::Face* f, const Eigen::Matrix3d& rot);

    void calculateDecimatedBoxes(BoxList &boxList, const Dcel &d, const Eigen::VectorXi& mapping, const std::set<int> &coveredFaces, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), int orientation = -1,  bool onlyTarget = false, const Vec3& target = Vec3());

    void calculateInitialBoxes(BoxList &boxList, const Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const Vec3& target = Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g, bool limit, const Pointd& limits, bool printTimes = false);

    void createVectorTriples(std::vector<std::tuple<int, Box3D, std::vector<bool> > >& vectorTriples, const BoxList& boxList, const Dcel &d);

    int deleteBoxesNonOptimal(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces);

    int deleteBoxesNonOptimal(BoxList& boxList, const Dcel &d);

    double deleteBoxes(BoxList& boxList, const Dcel &d);

    int deleteBoxesGSC(BoxList& boxList, const Dcel &d);

    static BoxList dummy2;
    double optimize(BoxList &solutions, Dcel& d, double kernelDistance, bool limit, Pointd limits = Pointd(), bool tolerance = true, bool onlyNearestTarget = true, double areaTolerance = 0, double angleTolerance = 0, bool file = false, bool decimate = true);

    void optimizeAndDeleteBoxes(BoxList &solutions, Dcel& d, double kernelDistance, bool limit, Pointd limits = Pointd(), bool heightfields = true, bool onlyNearestTarget = true, double areaTolerance = 0, double angleTolerance = 0, bool file = false, bool decimate = true, BoxList& allSolutions = dummy2);

    void boxPostProcessing(BoxList &solutions, const Dcel& d);

    std::vector<Box3D> splitBoxWithMoreThanOneConnectedComponent(const Box3D& originalBox, const std::vector<std::set<const Dcel::Face*> >& connectedComponents);

    void stupidSnapping(const Dcel& d, BoxList& solutions, double epsilon);

    bool smartSnapping(const Box3D& b1, Box3D& b2, std::vector<unsigned int>& trianglesCovered, const CGALInterface::AABBTree& tree);

    void smartSnapping(const Dcel& d, BoxList& solutions);

    void merging(const Dcel& d, BoxList& solutions);

    void deleteDuplicatedBoxes(BoxList &solutions);

    void booleanOperations(HeightfieldsList &he, SimpleEigenMesh& bc, BoxList &solutions, bool alternativeColors = false);

    void splitConnectedComponents(HeightfieldsList &he, BoxList &solutions, std::map<unsigned int, unsigned int>& mapping);

    void glueInternHeightfieldsToBaseComplex(HeightfieldsList &he, BoxList &solutions, SimpleEigenMesh& bc, const Dcel& inputMesh);

    void reduceHeightfields(HeightfieldsList& he, SimpleEigenMesh& bc, const Dcel& inputMesh);

    void gluePortionsToBaseComplex(HeightfieldsList &he, SimpleEigenMesh& bc, BoxList &solutions, const Dcel& inputMesh);

    //void largeScaleFabrication(const Dcel &input, double kernelDistance = 6, bool heightfields = false);

    SimpleEigenMesh getMarkerMesh(const HeightfieldsList& he, const Dcel& d);

    void updatePieceNormals(const CGALInterface::AABBTree& tree, Dcel &piece);
    void updatePieceNormals(const CGALInterface::AABBTree& tree, EigenMesh &piece);

    void updatePiecesNormals(const CGALInterface::AABBTree& tree, HeightfieldsList &he);

    void saveObjs(const QString& foldername, const EigenMesh& originalMesh, const Dcel& inputMesh, const EigenMesh& baseComplex, const HeightfieldsList& he);

    bool isAnHeightfield(const EigenMesh &m, const Vec3& v, bool strictly = false);

    void colorPieces(const Dcel& d, HeightfieldsList& he);

    bool isTinyFeature(const EigenMesh& m, double threshold);

    void tinyFeatureDetection(HeightfieldsList& he, double threshold);
}

#endif // ENGINE_H
