#ifndef ENGINE_H
#define ENGINE_H

#include <omp.h>
#include <stack>

#include "cg3/utilities/timer.h"
#include "boxlist.h"
#include "common.h"

#include <cg3/meshes/dcel/dcel.h>
#include "lib/grid/grid.h"
#include "energy.h"
#include <cg3/cgal/cgal.h>
#include "heightfieldslist.h"
#include "lib/grid/irregulargrid.h"

#define ORIENTATIONS 1
#define TARGETS 6
#define STARTING_NUMBER_FACES 600

#define BOOL_DEBUG

namespace Engine {
    Eigen::Matrix3d findOptimalOrientation(cg3::Dcel& d, cg3::EigenMesh& originalMesh);

    cg3::Vec3 getClosestTarget(const cg3::Vec3 &n);

    Eigen::Matrix3d rotateDcelAlreadyScaled(cg3::Dcel& d, unsigned int rot);

    Eigen::Matrix3d scaleAndRotateDcel(cg3::Dcel& d, unsigned int rot = 0, double factor = 1);

    void getFlippedFaces(std::set<const cg3::Dcel::Face*>& flippedFaces, std::set<const cg3::Dcel::Face*>& savedFaces, const cg3::Dcel& d, const cg3::Vec3& target, double angleThreshold, double areaThreshold);

    void setTrianglesTargets(cg3::Dcel scaled[]);

    void generateGridAndDistanceField(cg3::Array3D<cg3::Pointd> &grid, cg3::Array3D<gridreal> &distanceField, const cg3::SimpleEigenMesh& m, bool generateDistanceField = true, double gridUnit = 2, bool integer = true);

    void calculateGridWeights(Grid& g, const cg3::Array3D<cg3::Pointd> &grid, const cg3::Array3D<gridreal> &distanceField, const cg3::Dcel& d, double kernelDistance, bool tolerance, const cg3::Vec3 &target, std::set<const cg3::Dcel::Face*>& savedFaces);

    static std::set<const cg3::Dcel::Face*> dummy;
    static cg3::Array3D<gridreal> ddf;
    cg3::Array3D<gridreal> generateGrid(Grid &g, const cg3::Dcel &d, double kernelDistance = 6, bool tolerance = false, const cg3::Vec3& target = cg3::Vec3(), std::set<const cg3::Dcel::Face*> &savedFaces = dummy);

    void addBox(BoxList& boxList, const cg3::Vec3 target, const cg3::Dcel::Face* f, const Eigen::Matrix3d& rot);

    void calculateDecimatedBoxes(BoxList &boxList, const cg3::Dcel &d, const Eigen::VectorXi& mapping, const std::set<int> &coveredFaces, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), int orientation = -1,  bool onlyTarget = false, const cg3::Vec3& target = cg3::Vec3());

    void calculateInitialBoxes(BoxList &boxList, const cg3::Dcel &d, const Eigen::Matrix3d& rot = Eigen::Matrix3d::Identity(), bool onlyTarget = false, const cg3::Vec3& target = cg3::Vec3());

    void expandBoxes(BoxList &boxList, const Grid &g, bool limit, const cg3::Pointd& limits, bool printTimes = false);

    void createVectorTriples(std::vector<std::tuple<int, Box3D, std::vector<bool> > >& vectorTriples, const BoxList& boxList, const cg3::Dcel &d);

    int deleteBoxesNonOptimal(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces);

    int deleteBoxesNonOptimal(BoxList& boxList, const cg3::Dcel &d);

    double deleteBoxes(BoxList& boxList, const cg3::Dcel &d);

    int deleteBoxesGSC(BoxList& boxList, const cg3::Dcel &d);

    static BoxList dummy2;
    double optimize(BoxList &solutions, cg3::Dcel& d, double kernelDistance, bool limit, cg3::Pointd limits = cg3::Pointd(), bool tolerance = true, bool onlyNearestTarget = true, double areaTolerance = 0, double angleTolerance = 0, bool file = false, bool decimate = true);

    void optimizeAndDeleteBoxes(BoxList &solutions, cg3::Dcel& d, double kernelDistance, bool limit, cg3::Pointd limits = cg3::Pointd(), bool heightfields = true, bool onlyNearestTarget = true, double areaTolerance = 0, double angleTolerance = 0, bool file = false, bool decimate = true, BoxList& allSolutions = dummy2);

    void boxPostProcessing(BoxList &solutions, const cg3::Dcel& d);

    std::vector<Box3D> splitBoxWithMoreThanOneConnectedComponent(const Box3D& originalBox, const std::vector<std::set<const cg3::Dcel::Face*> >& connectedComponents);

    void stupidSnapping(const cg3::Dcel& d, BoxList& solutions, double epsilon);

    bool smartSnapping(const Box3D& b1, Box3D& b2, std::vector<unsigned int>& trianglesCovered, const cg3::cgal::AABBTree& tree);

    void smartSnapping(const cg3::Dcel& d, BoxList& solutions);

    void merging(const cg3::Dcel& d, BoxList& solutions);

    void deleteDuplicatedBoxes(BoxList &solutions);

    void booleanOperations(HeightfieldsList &he, cg3::SimpleEigenMesh& bc, BoxList &solutions, bool alternativeColors = false);

    void splitConnectedComponents(HeightfieldsList &he, BoxList &solutions, std::map<unsigned int, unsigned int>& mapping);

    void glueInternHeightfieldsToBaseComplex(HeightfieldsList &he, BoxList &solutions, cg3::SimpleEigenMesh& bc, const cg3::Dcel& inputMesh);

    void reduceHeightfields(HeightfieldsList& he, cg3::SimpleEigenMesh& bc, const cg3::Dcel& inputMesh);

    void gluePortionsToBaseComplex(HeightfieldsList &he, cg3::SimpleEigenMesh& bc, BoxList &solutions, const cg3::Dcel& inputMesh);

    //void largeScaleFabrication(const Dcel &input, double kernelDistance = 6, bool heightfields = false);

    cg3::SimpleEigenMesh getMarkerMesh(const HeightfieldsList& he, const cg3::Dcel& d);

    void updatePieceNormals(const cg3::cgal::AABBTree& tree, cg3::Dcel &piece);
    void updatePieceNormals(const cg3::cgal::AABBTree& tree, cg3::EigenMesh &piece);

    void updatePiecesNormals(const cg3::cgal::AABBTree& tree, HeightfieldsList &he);

    void saveObjs(const std::string &foldername, const cg3::EigenMesh& originalMesh, const cg3::Dcel& inputMesh, const cg3::EigenMesh& baseComplex, const HeightfieldsList& he);

    bool isAnHeightfield(const cg3::EigenMesh &m, const cg3::Vec3& v, bool strictly = false);

    void colorPieces(const cg3::Dcel& d, HeightfieldsList& he);


    void mergePostProcessing(HeightfieldsList& he, BoxList& solutions, cg3::EigenMesh &baseComplex, const cg3::Dcel &d, bool mergeDownwards = false);
}

#endif // ENGINE_H
