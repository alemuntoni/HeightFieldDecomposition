#include "engine.h"
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include <cg3/meshes/dcel/algorithms/dcel_algorithms.h>
#include <cg3/geometry/transformations3.h>
#include <cg3/utilities/set.h>
#include <cg3/libigl/decimate.h>
#include <cg3/libigl/booleans.h>
#include <cg3/libigl/mesh_adjacencies.h>
#include <cg3/libigl/connected_components.h>
#include <cg3/libigl/remove_duplicate_vertices.h>

#ifdef GUROBI_DEFINED
#include <gurobi_c++.h>
#endif

#include <cg3/cgal/polyhedron.h>
#include "unsigned_distances.h"
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>

#include "splitting.h"
#include "reconstruction.h"
#include <cg3/algorithms/global_optimal_rotation_matrix.h>

using namespace cg3;

Eigen::Matrix3d Engine::findOptimalOrientation(Dcel &d, EigenMesh& originalMesh) {
	Eigen::Matrix3d matr = cg3::globalOptimalRotationMatrix(d, 1000);
    d.rotate(matr);
	Point3d c = d.boundingBox().center();
    d.translate(-c);
	if (originalMesh.numberVertices() > 0){
        originalMesh.rotate(matr);
        originalMesh.translate(-c);
    }
    return matr;
}

Vec3d Engine::getClosestTarget(const Vec3d& n)
{
    double angle = n.dot(XYZ[0]);
    int k = 0;
    for (unsigned int i = 1; i < 6; i++) {
        if (n.dot(XYZ[i]) > angle) {
            angle = n.dot(XYZ[i]);
            k = i;
        }
    }
    return XYZ[k];
}

Eigen::Matrix3d Engine::rotateDcelAlreadyScaled(Dcel& d, unsigned int rot) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (rot > 0){
        switch (rot){
            case 1:
				cg3::rotationMatrix(Vec3d(0,0,1), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
				cg3::rotationMatrix(Vec3d(0,0,-1), M_PI/4, m);
                break;
            case 2:
				cg3::rotationMatrix(Vec3d(0,1,0), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
				cg3::rotationMatrix(Vec3d(0,-1,0), M_PI/4, m);
                break;
            case 3:
				cg3::rotationMatrix(Vec3d(1,0,0), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
				cg3::rotationMatrix(Vec3d(-1,0,0), M_PI/4, m);
                break;
            default:
                assert(0);
        }
    }
    return m;
}

Eigen::Matrix3d Engine::scaleAndRotateDcel(Dcel& d, unsigned int rot, double factor) {
	BoundingBox3 bb = d.boundingBox();
    double avg = 0;
    for (Dcel::HalfEdgeIterator heit = d.halfEdgeBegin(); heit != d.halfEdgeEnd(); ++heit){
		avg += (*heit)->length();
    }
	avg /= d.numberHalfEdges();
	double maxl = std::max(bb.maxX() - bb.minX(), bb.maxY() - bb.minY());
	maxl = std::max(maxl, bb.maxZ() - bb.minZ());
    int resolution = (maxl / avg + 1)*factor;
    double av = maxl / resolution;
	BoundingBox3 nBB(-(bb.max()-bb.min())/av, (bb.max()-bb.min())/av);
    d.scale(nBB);
    return rotateDcelAlreadyScaled(d, rot);
}

void Engine::getFlippedFaces(std::set<const Dcel::Face*> &flippedFaces, std::set<const Dcel::Face*> &savedFaces, const Dcel& d, const Vec3d& target, double angleThreshold, double areaThreshold) {
    double dot = - angleThreshold;
    for (const Dcel::Face* f : d.faceIterator()){
		if (f->normal().dot(target) < 0){
			if (f->normal().dot(target) < dot)
                flippedFaces.insert(f);
            else
                savedFaces.insert(f);
        }
    }
    if (areaThreshold > 0) { //if areathreshold = 0, flipped charts are ignored
		double totalArea = d.surfaceArea();
        areaThreshold*=totalArea;

        //Chart construction
        std::set<const Dcel::Face*> visitedFaces, connectedComponent;
        for (const Dcel::Face* ff : flippedFaces) {
            if (visitedFaces.find(ff) == visitedFaces.end()){
                std::stack<const Dcel::Face* > stack;
                stack.push(ff);
                connectedComponent.clear();
                double area = 0;
                while (stack.size() > 0) {
                    const Dcel::Face* f = stack.top();
					area += f->area();
                    stack.pop();
                    connectedComponent.insert(f);
                    visitedFaces.insert(f);
                    for (const Dcel::Face* ad : f->adjacentFaceIterator()){
                        if (connectedComponent.find(ad) == connectedComponent.end() && flippedFaces.find(ad) != flippedFaces.end()){
                            stack.push(ad);
                        }
                    }
                }
                if (area < areaThreshold){
                    savedFaces.insert(connectedComponent.begin(), connectedComponent.end());
                }
            }
        }
        for (const Dcel::Face* f : savedFaces)
            flippedFaces.erase(f);
    }
}

void Engine::generateGridAndDistanceField(Array3D<Point3d> &grid, Array3D<gridreal> &distanceField, const SimpleEigenMesh &m, bool generateDistanceField, double gridUnit, bool integer){
    assert(gridUnit > 0);
    // Bounding Box
    Eigen::RowVector3d Vmin, Vmax;
	m.boundingBox(Vmin, Vmax);

    // create grid GV
    Eigen::RowVector3d border((int)gridUnit*5, (int)gridUnit*5, (int)gridUnit*5);
    Eigen::RowVector3d nGmin;
    Eigen::RowVector3d nGmax;
    if (integer) {
        Eigen::RowVector3i Gmini = (Vmin).cast<int>() - border.cast<int>();
        Eigen::RowVector3i Gmaxi = (Vmax).cast<int>() + border.cast<int>();
        nGmin = Gmini.cast<double>();
        nGmax = Gmaxi.cast<double>();
        gridUnit = (int)gridUnit;
        assert(gridUnit > 0);
    }
    else {
        nGmin = Vmin - border;
        nGmax = Vmax + border; //bounding box of the Grid
    }
    Eigen::RowVector3i res = (nGmax.cast<int>() - nGmin.cast<int>())/2;
    unsigned int sizeX = res(0)+1, sizeY = res(1)+1, sizeZ = res(2)+1;
    std::vector<double> distances;
    Array3D<int> mapping(sizeX, sizeY, sizeZ, -1);
	std::vector<Point3d> insidePoints;
    int inside = 0;

    grid.resize(sizeX, sizeY, sizeZ);
    if (generateDistanceField){
        distanceField.resize(sizeX, sizeY, sizeZ);
        distanceField.fill(1);
    }
	cgal::AABBTree3 tree(m, true);
    Array3D<unsigned char> isInside(sizeX, sizeY, sizeZ);
    isInside.fill(false);

    int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
    for (unsigned int i = 0; i < sizeX; ++i){
        yi = nGmin(1);
        for (unsigned int j = 0; j < sizeY; ++j){
            zi = nGmin(2);
            for (unsigned int k = 0; k < sizeZ; ++k){
				grid(i,j,k) = Point3d(xi,yi,zi);
                zi+=gridUnit;
            }
            yi+=gridUnit;
        }
        xi += gridUnit;
    }

    if (generateDistanceField){
        unsigned int rr =  sizeX * sizeY * sizeZ;

        #pragma omp parallel for
        for (unsigned int n = 0; n < rr; n++){
            unsigned int k = (n % (sizeY*sizeZ))%sizeZ;
            unsigned int j = ((n-k)/sizeZ)%sizeY;
            unsigned int i = ((n-k)/sizeZ - j)/sizeY;
            ///
            isInside(i,j,k) = tree.isInside(grid(i,j,k), 3);
            //isInside(i,j,k) = tree.isInsidePseudoRandom(grid(i,j,k), 3);
            ///
        }

        for (unsigned int i = 0; i < sizeX; i++){
            for (unsigned int j = 0; j < sizeY; j++){
                for (unsigned int k = 0; k < sizeZ; k++){
                    if (isInside(i,j,k)){
                        insidePoints.push_back(grid(i,j,k));
                        mapping(i,j,k) = inside;
                        inside++;
                    }
                }
            }
        }


        // compute values
        //Eigen::VectorXd S = m.getSignedDistance(GV);

		distances = getUnsignedDistances(insidePoints, tree);

        for (unsigned int i = 0; i < sizeX; i++){
            for (unsigned int j = 0; j < sizeY; j++){
                for (unsigned int k = 0; k < sizeZ; k++){
                    if (isInside(i,j,k)){
                        assert(mapping(i,j,k) >= 0);
                        distanceField(i,j,k) = -distances[mapping(i,j,k)];
                    }
                }
            }
        }
    }
}

void Engine::calculateGridWeights(Grid& g, const Array3D<Point3d> &grid, const Array3D<gridreal> &distanceField, const Dcel& d, double kernelDistance, bool tolerance, const Vec3d &target, std::set<const Dcel::Face*>& savedFaces){
	Point3i res(grid.sizeX(), grid.sizeY(), grid.sizeZ());
	Point3d nGmin(grid(0,0,0));
	Point3d nGmax(grid(res.x()-1, res.y()-1, res.z()-1));
    g = Grid(res, grid, distanceField, nGmin, nGmax);
    g.setTarget(target);
    g.calculateWeightsAndFreezeKernel(d, kernelDistance, tolerance, savedFaces);
    Energy e(g);
    e.calculateFullBoxValues(g);
}

Array3D<gridreal> Engine::generateGrid(Grid& g, const Dcel& d, double kernelDistance, bool tolerance, const Vec3d &target, std::set<const Dcel::Face*>& savedFaces) {
    SimpleEigenMesh m(d);
	Array3D<Point3d> grid;
    Array3D<gridreal> distanceField;
    Engine::generateGridAndDistanceField(grid, distanceField, m);
    calculateGridWeights(g, grid, distanceField, d, kernelDistance, tolerance, target, savedFaces);
    return distanceField;
}

void Engine::setTrianglesTargets(Dcel scaled[]) {
    for (Dcel::FaceIterator fit = scaled[0].faceBegin(); fit != scaled[0].faceEnd(); ++fit){
		Vec3d n = (*fit)->normal();
        double angle = n.dot(XYZ[0]);
        int k = 0;
        for (unsigned int i = 1; i < 18; i++){
            if (n.dot(XYZ[i]) > angle){
                angle = n.dot(XYZ[i]);
                k = i;
            }
        }
        if (k < 6){
            (*fit)->setFlag(0);
			scaled[1].face((*fit)->id())->setFlag(0);
			scaled[2].face((*fit)->id())->setFlag(0);
			scaled[3].face((*fit)->id())->setFlag(0);
        }
        else if (k < 10) {
            (*fit)->setFlag(1);
			scaled[1].face((*fit)->id())->setFlag(1);
			scaled[2].face((*fit)->id())->setFlag(1);
			scaled[3].face((*fit)->id())->setFlag(1);
        }
        else if (k < 14) {
            (*fit)->setFlag(2);
			scaled[1].face((*fit)->id())->setFlag(2);
			scaled[2].face((*fit)->id())->setFlag(2);
			scaled[3].face((*fit)->id())->setFlag(2);
        }
        else if (k < 18) {
            (*fit)->setFlag(3);
			scaled[1].face((*fit)->id())->setFlag(3);
			scaled[2].face((*fit)->id())->setFlag(3);
			scaled[3].face((*fit)->id())->setFlag(3);
        }
    }
}

void Engine::addBox(BoxList& boxList, const Vec3d target, const Dcel::Face* f, const Eigen::Matrix3d& rot){
        Box3D box;
        box.setTarget(target);
		Point3d p1 = f->outerHalfEdge()->fromVertex()->coordinate();
		Point3d p2 = f->outerHalfEdge()->toVertex()->coordinate();
		Point3d p3 = f->outerHalfEdge()->next()->toVertex()->coordinate();
		Point3d bmin = p1;
        bmin = bmin.min(p2);
        bmin = bmin.min(p3);
        bmin = bmin - 1;
		Point3d bmax = p1;
        bmax = bmax.max(p2);
        bmax = bmax.max(p3);
        bmax = bmax + 1;
        box.setMin(bmin);
        box.setMax(bmax);
        box.setColor(colorOfNormal(target));
        box.setConstraint1(p1);
        box.setConstraint2(p2);
        box.setConstraint3(p3);
        box.setRotationMatrix(rot);
        boxList.addBox(box);
}

void Engine::calculateDecimatedBoxes(BoxList& boxList, const Dcel& d, const Eigen::VectorXi &mapping, const std::set<int>& coveredFaces, const Eigen::Matrix3d& rot, int orientation, bool onlyTarget, const Vec3d& target) {
    std::vector<int> facesToCover;
    for (unsigned int i = 0; i < mapping.size(); i++){
        if (coveredFaces.find(mapping(i)) == coveredFaces.end()){
            facesToCover.push_back(mapping(i));
        }
    }

    for (unsigned int i = 0; i < facesToCover.size(); i++){
		const Dcel::Face* f = d.face(facesToCover[i]);
		Vec3d n =f->normal();
		Vec3d closestTarget = getClosestTarget(n);
        if (!onlyTarget || (onlyTarget && closestTarget == target)){
			if (orientation<0 || f->flag()==orientation){
                addBox(boxList, closestTarget, f, rot);
            }
        }
    }
}

void Engine::calculateInitialBoxes(BoxList& boxList, const Dcel& d, const Eigen::Matrix3d &rot, bool onlyTarget, const Vec3d& target) {
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
		Vec3d n =f->normal();
		Vec3d closestTarget = getClosestTarget(n);
        if (!onlyTarget || (onlyTarget && closestTarget == target)){
            addBox(boxList, closestTarget, f, rot);
        }
    }
}

void Engine::expandBoxes(BoxList& boxList, const Grid& g, bool limit, const Point3d& limits, bool printTimes) {
    Energy e(g);
    Timer total("Boxlist expanding");
    int np = boxList.getNumberBoxes();
    Timer t("");
    #pragma omp parallel for schedule(dynamic, 2)
    for (int i = 0; i < np; i++){
        Box3D b = boxList.getBox(i);
        if (printTimes){
            std::cerr << "Minimization " << i << " box.\n";
            t = Timer("");
        }
        //e.gradientDiscend(b);
        if (!limit)
            e.BFGS(b);
        else{
			//put the Z limit to the milling direction of the box b
			//the other two directions will have X and Y accordingly
			//they can be switched at will
			Point3d actualLimits(std::min(limits.x(), limits.y()), std::min(limits.x(), limits.y()), std::min(limits.x(), limits.y())); //all the limits set to min(X, Y)
            bool find = false;
            for (unsigned int i = 0; i < 3 && !find; i++){
                if (XYZ[i] == b.getTarget() || XYZ[i+3] == b.getTarget()){
					actualLimits[i] = limits.z(); //the proper one is set to Z
                    find = true;
					actualLimits[(i+1)%3] = std::max(limits.x(), limits.y()); //one of the other two is set to max(X, Y)
                }
            }
            assert(find);
            e.BFGS(b,actualLimits);
        }
        if (printTimes){
            t.stop();
            std::cerr << "Box: " << i << "Time: " << t.delay() << "\n";
        }
        boxList.setBox(i, b);

    }
    total.stopAndPrint();
    std::cerr << "Number Boxes: " << np << "\n";
}

void Engine::createVectorTriples(std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, const BoxList& boxList, const Dcel& d) {
	cgal::AABBTree3 t(d);


    // creating vector of pairs

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
		t.containedDcelFaces(covered, b);

        std::list<const Dcel::Face*>::iterator it = covered.begin();
        while (it != covered.end()) {
            const Dcel::Face* f = *it;
			Point3d p1 = f->vertex1()->coordinate(), p2 = f->vertex2()->coordinate(), p3 = f->vertex3()->coordinate();

            if (!b.isIntern(p1) || !b.isIntern(p2) || !b.isIntern(p3)) {
                it =covered.erase(it);
            }
            else ++it;
        }

        std::vector<bool> v;
        int n = covered.size();
		v.resize(d.numberFaces(), false);
        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* f = *it;
			v[f->id()] = true;
        }
        std::tuple<int, Box3D, std::vector<bool> > triple (n, boxList.getBox(i), v);
        vectorTriples.push_back(triple);
    }
}


int Engine::minimalCoveringNonOptimal(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces){

    //ordering vector of triples
    struct triplesOrdering {
        bool operator ()(const std::tuple<int, Box3D, std::vector<bool> >& a, const std::tuple<int, Box3D, std::vector<bool> >& b) {
            if (std::get<0>(a) < std::get<0>(b))
                return true;
            if (std::get<0>(a) == std::get<0>(b))
                return ((std::get<1>(a).getVolume()) < (std::get<1>(b).getVolume()));
            return false;
        }
    };

    std::vector<std::tuple<int, Box3D, std::vector<bool> > > vectorTriples0;
    for (unsigned int i = 0; i < vectorTriples.size(); i++){
        Box3D b = std::get<1>(vectorTriples[i]);
        if (b.getRotationMatrix() == Eigen::Matrix3d::Identity()){
            vectorTriples0.push_back(vectorTriples[i]);
            vectorTriples.erase(vectorTriples.begin() + i);
            i--;
        }
    }
    std::sort(vectorTriples0.begin(), vectorTriples0.end(), triplesOrdering());


    std::sort(vectorTriples.begin(), vectorTriples.end(), triplesOrdering());

    vectorTriples.insert(vectorTriples.end(), vectorTriples0.begin(), vectorTriples0.end());

    //create m
    std::vector<std::vector<bool> > m;
    m.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++)
        m.push_back(std::get<2>(vectorTriples[i]));

    //create sums
    std::vector<unsigned int> sums;
    sums.resize(numberFaces, 0);
    for (unsigned j = 0; j < numberFaces; j++){
        for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++){
            sums[j] += m[i][j];
        }
    }

    //calculating erasable elements
    std::vector<unsigned int> eliminate;
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++){
        bool b = true;
        for (unsigned int j = 0; j < numberFaces; j++)
            if (sums[j]-m[i][j] < 1 && sums[j]!=0)
                b = false;
        if (b){
            for (unsigned int j = 0; j < numberFaces; j++)
                sums[j]-=m[i][j];
            eliminate.push_back(i);
        }
    }


    //
    for (int i = eliminate.size()-1; i >= 0; i--){
        vectorTriples.erase(vectorTriples.begin() + eliminate[i]);
    }

    vectorTriples0.clear();
    for (unsigned int i = 0; i < vectorTriples.size(); i++){
        Box3D b = std::get<1>(vectorTriples[i]);
        if (b.getRotationMatrix() == Eigen::Matrix3d::Identity()){
            vectorTriples0.push_back(vectorTriples[i]);
            vectorTriples.erase(vectorTriples.begin() + i);
            i--;
        }
    }
    vectorTriples.insert(vectorTriples.begin(), vectorTriples0.begin(), vectorTriples0.end());

    int n = boxList.getNumberBoxes();
    boxList.clearBoxes();
    for (unsigned int i = 0; i < vectorTriples.size(); i++){
        boxList.addBox(std::get<1>(vectorTriples[i]));
    }
    return n-eliminate.size();
}

int Engine::minimalCoveringNonOptimal(BoxList& boxList, const Dcel& d) {
    Dcel scaled0(d);
    Eigen::Matrix3d m[ORIENTATIONS];
    m[0] = Eigen::Matrix3d::Identity();
	cgal::AABBTree3 t0(scaled0);
    #if ORIENTATIONS > 1
	getRotationMatrix(Vec3d(0,0,1), 0.785398, m[1]);
    Dcel scaled1(d);
    scaled1.rotate(m[1]);
	getRotationMatrix(Vec3d(0,0,-1), 0.785398, m[1]);
	CGALInterface::AABBTree3 t1(scaled1);
	getRotationMatrix(Vec3d(1,0,0), 0.785398, m[2]);
    Dcel scaled2(d);
    scaled2.rotate(m[2]);
	getRotationMatrix(Vec3d(-1,0,0), 0.785398, m[2]);
	CGALInterface::AABBTree3 t2(scaled2);
	getRotationMatrix(Vec3d(0,1,0), 0.785398, m[3]);
    Dcel scaled3(d);
    scaled3.rotate(m[3]);
	getRotationMatrix(Vec3d(0,-1,0), 0.785398, m[3]);
	CGALInterface::AABBTree3 t3(scaled3);
    #endif

    std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples;

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
        if (b.getRotationMatrix() == m[0])
			t0.containedDcelFaces(covered, b);
        #if ORIENTATIONS > 1
        else if (b.getRotationMatrix() == m[1])
            t1.getIntersectedDcelFaces(covered, b);
        else if (b.getRotationMatrix() == m[2])
            t2.getIntersectedDcelFaces(covered, b);
        else if (b.getRotationMatrix() == m[3])
            t3.getIntersectedDcelFaces(covered, b);
        #endif
        else assert(0);

        std::list<const Dcel::Face*>::iterator it = covered.begin();
        while (it != covered.end()) {
            const Dcel::Face* f = *it;
			Point3d p1 = f->vertex1()->coordinate(), p2 = f->vertex2()->coordinate(), p3 = f->vertex3()->coordinate();

            if (!b.isIntern(p1) || !b.isIntern(p2) || !b.isIntern(p3)) {
                it =covered.erase(it);
            }
            else ++it;
        }

        std::vector<bool> v;
        int n = covered.size();
		v.resize(d.numberFaces(), false);
        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* f = *it;
			v[f->id()] = true;
        }
        std::tuple<int, Box3D, std::vector<bool> > triple (n, boxList.getBox(i), v);
        vectorTriples.push_back(triple);
    }

	return minimalCoveringNonOptimal(boxList, vectorTriples, d.numberFaces());
}


bool Engine::minimalCovering(BoxList& boxList, const Dcel& d) {
    #ifdef GUROBI_DEFINED
    unsigned int nBoxes = boxList.getNumberBoxes();
	unsigned int nTris = d.numberFaces();
    Array2D<int> B(nBoxes+1, nTris, 0);
	cgal::AABBTree3 aabb(d);
    for (unsigned int i = 0; i < nBoxes; i++){
		std::list<const Dcel::Face*> containedFaces = aabb.completelyContainedDcelFaces(boxList.getBox(i));
        for (const Dcel::Face* f : containedFaces){
			B(i,f->id()) = 1;
        }
    }

    //this piece of code allows to find a solution also if there are uncovered triangles.
    //it creates a "dummy box" for every uncovered triangles
    bool bb = false;
	for (unsigned int j = 0; j < B.sizeY(); j++){
        int sum = 0;
		for (unsigned int i = 0; i < B.sizeX() && sum == 0; i++){
            sum += B(i,j);
        }
        if (sum == 0){
            bb = true;
            B(nBoxes, j) = 1;
        }
    }
    if (bb)
        std::cerr << "Warning: Uncovered triangles by best boxes.\n";

    try {
        GRBEnv env;
        GRBModel model(env);

        //x
        GRBVar* x = nullptr;
        x = model.addVars(nBoxes+1, GRB_BINARY);

        //constraints
        for (unsigned int j = 0; j < nTris; j++){
            GRBLinExpr line = 0;
			for (unsigned int i = 0; i < B.sizeX(); i++){
                line += B(i,j) * x[i];
            }
            model.addConstr(line >= 1);
        }

        //obj
        GRBLinExpr obj = 0;
        for (unsigned int i = 0; i < nBoxes; i++)
            obj += x[i];
        model.setObjective(obj, GRB_MINIMIZE);
        Timer tGurobi("tGurobi");
        model.optimize();
        tGurobi.stopAndPrint();

        unsigned int deleted = 0;
        for (int i = nBoxes-1; i >= 0; i--){
            if (x[i].get(GRB_DoubleAttr_X) == 0){
                boxList.removeBox(i);
                deleted++;
            }
        }
        std::cerr << "N survived boxes: " << nBoxes - deleted << "\n";
        return bb;
    }
    catch (GRBException e) {
        std::cerr << "Gurobi Exception\n" << e.getErrorCode() << " : " << e.getMessage() << std::endl;
        throw std::runtime_error("Optimization failed.");
    }
    catch (...) {
        std::cerr << "Unknown Gurobi Optimization error!" << std::endl;
        throw std::runtime_error("Optimization failed.");
    }
    #else
    return deleteBoxesNonOptimal(boxList, d);
    #endif
}

bool Engine::secondMinimalCovering(BoxList& bestList, BoxList& boxList, const Dcel& d) {
    #ifdef GUROBI_DEFINED
    unsigned int nBoxes = boxList.getNumberBoxes();
	unsigned int nTris = d.numberFaces();
    Array2D<int> B(nBoxes+1, nTris, 0);
	cgal::AABBTree3 aabb(d);
    for (unsigned int i = 0; i < bestList.size(); i++){
		std::list<const Dcel::Face*> containedFaces = aabb.completelyContainedDcelFaces(bestList.getBox(i));
        for (const Dcel::Face* f : containedFaces){
			B(nBoxes,f->id()) = 1;
        }
    }
    for (unsigned int i = 0; i < nBoxes; i++){
		std::list<const Dcel::Face*> containedFaces = aabb.completelyContainedDcelFaces(boxList.getBox(i));
        for (const Dcel::Face* f : containedFaces){
			B(i,f->id()) = 1;
        }
    }


    //this piece of code allows to find a solution also if there are uncovered triangles.
    //it creates a "dummy box" for every uncovered triangles
    bool bb = false;
	for (unsigned int j = 0; j < B.sizeY(); j++){
        int sum = 0;

        if (B(nBoxes, j) != 1) {
			for (unsigned int i = 0; i < B.sizeX() && sum == 0; i++){
                sum += B(i,j);
            }
            if (sum == 0){
                bb = true;
                B(nBoxes, j) = 1;
            }
        }
    }
    if (bb)
        std::cerr << "WARNING: Uncovered triangles.\n";

    try {
        GRBEnv env;
        GRBModel model(env);

        //x
        GRBVar* x = nullptr;
        x = model.addVars(nBoxes+1, GRB_BINARY);

        //constraints
        for (unsigned int j = 0; j < nTris; j++){
            GRBLinExpr line = 0;
			for (unsigned int i = 0; i < B.sizeX(); i++){
                line += B(i,j) * x[i];
            }
            model.addConstr(line >= 1);
        }

        //obj
        GRBLinExpr obj = 0;
        for (unsigned int i = 0; i < nBoxes; i++)
            obj += x[i];
        model.setObjective(obj, GRB_MINIMIZE);
        Timer tGurobi("tGurobi");
        model.optimize();
        tGurobi.stopAndPrint();

        unsigned int deleted = 0;
        for (int i = nBoxes-1; i >= 0; i--){
            if (x[i].get(GRB_DoubleAttr_X) == 0){
                boxList.removeBox(i);
                deleted++;
            }
        }
        std::cerr << "N survived boxes: " << nBoxes - deleted << "\n";
        return bb;
    }
    catch (GRBException e) {
        std::cerr << "Gurobi Exception\n" << e.getErrorCode() << " : " << e.getMessage() << std::endl;
        throw std::runtime_error("Optimization failed.");
    }
    catch (...) {
        std::cerr << "Unknown Gurobi Optimization error!" << std::endl;
        throw std::runtime_error("Optimization failed.");
    }
    #else
    return deleteBoxesNonOptimal(boxList, d);
    #endif
}


int Engine::deleteBoxesGSC(BoxList& boxList, const Dcel& d) {
    unsigned int nBoxes = boxList.getNumberBoxes();
    std::map<int, std::set<int>> F;
    std::set<int> W;
    std::vector<int> C;

	for (unsigned int i = 0; i < d.numberFaces(); i++)
        W.insert(i);

	cgal::AABBTree3 aabb(d);
    for (unsigned int i = 0; i < nBoxes; i++){
		std::list<const Dcel::Face*> containedFaces = aabb.completelyContainedDcelFaces(boxList.getBox(i));
        std::set<int> s;
        for (const Dcel::Face* f : containedFaces){
			s.insert(f->id());
        }
        F[i] = s;
    }
    Timer t("AAA");
    while (W.size() > 0){
        std::set<int> S;
        int a = -1;
        int found = -1;
        for (std::pair<int, std::set<int>> p : F){
            std::set<int> inters = cg3::intersection(p.second, W);
            if ((int)inters.size() > a){
                a = inters.size();
                found = p.first;
                S = p.second;
            }
        }
        assert(a > 0 && found >= 0);
        W = cg3::difference(W, S);
        F.erase(found);
        C.push_back(found);
    }

    unsigned int deleted = 0;
    std::map<int,std::set<int>>::reverse_iterator rit;
    for (rit=F.rbegin(); rit!=F.rend(); ++rit){
        boxList.removeBox(rit->first);
        deleted++;
    }
    t.stopAndPrint();
    return nBoxes - deleted;
}


double Engine::optimize(BoxList& solutions, Dcel& d, double kernelDistance, bool limit, Point3d limits, bool tolerance, bool onlyNearestTarget, double areaTolerance, double angleTolerance, bool file, bool decimate) {
    assert(kernelDistance >= 0 && kernelDistance <= 1);
    solutions.clearBoxes();
    Dcel scaled[ORIENTATIONS];
    Eigen::Matrix3d m[ORIENTATIONS];

    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        scaled[i] = d;
        m[i] = Engine::rotateDcelAlreadyScaled(scaled[i], i);
    }

    #if ORIENTATIONS > 1
    if (onlyNearestTarget)
        Engine::setTrianglesTargets(scaled);
    #endif

    Grid g[ORIENTATIONS][TARGETS];
    BoxList bl[ORIENTATIONS][TARGETS];
    std::set<int> coveredFaces;
    int factor = 1024;

	unsigned int numberFaces = d.numberFaces();
    if (decimate){
        while (numberFaces/factor  < STARTING_NUMBER_FACES && factor != 1)
            factor/=2;
        numberFaces/=factor;
    }
	cgal::AABBTree3 aabb[ORIENTATIONS];
    for (unsigned int i = 0; i < ORIENTATIONS; i++)
		aabb[i] = cgal::AABBTree3(scaled[i]);
    for (unsigned int i = 0; i < ORIENTATIONS; ++i){
        bool first = true;
        if (file) {
            double totalTimeGG = 0;
			Array3D<Point3d> grid;
            Array3D<gridreal> distanceField;
            for (unsigned int j = 0; j < TARGETS; ++j) {
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    std::set<const Dcel::Face*> flippedFaces, savedFaces;
                    Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                    Grid g;
                    Timer gg("Generating Grid");
                    //distanceField = Engine::generateGrid(g, scaled[i], kernelDistance, tolerance, XYZ[j], savedFaces);
                    if (first) {
                        SimpleEigenMesh m(scaled[i]);
                        Engine::generateGridAndDistanceField(grid, distanceField, m);
                        calculateGridWeights(g, grid, distanceField, d, kernelDistance, tolerance, XYZ[j], savedFaces);
                        first = false;
                    }
                    else {
                        Engine::calculateGridWeights(g, grid, distanceField, d, kernelDistance, tolerance, XYZ[j], savedFaces);
                    }
                    gg.stopAndPrint();
                    totalTimeGG += gg.delay();
                    g.resetSignedDistances();
                    std::stringstream ss ;
                    ss << "grid" << i << "_" << j << ".bin";
                    std::ofstream myfile;
                    myfile.open (ss.str(), std::ios::out | std::ios::binary);
                    g.serialize(myfile);
                    myfile.close();
                    std::cerr << "Generated grid or " << i << " t " << j << "\n";
                #ifdef USE_2D_ONLY
                }
                #endif
            }
            std::cerr << "Total time generating Grids: " << totalTimeGG << "\n";
        }
        else {
			Array3D<Point3d> grid;
            Array3D<gridreal> distanceField;
            SimpleEigenMesh m(scaled[i]);
            Engine::generateGridAndDistanceField(grid, distanceField, m);
            # pragma omp parallel for
            for (unsigned int j = 0; j < TARGETS; ++j) {
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    std::set<const Dcel::Face*> flippedFaces, savedFaces;
                    Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                    Engine::calculateGridWeights(g[i][j], grid, distanceField, d, kernelDistance, tolerance, XYZ[j], savedFaces);
                    g[i][j].resetSignedDistances();
                    std::cerr << "Generated grid or " << i << " t " << j << "\n";
                #ifdef USE_2D_ONLY
                }
                #endif
            }
        }
    }
    bool end = false;

    double totalTbg = 0;
	while (coveredFaces.size() < scaled[0].numberFaces() && !end){
        BoxList tmp[ORIENTATIONS][TARGETS];
        Eigen::VectorXi faces[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            EigenMesh m(scaled[i]);
            libigl::decimateMesh(m, numberFaces, faces[i]);
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    std::cerr << "Calculating Boxes\n";
                    #if ORIENTATIONS > 1
                    if (onlyNearestTarget){
                        Engine::calculateDecimatedBoxes(tmp[i][j],scaled[i], faces[i], coveredFaces, m[i], i, true, XYZ[j]);
                    }
                    else
                    #endif
                        Engine::calculateDecimatedBoxes(tmp[i][j],scaled[i], faces[i], coveredFaces, m[i], -1, onlyNearestTarget, XYZ[j]);
                        //Engine::calculateDecimatedBoxes(tmp[i][j],scaled[i], faces[i], coveredFaces, m[i], -1, false);
                    if (tmp[i][j].getNumberBoxes() > 0){

                        if (file) {
                            Grid g;
                            std::stringstream ss ;
                            ss << "grid" << i << "_" << j << ".bin";
                            std::ifstream myfile;
                            myfile.open (ss.str(), std::ios::in | std::ios::binary);
                            g.deserialize(myfile);
                            myfile.close();
                            std::cerr << "Starting boxes growth\n";
                            Timer tt("Boxes Growth");
                            Engine::expandBoxes(tmp[i][j], g, limit, limits);
                            tt.stop();
                            totalTbg += tt.delay();
                            std::cerr << "Orientation: " << i << " Target: " << j << " completed.\n";
                        }
                        else {
                            std::cerr << "Starting boxes growth\n";
                            Timer tt("Boxes Growth");
                            Engine::expandBoxes(tmp[i][j], g[i][j], limit, limits);
                            tt.stop();
                            totalTbg += tt.delay();
                            std::cerr << "Orientation: " << i << " Target: " << j << " completed.\n";
                        }
                    }
                    else {
                        std::cerr << "Orientation: " << i << " Target: " << j << " no boxes to expand.\n";
                    }
                #ifdef USE_2D_ONLY
                }
                #endif
            }
        }

        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    for (unsigned int k = 0; k < tmp[i][j].getNumberBoxes(); ++k){
                        std::list<const Dcel::Face*> list;
						aabb[i].completelyContainedDcelFaces(list, tmp[i][j].getBox(k));
                        for (std::list<const Dcel::Face*>::iterator it = list.begin(); it != list.end(); ++it){
							coveredFaces.insert((*it)->id());
                        }
                    }
                #ifdef USE_2D_ONLY
                }
                #endif
            }
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    bl[i][j].insert(tmp[i][j]);
                #ifdef USE_2D_ONLY
                }
                #endif

            }
        }

        std::cerr << "Starting Number Faces: " << numberFaces << "; Total Covered Faces: " << coveredFaces.size() << "\n";
		std::cerr << "Target: " << scaled[0].numberFaces() << "\n";
		if (numberFaces == scaled[0].numberFaces()) {
            end = true;
			if (coveredFaces.size() != scaled[0].numberFaces()){
                std::cerr << "WARNING: Not every face has been covered by a box.\n";
				std::cerr << "Number uncovered faces: " << scaled[0].numberFaces() - coveredFaces.size() << "\n";
                for (Dcel::Face* f : d.faceIterator()){
					if (coveredFaces.find(f->id()) == coveredFaces.end()){
						std::cerr << "Uncovered face id: " << f->id() << "\n";
                        f->setColor(Color(0,0,0));
                    }
                }
            }
        }
        numberFaces*=2;
		if (numberFaces > scaled[0].numberFaces())
			numberFaces = scaled[0].numberFaces();
    }
    std::cerr << "Total time Boxes Growth: " << totalTbg << "\n";

    if (file){
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                #ifdef USE_2D_ONLY
                if (j != 1 && j != 4){
                #endif
                    std::stringstream ss ;
                    ss << "grid" << i << "_" << j << ".bin";
                    std::remove(ss.str().c_str());
                #ifdef USE_2D_ONLY
                }
                #endif
            }
        }
    }
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        for (unsigned int j = 0; j < TARGETS; ++j){
            #ifdef USE_2D_ONLY
            if (j != 1 && j != 4){
            #endif
                solutions.insert(bl[i][j]);
            #ifdef USE_2D_ONLY
            }
            #endif
        }
    }
    return totalTbg;
}

/**
 * @brief Engine::createAndMinimizeAllBoxes
 * @param bl
 * @param d: Dcel already scaled
 * @param kernelDistance
 * @param heightfields
 * @param onlyNearestTarget
 * @param areaTolerance
 * @param angleTolerance
 */
void Engine::optimizeAndDeleteBoxes(BoxList& solutions, Dcel& d, double kernelDistance, bool limit, Point3d limits, bool tolerance, bool onlyNearestTarget, double areaTolerance, double angleTolerance, bool file, bool decimate, BoxList& allSolutions) {
    optimize(solutions, d, kernelDistance, limit, limits, tolerance, onlyNearestTarget, areaTolerance, angleTolerance, file, decimate);
    allSolutions=solutions;

    //
	cg3::cgal::AABBTree3 tree(d);
    BoxList bestSolutions, otherSolutions;
    for (const Box3D& b : solutions){
        int nEdges = 0;
        for (unsigned int i = 0; i < 6; i++){
			BoundingBox3 bb = b;
            if (i < 3){
                bb(i+3) = bb(i) + CG3_EPSILON;

            }
            else {
                bb(i-3) = bb(i) - CG3_EPSILON;
            }
			if (tree.numberIntersectedPrimitives(bb) > 0)
                nEdges++;
        }
        if (nEdges <= 1)
            bestSolutions.addBox(b);
        else
            otherSolutions.addBox(b);
    }
    std::cerr << "Number of best solutions : " << bestSolutions.size();
    std::cerr << "Number of other solutions : " << otherSolutions.size();
    Timer tGurobi("Gurobi");
    bool otherAreNecessary = minimalCovering(bestSolutions, d);
    if (otherAreNecessary){
        std::cerr << "Best Solutions were not necessary.\n";
        secondMinimalCovering(bestSolutions, otherSolutions, d);
    }
    else {
        std::cerr << "Best Solutions were enough!!!\n";
    }
    tGurobi.stopAndPrint();
    //

    /*Timer tGurobi("Gurobi");
    deleteBoxes(solutions, d);
    tGurobi.stopAndPrint();*/
}

void Engine::boxPostProcessing(BoxList& solutions, const Dcel& d) {
	cgal::AABBTree3 tree(d);
    for (int bi = solutions.getNumberBoxes()-1; bi >= 0; bi--) {
        Box3D b = solutions.getBox(bi);
		std::list<const Dcel::Face*> list = tree.containedDcelFaces(b);
		std::vector<std::set<const Dcel::Face*> > connectedComponents = dcelAlgorithms::connectedComponents(list.begin(), list.end());
        if (connectedComponents.size() > 1){
            std::cerr << "Box " << bi << " has " << connectedComponents.size() << " connected components\n";
            std::vector<Box3D> tmpvect = splitBoxWithMoreThanOneConnectedComponent(b, connectedComponents);
            solutions.removeBox(bi);
            for (Box3D& bb : tmpvect){
                bb.generateEigenMesh();
                solutions.addBox(bb);
            }
        }
    }
}

std::vector<Box3D> Engine::splitBoxWithMoreThanOneConnectedComponent(const Box3D& originalBox, const std::vector<std::set<const Dcel::Face*> > &connectedComponents) {
    std::vector<Box3D> splittedBoxes;
    for (unsigned int i = 0; i < connectedComponents.size(); i++){
        Box3D b;
        b.setTarget(originalBox.getTarget());
        b.setColor(originalBox.getColor());
        bool first = true;
        for (const Dcel::Face* f : connectedComponents[i]){
            for (const Dcel::Vertex* v : f->incidentVertexIterator()){
                if (first){
                    first = false;
					b.setMin(v->coordinate());
					b.setMax(v->coordinate());
                }
                else {
					b.setMin(b.min().min(v->coordinate()));
					b.setMax(b.max().max(v->coordinate()));
                }
            }
        }
        for (unsigned int orientation = 0; orientation < 3; orientation++){
            if (originalBox.getTarget() == XYZ[orientation] || originalBox.getTarget() == XYZ[orientation+3]){
                b.min()[orientation] = originalBox.min()[orientation];
                b.max()[orientation] = originalBox.max()[orientation];
            }
        }
        splittedBoxes.push_back(b);
    }
    return splittedBoxes;
}

bool checkNewBox(const Box3D& tmp, Box3D& b2, std::vector<unsigned int>& trianglesCovered, const cgal::AABBTree3& tree){
    std::list<unsigned int> newTriangles;
	tree.completelyContainedDcelFaces(newTriangles, tmp);
    std::set<unsigned int> uncovered = cg3::difference(b2.getTrianglesCovered(), std::set<unsigned int>(newTriangles.begin(), newTriangles.end()));
    bool shrink = true;
    for (unsigned int t : uncovered){
        if (trianglesCovered[t] == 1)
            shrink = false;
    }
    if (shrink){
        b2 = tmp;
        b2.setTrianglesCovered(std::set<unsigned int>(newTriangles.begin(), newTriangles.end()));
        for (unsigned int t : uncovered){
            trianglesCovered[t]--;
        }
        return true;
    }
    return false;
}

void Engine::stupidSnapping(const Dcel& d, BoxList& solutions, double epsilon) {
	BoundingBox3 bb = d.boundingBox();
    for (unsigned int i = 0; i < solutions.getNumberBoxes(); i++){
        Box3D b1 = solutions.getBox(i);
        for (unsigned int coord = 0; coord < 3; coord++) {
            if (std::abs(b1(coord)-bb(coord)) < epsilon) {
                b1(coord) = bb(coord);
            }
            if (std::abs(b1(coord)-bb(coord+3)) < epsilon){
                b1(coord+3) = bb(coord);
            }
            if (std::abs(b1(coord+3)-bb(coord)) < epsilon){
                b1(coord) = bb(coord+3);
            }
            if (std::abs(b1(coord+3)-bb(coord+3)) < epsilon){
                b1(coord+3) = bb(coord+3);
            }
        }
        solutions.setBox(i, b1);
    }


    for (unsigned int i = 0; i < solutions.getNumberBoxes()-1; i++){
        Box3D b1 = solutions.getBox(i);
        for (unsigned int j = i+1; j < solutions.getNumberBoxes(); j++){
            Box3D b2 = solutions.getBox(j);
            for (unsigned int coord = 0; coord < 3; coord++) {
                if (std::abs(b1(coord)-b2(coord)) < epsilon) {
                    if (b1(coord) < b2(coord))
                        b2(coord) = b1(coord);
                    else
                        b1(coord) = b2(coord);
                }
                if (std::abs(b1(coord)-b2(coord+3)) < epsilon){
                    b2(coord+3) = b1(coord);
                }
                if (std::abs(b1(coord+3)-b2(coord)) < epsilon){
                    b2(coord) = b1(coord+3);
                }
                if (std::abs(b1(coord+3)-b2(coord+3)) < epsilon){
                    if (b1(coord+3) > b2(coord+3))
                        b2(coord+3) = b1(coord+3);
                    else
                        b1(coord+3) = b2(coord+3);
                }
            }
            solutions.setBox(j, b2);
        }
    }
}

bool Engine::smartSnapping(const Box3D& b1, Box3D& b2, std::vector<unsigned int>& trianglesCovered, const cgal::AABBTree3& tree) {
    bool found = false;
    Box3D tmp = b2;
	if (isInBounds(b2.minX(), b1.minX(), b1.maxX()) && b2.maxX() > b1.maxX()){
		tmp.setMinX(b1.maxX());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
        else
            tmp = b2;
    }
	if (isInBounds(b2.minY(), b1.minY(), b1.maxY()) && b2.maxY() > b1.maxY()) {
		tmp.setMinY(b1.maxY());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
        else
            tmp = b2;
    }
	if (isInBounds(b2.minZ(), b1.minZ(), b1.maxZ()) && b2.maxZ() > b1.maxZ()) {
		tmp.setMinZ(b1.maxZ());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
        else
            tmp = b2;
    }
	if (isInBounds(b2.maxX(), b1.minX(), b1.maxX()) && b2.minX() < b1.minX()){
		tmp.setMaxX(b1.minX());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
        else
            tmp = b2;
    }
	if (isInBounds(b2.maxY(), b1.minY(), b1.maxY()) && b2.minY() < b1.minY()){
		tmp.setMaxY(b1.minY());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
        else
            tmp = b2;
    }
	if (isInBounds(b2.maxZ(), b1.minZ(), b1.maxZ()) && b2.minZ() < b1.minZ()){
		tmp.setMaxZ(b1.minZ());
        found = checkNewBox(tmp, b2, trianglesCovered, tree);
        if (found)
            return true;
    }
    return false;
}

void Engine::smartSnapping(const Dcel& d, BoxList& solutions) {
	cgal::AABBTree3 tree(d);
    solutions.calculateTrianglesCovered(tree);
	std::vector<unsigned int> trianglesCovered(d.numberFaces(), 0);
    for (unsigned int i = 0; i < solutions.getNumberBoxes(); i++){
        const std::set<unsigned int>& s = solutions[i].getTrianglesCovered();
        for (unsigned int j : s){
            trianglesCovered[j]++;
        }
    }
    // priority first to dangeorus intersections
    for (unsigned int i = 0; i < solutions.getNumberBoxes()-1; i++){
        Box3D b1 = solutions.getBox(i);
        for (unsigned int j = i+1; j < solutions.getNumberBoxes(); j++){
            Box3D b2 = solutions.getBox(j);
            if (Splitting::boxesIntersect(b1,b2)){
                if (Splitting::isDangerousIntersection(b1, b2, tree, false) ||
                        Splitting::isDangerousIntersection(b2, b1, tree, false)){
                    if (Engine::smartSnapping(b1, b2, trianglesCovered, tree)){
                        //std::cerr << "Smart snapping " << j << " in " << i << "\n";
                        solutions.setBox(j, b2);
                    }
                    else if (Engine::smartSnapping(b2, b1, trianglesCovered, tree)){
                        //std::cerr << "Smart snapping " << i << " in " << j << "\n";
                        solutions.setBox(i, b1);
                    }
                }
            }
        }
    }
    //
    for (unsigned int i = 0; i < solutions.getNumberBoxes()-1; i++){
        Box3D b1 = solutions.getBox(i);
        for (unsigned int j = i+1; j < solutions.getNumberBoxes(); j++){
            Box3D b2 = solutions.getBox(j);
            if (Splitting::boxesIntersect(b1,b2)){
                // no dangerous intersection
                    if (Engine::smartSnapping(b1, b2, trianglesCovered, tree)){
                        //std::cerr << "Smart snapping " << j << " in " << i << "\n";
                        solutions.setBox(j, b2);
                    }
                    else if (Engine::smartSnapping(b2, b1, trianglesCovered, tree)){
                        //std::cerr << "Smart snapping " << i << " in " << j << "\n";
                        solutions.setBox(i, b1);
                    }
            }
        }
    }

    solutions.generatePieces();
    solutions.calculateTrianglesCovered(tree);
    solutions.sortByTrianglesCovered();
}

void Engine::merging(const Dcel& d, BoxList& solutions) {
	cgal::AABBTree3 tree(d);
	std::vector<unsigned int> trianglesCovered(d.numberFaces(), 0);
    for (unsigned int i = 0; i < solutions.getNumberBoxes(); i++){
        const std::set<unsigned int>& s = solutions[i].getTrianglesCovered();
        for (unsigned int j : s){
            trianglesCovered[j]++;
        }
    }
    for (unsigned int i = 0; i < solutions.getNumberBoxes(); i++){
        for (unsigned int j = 0; j < solutions.getNumberBoxes(); j++){
            std::cerr << "i: " << i << "; j : " << j << "\n";
            if (i != j){
                Box3D& a = solutions[i];
                Box3D& b = solutions[j];
                if (solutions[i].getTarget() == solutions[j].getTarget() &&  Splitting::boxesIntersect(solutions[i], solutions[j])){
                    std::cerr << "Boxes " << i << " and " << j << " may be merged. \n";
                    int t = indexOfNormal(solutions[i].getTarget());
                    assert(t >= 0);
                    double baseA = a.getBaseLevel(), baseB = b.getBaseLevel();
                    if (baseA <= baseB){
                        if (baseA != baseB){
                            if (t < 3){
                                //x, y, z
                                Box3D tmpa = a;
                                tmpa.setBaseLevel(baseB);
                                std::list<unsigned int> newTrianglesA;
								tree.completelyContainedDcelFaces(newTrianglesA, tmpa);
                                std::set<unsigned int> nonCoveredTrianglesA(newTrianglesA.begin(), newTrianglesA.end());
                                nonCoveredTrianglesA = cg3::difference(a.getTrianglesCovered(), nonCoveredTrianglesA);
                                bool shrink = true;
                                for (unsigned int t : nonCoveredTrianglesA){
                                    if (trianglesCovered[t] == 1)
                                        shrink = false;
                                }
                                if (shrink){
                                    for (unsigned int t : nonCoveredTrianglesA){
                                        trianglesCovered[t]--;
                                    }
                                    a.setBaseLevel(baseB);
                                    a.setTrianglesCovered(std::set<unsigned int>(newTrianglesA.begin(), newTrianglesA.end()));
                                    std::cerr << "Box " << i << " shrinked to level of Box " << j << "\n";

                                    SimpleEigenMesh u = libigl::union_(a.getEigenMesh(), b.getEigenMesh());
                                    a.setEigenMesh(u);
                                    a.setTrianglesCovered(cg3::union_(a.getTrianglesCovered(), b.getTrianglesCovered()));
									a.setMin(u.boundingBox().min());
									a.setMax(u.boundingBox().max());
                                    solutions[i].setSplitted(true);
                                    solutions.removeBox(j);
                                    if (i > j)
                                        i--;
                                    j--;
                                }
                            }
                            else {
                                //-x, -y, -z
                                Box3D tmpb = b;
                                tmpb.setBaseLevel(baseA);
                                std::list<unsigned int> newTrianglesB;
								tree.completelyContainedDcelFaces(newTrianglesB, tmpb);
                                std::set<unsigned int> nonCoveredTrianglesB(newTrianglesB.begin(), newTrianglesB.end());
                                nonCoveredTrianglesB = cg3::difference(b.getTrianglesCovered(), nonCoveredTrianglesB);
                                bool shrink = true;
                                for (unsigned int t : nonCoveredTrianglesB){
                                    if (trianglesCovered[t] == 1)
                                        shrink = false;
                                }
                                if (shrink){
                                    for (unsigned int t : nonCoveredTrianglesB){
                                        trianglesCovered[t]--;
                                    }
                                    b.setBaseLevel(baseA);
                                    b.setTrianglesCovered(std::set<unsigned int>(newTrianglesB.begin(), newTrianglesB.end()));
                                    std::cerr << "Box " << j << " shrinked to level of Box " << i << "\n";

                                    SimpleEigenMesh u = libigl::union_(a.getEigenMesh(), b.getEigenMesh());
                                    a.setEigenMesh(u);
                                    a.setTrianglesCovered(cg3::union_(a.getTrianglesCovered(), b.getTrianglesCovered()));
									a.setMin(u.boundingBox().min());
									a.setMax(u.boundingBox().max());
                                    solutions[i].setSplitted(true);
                                    solutions.removeBox(j);
                                    if (i > j)
                                        i--;
                                    j--;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void Engine::deleteDuplicatedBoxes(BoxList& solutions) {
    for (unsigned int i = 0; i < solutions.getNumberBoxes()-1; i++){
        if (solutions.getBox(i).min() == solutions.getBox(i+1).min() &&
            solutions.getBox(i).max() == solutions.getBox(i+1).max()){
            solutions.removeBox(i);
            i--;
        }
    }
}

void Engine::booleanOperations(HeightfieldsList &he, SimpleEigenMesh &bc, BoxList &solutions, bool alternativeColors) {
    deleteDuplicatedBoxes(solutions);
    #ifdef CG3_USING_LIBIGL_CSGTREE
    igl::copyleft::cgal::CSGTree tree = libigl::eigenMeshToCSGTree(bc);
    #endif
    Timer timer("Boolean Operations");
    he.resize(solutions.getNumberBoxes());
    const double pass = 240.0 / solutions.getNumberBoxes();
    Color c;
    for (unsigned int i = 0; i <solutions.getNumberBoxes() ; i++){
        c.setHsv((int)(i*pass),255,255);
        SimpleEigenMesh box;
        SimpleEigenMesh intersection;
        box = solutions.getBox(i).getEigenMesh();
        //double eps = ((double) rand() / (RAND_MAX));
		//box.scale(Vec3d(1+ eps* 1e-5, 1+ eps* 1e-5, 1+ eps* 1e-5));
        //#ifdef BOOL_DEBUG
        //box.saveOnObj("booleans/box" + std::to_string(i) + ".obj");
        //#endif
        #ifdef CG3_USING_LIBIGL_CSGTREE
        libigl::intersection(intersection, tree, box);
        tree = libigl::difference(tree, box);
        #else
        libigl::intersection(intersection, bc, box);
        bc = libigl::difference(bc, box);
        #endif
        DrawableEigenMesh dimm(intersection);
        if (alternativeColors){
            dimm.setFaceColor(c.redF(), c.greenF(), c.blueF());
            he.addHeightfield(dimm, solutions.getBox(i).getRotatedTarget(), i, false);
        }
        else
            he.addHeightfield(dimm, solutions.getBox(i).getRotatedTarget(), i, true);
		std::cerr << i << ": " << solutions[i].getId() << "\n";
    }
    timer.stopAndPrint();
    #ifdef CG3_USING_LIBIGL_CSGTREE
    bc = libigl::CSGTreeToEigenMesh(tree);
    #endif
    for (int i = he.getNumHeightfields()-1; i >= 0 ; i--) {
        if (he.getNumberVerticesHeightfield(i) == 0) {
            he.removeHeightfield(i);
            solutions.removeBox(i);
        }
    }
}

void Engine::splitConnectedComponents(HeightfieldsList& he, BoxList& solutions, std::map<unsigned int, unsigned int> &mapping) {
	int lastId = solutions[0].getId();
    for (unsigned int i = 1; i < solutions.getNumberBoxes(); i++){
		if (solutions[i].getId() > lastId)
			lastId = solutions[i].getId();
    }
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        EigenMesh m = he.getHeightfield(i);
        std::vector<SimpleEigenMesh> cc;
		cc = libigl::connectedComponents(m);
        if (cc.size() > 1){
            ///
			std::cerr << "Split: " << solutions[i].getId() << "; Number: " << cc.size() << "\n";
            ///
            Box3D box = solutions.getBox(i);
			Vec3d target = he.getTarget(i);
            EigenMesh em(cc[0]);
			em.setFaceColor(m.faceColor(0));
            he.setHeightfield(em, i, false);
            for (unsigned int j = 1; j < cc.size(); j++){
                em = EigenMesh(cc[j]);
				em.setFaceColor(m.faceColor(0));
                he.insertHeightfield(em, target, i+j, false);
                solutions.insert(box, i+j);
                solutions[i+j].setId(lastId+1);
				unsigned int tmp = mapping[solutions[i].getId()];
                bool cont = true;
                do {
                    mapping[lastId+1] = tmp;
                    if (mapping[tmp] == tmp)
                        cont = false;
                    else {
                        tmp = mapping[tmp];
                    }
                } while (cont);
                lastId++;
            }
            i = i+cc.size()-1;
        }
    }
}

void Engine::glueInternHeightfieldsToBaseComplex(HeightfieldsList& he, BoxList& solutions, SimpleEigenMesh& bc, const Dcel& inputMesh) {
	cgal::AABBTree3 aabb(inputMesh, true);
    for (int i = (int)he.getNumHeightfields()-1; i >= 0; i--){
        EigenMesh m = he.getHeightfield(i);
        bool inside = true;
		for (unsigned int j = 0; j < m.numberVertices() && inside; j++){
			if (aabb.squaredDistance(m.vertex(j)) < CG3_EPSILON)
                inside = false;
        }
        if (inside){
            libigl::union_(bc, bc, m);
            he.removeHeightfield(i);
            solutions.removeBox(i);
        }
    }
}

void Engine::reduceHeightfields(HeightfieldsList& he, SimpleEigenMesh& bc, const Dcel& inputMesh) {
	cgal::AABBTree3 aabb(inputMesh, true);
    for (int i = he.getNumHeightfields()-1; i >= 0; i--){
		BoundingBox3 realBoundingBox;
        bool first = true;
        for (unsigned int j = 0; j < he.getNumberVerticesHeightfield(i); j++){
			Point3d p = he.getVertexOfHeightfield(i,j);
			if (aabb.squaredDistance(p) < CG3_EPSILON){
                if (first){
                    first = false;
                    realBoundingBox.min() = p;
                    realBoundingBox.max() = p;
                }
                else {
                    realBoundingBox.min() = realBoundingBox.min().min(p);
                    realBoundingBox.max() = realBoundingBox.max().max(p);
                }
            }
        }
        /*for (unsigned int t = 0; t < 6; t++){
            if (he.getTarget(i) == XYZ[t]){
                if (t < 3)
                    realBoundingBox[t] = std::max(realBoundingBox[t] - 1, he.getHeightfield(i).getBoundingBox()[t]);
                else
                    realBoundingBox[t] = std::min(realBoundingBox[t] + 1, he.getHeightfield(i).getBoundingBox()[t]);
            }
        }*/


		if (! epsilonEqual(realBoundingBox.min(), he.getHeightfield(i).boundingBox().min()) ||
			! epsilonEqual(realBoundingBox.max(), he.getHeightfield(i).boundingBox().max()) ){
            SimpleEigenMesh box = EigenMeshAlgorithms::makeBox(realBoundingBox);
            SimpleEigenMesh oldHeightfield = he.getHeightfield(i);
            SimpleEigenMesh gluePortion = libigl::difference(oldHeightfield, box);
            SimpleEigenMesh newHeightfield = libigl::intersection(oldHeightfield, box);
            libigl::union_(bc, bc, gluePortion);
            he.setHeightfield(newHeightfield,i,true);
        }
    }
}

void Engine::gluePortionsToBaseComplex(HeightfieldsList& he, SimpleEigenMesh& bc, BoxList& solutions, const Dcel& inputMesh) {
	cgal::AABBTree3 aabb(inputMesh, true);
    for (unsigned int i = solutions.getNumberBoxes()-1; i >= 1; i--){
        SimpleEigenMesh heightfield = he.getHeightfield(i);
		std::vector<Point3d> pointsOnSurface;
		for (unsigned int j = 0; j < heightfield.numberVertices(); j++){
			Point3d p = heightfield.vertex(j);
			if (aabb.squaredDistance(p) < CG3_EPSILON) pointsOnSurface.push_back(p);
        }
        Eigen::Matrix3d m = solutions.getBox(i).getRotationMatrix(), mt;
        Eigen::Matrix3d arr[4];
        arr[0] = Eigen::Matrix3d::Identity();
		cg3::rotationMatrix(Vec3d(0,0,-1), M_PI/4, arr[1]);
		cg3::rotationMatrix(Vec3d(0,-1,0), M_PI/4, arr[2]);
		cg3::rotationMatrix(Vec3d(-1,0,0), M_PI/4, arr[3]);
        if (m == arr[0])
            mt = arr[0];
        else if (m == arr[1])
			cg3::rotationMatrix(Vec3d(0,0,1), M_PI/4, mt);
        else if (m == arr[2])
			cg3::rotationMatrix(Vec3d(0,1,0), M_PI/4, mt);
        else if (m == arr[3])
			cg3::rotationMatrix(Vec3d(1,0,0), M_PI/4, mt);
        else assert(0);
        pointsOnSurface[0].rotate(mt);
		Point3d min = pointsOnSurface[0], max = pointsOnSurface[0];
        for (unsigned int j = 1; j < pointsOnSurface.size(); j++){
            pointsOnSurface[j].rotate(mt);
            min = min.min(pointsOnSurface[j]);
            max = max.max(pointsOnSurface[j]);
        }
		Vec3d target = solutions.getBox(i).getTarget();
        Box3D b;
        if (target == XYZ[0] || target == XYZ[3]){
            b = solutions.getBox(i);
            b.setMinY(min.y());
            b.setMinZ(min.z());
            b.setMaxY(max.y());
            b.setMaxZ(max.z());
			b.setMinX(b.minX()-5*CG3_EPSILON);
			b.setMaxX(b.maxX()+5*CG3_EPSILON);
        }
        else if (target == XYZ[1] || target == XYZ[4]) {
            b = solutions.getBox(i);
            b.setMinX(min.x());
            b.setMinZ(min.z());
            b.setMaxX(max.x());
            b.setMaxZ(max.z());
			b.setMinY(b.minY()-5*CG3_EPSILON);
			b.setMaxY(b.maxY()+5*CG3_EPSILON);
        }
        else if (target == XYZ[2] || target == XYZ[5]) {
            b = solutions.getBox(i);
            b.setMinY(min.y());
            b.setMinX(min.x());
            b.setMaxY(max.y());
            b.setMaxX(max.x());
			b.setMinZ(b.minZ()-5*CG3_EPSILON);
			b.setMaxZ(b.maxZ()+5*CG3_EPSILON);
        }
        else assert(0);
        SimpleEigenMesh inters;
        SimpleEigenMesh diff;
        SimpleEigenMesh box;
        //solutions.setBox(i,b);
        box = b.getEigenMesh();
        libigl::intersection(inters, heightfield, box);
        libigl::difference(diff, heightfield, box);
        libigl::union_(bc, bc, diff);
        he.addHeightfield(DrawableEigenMesh(inters), solutions.getBox(i).getRotatedTarget(),  i);
        std::cerr << i << "\n";
    }

}

std::set<unsigned int> chartExpansion(const EigenMesh &hf, unsigned int f, std::vector<bool> &seen) {
    std::stack<unsigned int> stack;
    std::set<unsigned int> chart;
	Vec3d nf = hf.faceNormal(f);
	Eigen::MatrixXi fadj;
	libigl::faceToFaceAdjacencies(hf, fadj);
    stack.push(f);
    do {
        f = stack.top();
        stack.pop();
        if (!seen[f]){
            seen[f] = true;
            chart.insert(f);
            for (unsigned int ia = 0; ia < 3; ia++){
                int adj = fadj(f,ia);
				if (adj >= 0 && hf.faceNormal(adj) == nf){
                    stack.push(adj);
                }
            }
        }

    } while (stack.size() > 0);
    return chart;
}

std::vector<Point3d> getPolygonFromChartMarker(const EigenMesh&hf, const cgal::AABBTree3& tree, const std::set<unsigned int>& chart){
    std::vector<std::pair<unsigned int, unsigned int> > segments;
	Eigen::MatrixXi fadj;
	libigl::faceToFaceAdjacencies(hf, fadj);
    for (unsigned int f : chart){
        for (unsigned int ia = 0; ia <3; ia++){
            int adj = fadj(f,ia);
            if (adj != -1 && chart.find(adj) == chart.end()){
				std::pair<int, int> p = hf.commonVertices(f, (unsigned int)adj);
                assert(p.first >= 0 && p.second >= 0);
                segments.push_back(std::pair<unsigned int, unsigned int>(p.first, p.second));
            }
        }
    }

	std::vector<Point3d> polygon;
    unsigned int first, actual;
    first = segments[0].first;
	polygon.push_back(hf.vertex(first));
    actual = segments[0].second;
    do {
		polygon.push_back(hf.vertex(actual));
        bool found = false;
        for (unsigned int i = 0; i < segments.size() && !found; i++){
            if (actual == segments[i].first){
                actual = segments[i].second;
                found = true;
            }
        }
        assert(found);

    } while (actual != first);

    for (int i = polygon.size()-1; i >= 0; i--){
		if (tree.squaredDistance(polygon[i]) != 0){
            polygon.erase(polygon.begin() + i);
        }
    }

    return polygon;
}

SimpleEigenMesh Engine::getMarkerMesh(const HeightfieldsList& he, const Dcel &d) {

    /*SimpleEigenMesh marked;
	CGALInterface::AABBTree3 tree(d, true);

    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        SimpleEigenMesh hf = he.getHeightfield(i);
        EigenMeshAlgorithms::removeDuplicateVertices(hf);

		std::vector<bool> seen(hf.numberFaces(), false);
        std::vector< std::set<unsigned int> >charts;

		for (unsigned int f = 0; f < hf.numberFaces(); f++) {
			Vec3d nf = hf.faceNormal(f);
            int idf = indexOfNormal(nf);
            if (idf != -1){
                if (!seen[f])
                    charts.push_back(chartExpansion(hf, f, seen));
            }
        }

        for (std::set<unsigned int> &chart: charts){
            //polygons
			std::vector<Point3d> pol = getPolygonFromChartMarker(hf, tree, chart);
            marked.addVertex(pol[0]);
            unsigned int last = marked.getNumberVertices()-1;
            unsigned int first = last;
            for (unsigned int i = 1; i < pol.size(); i++){
				Point3d p1 = pol[i-1];
				Point3d p2 = pol[i];
				Point3d m = (p1+p2)/2;
                if (tree.getSquaredDistance(m) < 0.5){
                    marked.addVertex(m);
                    marked.addVertex(p2);
                    unsigned int mid = last+1;
                    unsigned int actual = last+2;
                    marked.addFace(last, actual, mid);
                    last = actual;
                }
                else {
                    marked.addVertex(p2);
                    last++;
                }
            }
			Point3d m = (pol[0] + pol[pol.size()-1]) / 2;
            if (tree.getSquaredDistance(m) < 0.5){
                marked.addVertex(m);
                unsigned int mid = marked.getNumberVertices()-1;
                marked.addFace(last, first, mid);
            }

        }
    }

    return marked;*/


	cgal::AABBTree3 tree(d, true);
	std::set< std::pair<Point3d, Point3d> > edges;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const EigenMesh& mesh = he.getHeightfield(i);
		Eigen::MatrixXi TT;
		libigl::faceToFaceAdjacencies(mesh, TT);
		for (unsigned int f = 0; f < mesh.numberFaces(); f++){
			Vec3d n1 = mesh.faceNormal(f);
            if (n1.dot(he.getTarget(i))<=CG3_EPSILON){
                for (unsigned int k = 0; k < 3; k++){
                    int adj = TT(f,k);
                    if (adj >= 0){
						Vec3d n2 = mesh.faceNormal(adj);
                        if (n2.dot(he.getTarget(i))>=-CG3_EPSILON){
							Point3i f1 = mesh.face(f);
							Point3i f2 = mesh.face(adj);
							std::set<Point3d> allPoints;
                            for (unsigned int i = 0; i < 3; i++){
								allPoints.insert(mesh.vertex(f1[i]));
								allPoints.insert((mesh.vertex(f1[i]) + mesh.vertex(f1[(i+1)%3]))/2);
                            }
                            for (unsigned int i = 0; i < 3; i++){
								allPoints.insert(mesh.vertex(f2[i]));
								allPoints.insert((mesh.vertex(f2[i]) + mesh.vertex(f2[(i+1)%3]))/2);
                            }
                            bool allNear = true;
                            bool allDist = true;
							for (Point3d  p : allPoints){
								if (tree.squaredDistance(p) > CG3_EPSILON)
                                    allNear = false;
                                else
                                    allDist = false;
                            }
                            if (!allNear && !allDist){
                                int v1 = -1, v2 = -1;
                                bool finded = false;
                                unsigned int t1, tmp = -1;
                                for (t1 = 0; t1 < 3 && !finded; t1++){
                                    for (unsigned int t2 = 0; t2 < 3  && !finded; t2++){
                                        if (f1[t1] == f2[t2]){
                                            v1 = f1[t1];
                                            tmp = t1;
                                            finded = true;
                                        }
                                    }
                                }
                                assert(finded && tmp < 2);
                                finded = false;
                                for (t1 = tmp+1; t1 < 3 && !finded; t1++){
                                    for (unsigned int t2 = 0; t2 < 3  && !finded; t2++){
                                        if (f1[t1] == f2[t2]){
                                            v2 = f1[t1];
                                            finded = true;
                                        }
                                    }
                                }
                                assert(finded && v1 != v2);
								Point3d p1 = mesh.vertex(v1);
								Point3d p2 = mesh.vertex(v2);
								std::pair<Point3d, Point3d> edge;
                                if (p1 < p2){
                                    edge.first = p1;
                                    edge.second = p2;
                                }
                                else {
                                    edge.first = p2;
                                    edge.second = p1;
                                }
                                edges.insert(edge);
                            }
                        }
                    }
                }
            }
        }

    }
    SimpleEigenMesh marked;
	std::map<Point3d, int> mapVertices;
    int n = 0;
	for (std::pair<Point3d, Point3d> edge : edges){
        int v1;
        if (mapVertices.find(edge.first) == mapVertices.end()){
            v1 = n;
            mapVertices[edge.first] = n;
            marked.addVertex(edge.first);
            n++;
        }
        else {
            v1 = mapVertices[edge.first];
        }
        int v2;
        if (mapVertices.find(edge.second) == mapVertices.end()){
            v2 = n;
            mapVertices[edge.second] = n;
            marked.addVertex(edge.second);
            n++;
        }
        else {
            v2 = mapVertices[edge.second];
        }
        int v3;
		Point3d midPoint = (edge.first + edge.second) / 2.0;
        if (mapVertices.find(midPoint) == mapVertices.end()){
            v3 = n;
            mapVertices[midPoint] = n;
            marked.addVertex(midPoint);
            n++;
        }
        else {
            v3 = mapVertices[midPoint];
        }
        marked.addFace(v1, v2, v3);
    }
    return marked;
}

std::vector<Point3d> getPolygonFromChart(const EigenMesh&hf, const std::set<unsigned int>& chart){
    std::vector<std::pair<unsigned int, unsigned int> > segments;
	Eigen::MatrixXi fadj;
	libigl::faceToFaceAdjacencies(hf, fadj);
    for (unsigned int f : chart){
        for (unsigned int ia = 0; ia <3; ia++){
            int adj = fadj(f,ia);
            if (adj != -1 && chart.find(adj) == chart.end()){
				std::pair<int, int> p = hf.commonVertices(f, (unsigned int)adj);
                assert(p.first >= 0 && p.second >= 0);
                segments.push_back(std::pair<unsigned int, unsigned int>(p.first, p.second));
            }
        }
    }

	std::vector<Point3d> polygon;
    unsigned int first, actual;
    first = segments[0].first;
	polygon.push_back(hf.vertex(first));
    actual = segments[0].second;
    do {
		polygon.push_back(hf.vertex(actual));
        bool found = false;
        for (unsigned int i = 0; i < segments.size() && !found; i++){
            if (actual == segments[i].first){
                actual = segments[i].second;
                found = true;
            }
        }
        assert(found);

    } while (actual != first);


    return polygon;
}

void Engine::saveObjs(const std::string& foldername, const EigenMesh &originalMesh, const Dcel& inputMesh, const EigenMesh &baseComplex, const HeightfieldsList &he) {
    std::string originalMeshString = foldername + "/OriginalMesh.obj";
    std::string inputMeshString = foldername + "/InputMesh.obj";
    std::string baseComplexString = foldername + "/BaseComplex.obj";
    std::string heightfieldString = foldername + "/Heightfield";
    //std::string markerString = foldername + "/Marker.obj";
    std::string structureString = foldername + "/Structure.obj";
	//CGALInterface::AABBTree3 tree(inputMesh);
	if (originalMesh.numberVertices() > 0)
        originalMesh.saveOnObj(originalMeshString);
	inputMesh.saveOnObj(inputMeshString);
    baseComplex.saveOnObj(baseComplexString);
    //SimpleEigenMesh marker = getMarkerMesh(he, inputMesh);
    //marker.saveOnObj(markerString);
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        EigenMesh h = he.getHeightfield(i);
        Dcel d(h);
        //updatePieceNormals(tree, d);
		d.saveOnObj(heightfieldString + std::to_string(i) + ".obj");
    }

    /*if (baseComplex.getNumberVertices() > 0) {

        double diameter = inputMesh.getAverageHalfEdgesLength();
        EigenMesh structure;
        EigenMesh cbc(baseComplex);
        libigl::removeDuplicateVertices(cbc);
		std::vector<bool> seen(cbc.numberFaces(), false);
        std::vector< std::set<unsigned int> >charts;

		for (unsigned int f = 0; f < cbc.numberFaces(); f++) {
			Vec3d nf = cbc.faceNormal(f);
            int idf = indexOfNormal(nf);
            if (idf != -1){
                if (!seen[f])
                    charts.push_back(chartExpansion(cbc, f, seen));
            }
        }

		std::set<Point3d> spheres;
        for (std::set<unsigned int> &chart: charts){
            //polygons
			std::vector<Point3d> polygon = getPolygonFromChart(cbc, chart);

            for (unsigned int j = 0; j < polygon.size(); j++){
				Vec3d n1 = polygon[(j+1)%polygon.size()]-polygon[j];
                n1.normalize();
				Vec3d n2 = polygon[(j+2)%polygon.size()]-polygon[(j+1)%polygon.size()];
                n2.normalize();
                if (epsilonEqual(n1, n2)){
                    polygon.erase(polygon.begin()+((j+1)%polygon.size()));
                    j--;
                }
            }

            for (unsigned int i = 0; i < polygon.size(); i++){
                if (spheres.find(polygon[i]) == spheres.end()){
                    EigenMesh sphr = EigenMeshAlgorithms::makeSphere(polygon[i], diameter);
                    sphr.setFaceColor(Color(255,255,255));
                    structure = EigenMesh::merge(structure, sphr);
                    spheres.insert(polygon[i]);
                }
                EigenMesh cyl = EigenMeshAlgorithms::makeCylinder(polygon[i], polygon[(i+1)%polygon.size()], diameter/2);
                cyl.setFaceColor(Color(0,0,0));
                structure = EigenMesh::merge(structure, cyl);
            }
        }
        structure.saveOnObj(structureString);
    }*/
}

void Engine::updatePieceNormals(const cgal::AABBTree3& tree, Dcel& piece) {
    for (Dcel::Vertex* v : piece.vertexIterator()){
		if (tree.squaredDistance(v->coordinate() < CG3_EPSILON)){
			const Dcel::Vertex* n = tree.nearestDcelVertex(v->coordinate());
			v->setNormal(n->normal());
        }
    }
}

void Engine::updatePieceNormals(const cgal::AABBTree3& tree, EigenMesh& piece) {
    Dcel d(piece);
    for (Dcel::Vertex* v : d.vertexIterator()){
		if (tree.squaredDistance(v->coordinate() < CG3_EPSILON)){
			const Dcel::Vertex* n = tree.nearestDcelVertex(v->coordinate());
			if (n->coordinate().dist(v->coordinate()) < 3)
				v->setNormal(n->normal());
        }
    }
    piece = EigenMesh(d);
}

void Engine::updatePiecesNormals(const cgal::AABBTree3& tree, HeightfieldsList& he) {
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        updatePieceNormals(tree, he.getHeightfield(i));
    }
}

bool Engine::isAnHeightfield(const EigenMesh& m, const Vec3d& v, bool strictly) {
    bool first = true;
    bool heightfield = true;
    double baseCoord = -1;
    int dir = -1;
    if (v == XYZ[0] || v == XYZ[3])
        dir = 0;
    else if (v == XYZ[1] || v == XYZ[4])
        dir = 1;
    else if (v == XYZ[2] || v == XYZ[5])
        dir = 2;
    else
        assert(0);
	for (unsigned int i = 0; i < m.numberFaces() && heightfield; i++){
		Vec3d normal = m.faceNormal(i);
        if (epsilonEqual(normal, -v)){
			Point3i face = m.face(i);
			Point3d p0 = m.vertex(face[0]);
			Point3d p1 = m.vertex(face[1]);
			Point3d p2 = m.vertex(face[2]);
            if (first){
                first = false;
                baseCoord = p0[dir];
            }
            else {
                if (! epsilonEqual(p0[dir],baseCoord))
                    heightfield = false;
            }
            if (! epsilonEqual(p1[dir],baseCoord))
                heightfield = false;
            if (! epsilonEqual(p2[dir],baseCoord))
                heightfield = false;
        }
        else if (strictly) {
            if (normal.dot(v) < 0)
                heightfield = false;
        }
    }
    return heightfield;
}

void Engine::colorPieces(const Dcel& d, HeightfieldsList& he) {
	cgal::AABBTree3 tree(d);
    constexpr int nColors = 9;
    std::array<QColor, nColors> colors;
    colors[0] = QColor(221, 126, 107); //
    colors[1] = QColor(255, 229, 153); //
    colors[2] = QColor(164, 194, 244); //
    colors[3] = QColor(213, 166, 189); //
    colors[4] = QColor(234, 153, 153); //
    colors[5] = QColor(182, 215, 168); //
    //colors[6] = QColor(217, 234, 211);//QColor(159, 197, 232); //
    colors[6] = QColor(249, 203, 156);//
    //colors[8] = QColor(207, 216, 233);//QColor(162, 196, 201);
    colors[7] = QColor(180, 167, 214);//
    colors[8] = QColor(162, 196, 201);

    /*colors[0] = QColor(182, 215, 168); //
    colors[1] = QColor(198, 178, 219); //
    colors[2] = QColor(234, 153, 153); //
    colors[3] = QColor(255, 229, 153); //
    colors[4] = QColor(162, 196, 201); //
    colors[5] = QColor(213, 166, 189); //
    colors[6] = QColor(164, 194, 244); //
    colors[7] = QColor(221, 126, 107);//
    colors[8] = QColor(249, 203, 156);//
    colors[9] = QColor(180, 167, 214);//*/


    std::map< const Dcel::Vertex*, int > mapping = Reconstruction::getMappingId(d, he);
    std::vector< std::set<int> > adjacences(he.getNumHeightfields());
    for (const Dcel::Vertex* v : d.vertexIterator()){
        if (mapping.find(v) != mapping.end()){
            int hev = mapping[v];
            for (const Dcel::Vertex* adj : v->adjacentVertexIterator()){
                if (mapping.find(adj) != mapping.end()){
                    int headj = mapping[adj];
                    if (hev != headj){
                        adjacences[hev].insert(headj);
                        adjacences[headj].insert(hev);
                    }
                }
            }
        }
    }

    std::vector<bool> colored(he.getNumHeightfields(), false);
    std::vector<Color> heColors(he.getNumHeightfields(), Color(0,0,0));
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        if (!colored[i]){
            std::set<Color> adjColors;
            for (int adj : adjacences[i]){
                if (colored[adj])
                    adjColors.insert(heColors[adj]);
            }

            Color color;
            bool finded = false;
            unsigned int k = i % nColors;
            do {
                if (adjColors.find(colors[k]) == adjColors.end()){
                    finded = true;
                    color = colors[k];
                }
                k = (k+1)%nColors;
            } while(k != i %nColors && !finded);

            if (finded)
                heColors[i] = color;
            else
                heColors[i] = Color(0,0,0);
            colored[i] = true;


            EigenMesh mesh = he.getHeightfield(i);
            Dcel dd(mesh);
            Engine::updatePieceNormals(tree, dd);
            mesh = EigenMesh(dd);
            mesh.setFaceColor(color.redF(),color.greenF(),color.blueF());
            he.setHeightfield(mesh, i);
        }
    }
}

std::vector<Point3d> getBasePolygon(const EigenMesh &m, const Vec3d& target){
	std::vector<bool> seen(m.numberFaces(), false);
    std::vector< std::set<unsigned int> >charts;

	for (unsigned int f = 0; f < m.numberFaces(); f++) {
		Vec3d nf = m.faceNormal(f);
        if (nf == -target){
            if (!seen[f])
                    charts.push_back(chartExpansion(m, f, seen));
        }
    }

    if (charts.size() > 1 || charts.size() == 0)
        std::cerr << "Maybe there is a problem with the heightfield..\n";

	std::vector<Point3d> polygon;
    for (std::set<unsigned int> &chart: charts){

        //polygons
		std::vector<Point3d> p = getPolygonFromChart(m, chart);
        polygon.insert(polygon.end(), p.begin(), p.end());
    }

    return polygon;

}

void bringBaseDown(EigenMesh& m, const Vec3d& target, double oldbase, double newbase){
	std::vector<Point3d> base = getBasePolygon(m, target);
    std::vector<unsigned int> baseIndices;
    std::set<unsigned int> baseIndicesSet;
    std::vector<unsigned int> newBaseIndices;
    baseIndices.resize(base.size());
	unsigned int numberVerticesBeforeInsertion = m.numberVertices();
    for (unsigned int i = 0; i < numberVerticesBeforeInsertion; i++){
		std::vector<Point3d>::iterator it = std::find(base.begin(), base.end(), m.vertex(i));
        if (it != base.end()){
            baseIndices[it - base.begin()] = i;
            baseIndicesSet.insert(i);
        }
    }
    unsigned int numberVerticesAfterInsertion = numberVerticesBeforeInsertion;
	for (Point3d p : base){
        m.addVertex(p);
        newBaseIndices.push_back(numberVerticesAfterInsertion++);
    }

	for (unsigned int f = 0; f < m.numberFaces(); f++){
		if (m.faceNormal(f) != -target){
			Point3i vf = m.face(f);
			Point3i nvf;
            for (unsigned int k = 0; k < 3; k++){
                if (baseIndicesSet.find(vf[k]) != baseIndicesSet.end()){
                    std::vector<unsigned int>::iterator it = std::find(baseIndices.begin(), baseIndices.end(), vf[k]);
                    assert(it != baseIndices.end());
                    nvf[k] = newBaseIndices[it - baseIndices.begin()];
                }
                else
                    nvf[k] = vf[k];
            }
            m.setFace(f, nvf.x(), nvf.y(), nvf.z());
        }
    }

    for (unsigned int i = 0; i < numberVerticesBeforeInsertion; i++){
		if (m.vertex(i)[indexOfNormal(target)%3] == oldbase){
			Point3d p = m.vertex(i);
            p[indexOfNormal(target)%3] = newbase;
            m.setVertex(i, p);
        }
    }

    for (unsigned int i = 0; i < baseIndices.size(); i++){
		Point3i f1(newBaseIndices[i], baseIndices[i], newBaseIndices[(i+1)%baseIndices.size()]);
		Point3i f2(newBaseIndices[(i+1)%baseIndices.size()], baseIndices[i], baseIndices[(i+1)%baseIndices.size()]);
		m.addFace(f1.x(), f1.z(), f1.y());
        m.addFace(f2.x(), f2.z(), f2.y());
    }
}


void Engine::mergePostProcessing(HeightfieldsList &he, BoxList &solutions, EigenMesh& baseComplex, const Dcel& d, bool mergeDownwards){

    //first merging
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
		cgal::AABBTree3 tree(he.getHeightfield(i), true);
        for (unsigned int j = 0; j < he.getNumHeightfields(); j++){
            if ( i != j){
                //can I merge j with i?
                if (he.getTarget(i) == he.getTarget(j)){
                    //take all the points of the base of j
                    EigenMesh m = he.getHeightfield(j);
                    libigl::removeDuplicateVertices(m);
					std::vector<Point3d> base = getBasePolygon(m, he.getTarget(j));

                    //and check the distance between every point of the base of j and the surface of i
                    //if all distances are 0, blocks can be merged
                    bool same = true;
                    for (unsigned int i = 0; i < base.size() && same; i++){
						double dist = tree.squaredDistance(base[i]);
                        if (dist > 1e-05)
                            same = false;
                    }
                    if (same && base.size() != 0){ //heightfield j can be merged to heightfield i
                        EigenMesh un = libigl::union_(he.getHeightfield(i), he.getHeightfield(j));
						un.setFaceColor(he.getHeightfield(i).faceColor(0));
                        he.setHeightfield(un, i);
                        he.removeHeightfield(j);
						solutions.changeBoxLimits(un.boundingBox(), i);
                        solutions.removeBox(j);
                        if (i > j)
                            i--;
                        j--;

                    }
                }
            }
        }
    }

    //second merging
	cgal::AABBTree3 tree(d);

    //building adjacences
    std::map< const Dcel::Vertex*, int > mapping = Reconstruction::getMappingId(d, he);
    std::vector< std::set<int> > adjacences(he.getNumHeightfields());
    for (const Dcel::Vertex* v : d.vertexIterator()){
        if (mapping.find(v) != mapping.end()){
            int hev = mapping[v];
            for (const Dcel::Vertex* adj : v->adjacentVertexIterator()){
                if (mapping.find(adj) != mapping.end()){
                    int headj = mapping[adj];
                    if (hev != headj){
                        adjacences[hev].insert(headj);
                        adjacences[headj].insert(hev);
                    }
                }
            }
        }
    }

    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        bool br = false;
        for (std::set<int>::iterator jit = adjacences[i].begin(); jit != adjacences[i].end() && !br; ++jit){
            unsigned int j = *jit;
            if (he.getTarget(i) == he.getTarget(j)){ //two adjacent blocks with same hf direction
				BoundingBox3 bbi = he.getHeightfield(i).boundingBox();
				BoundingBox3 bbj = he.getHeightfield(j).boundingBox();


                int iohf = indexOfNormal(he.getTarget(i));
                double basei, basej;
                bool isPossibleReduce = true;
                if (iohf < 3){
                    basei = bbi.min()[iohf];
                    basej = bbj.min()[iohf];
                    if (basej > basei)
                        isPossibleReduce = false;
                }
                else{
                    basei = bbi.max()[iohf-3];
                    basej = bbj.max()[iohf-3];
                    if (basej < basei)
                        isPossibleReduce = false;
                }

                //can j reduce to i?
                bool merged = false;
                if (isPossibleReduce){
					std::vector<Point3d> points;
					points.reserve(he.getHeightfield(j).numberVertices());
					for (unsigned int i = 0; i < he.getHeightfield(j).numberVertices(); i++){
						points.push_back(he.getHeightfield(j).vertex(i));
                    }

                    //sort points using iohf%3
                    struct cmp {
                        unsigned int i;
                        cmp(unsigned int i):i(i){}

						bool operator()(const Point3d& p1, const Point3d& p2){
                            if (p1[i] < p2[i])
                                return true;
                            if (p1[i] == p2[i])
                                return p1 < p2;
                            return false;

                        }
                    };

                    std::sort(points.begin(), points.end(), cmp(iohf%3));

                    //check if look the points forward or backward according to iohf<3
					std::vector<Point3d> pointsToDelete;
                    if (iohf < 3){
                        bool less = true;
                        for (unsigned int i = 0; i < points.size() && less; i++){
                            if (points[i][iohf%3] < basei)
                                pointsToDelete.push_back(points[i]);
                            else
                                less = false;
                        }
                    }
                    else {
                        bool less = true;
                        for (int i = points.size()-1; i >=0  && less; i--){
                            if (points[i][iohf%3] > basei)
                                pointsToDelete.push_back(points[i]);
                            else
                                less = false;
                        }
                    }
                    merged = true;
                    for (unsigned int i = 0; i < pointsToDelete.size() && merged; i++){
						if (tree.squaredDistance(pointsToDelete[i]) < 1e-4)
                            merged = false;
                    }
                    if (merged){
                        //no points of j touch the surface, we can cut that piece.
                        //he.getHeightfield(j).setFaceColor(Color(0,255,0));
                        if (iohf < 3)
                            bbj.max()[iohf] = basei;
                        else
                            bbj.min()[iohf-3] = basei;
						Color ci = he.getHeightfield(i).faceColor(0);
                        EigenMesh res = libigl::difference(he.getHeightfield(j), EigenMesh(EigenMeshAlgorithms::makeBox(bbj)));
                        res = libigl::union_(res, he.getHeightfield(i));
                        baseComplex = libigl::union_(baseComplex, libigl::intersection(he.getHeightfield(j), EigenMesh(EigenMeshAlgorithms::makeBox(bbj))));
                        res.setFaceColor(ci);

                        he.setHeightfield(res, i);
                        he.removeHeightfield(j);
						solutions.changeBoxLimits(res.boundingBox(), i);
                        solutions.removeBox(j);
                    }
                }
                ///
                else if (mergeDownwards){
					cgal::AABBTree3 treebc(baseComplex);
					cgal::AABBTree3 treei(he.getHeightfield(i));
					std::set<Point3d> basePoints;
					for (unsigned int pj = 0; pj < he.getHeightfield(j).numberVertices(); pj++){
						if (he.getHeightfield(j).vertex(pj)[iohf%3] == basej){
							basePoints.insert(he.getHeightfield(j).vertex(pj));
							//mw->addDebugSphere(he.getHeightfield(j).vertex(pj), 2, QColor(255,0,0));
                        }
                    }

                    merged = true;
					for (Point3d p : basePoints){
                        p[iohf%3] = basei;
						if (!(treebc.squaredDistance(p) == 0 || treei.squaredDistance(p) == 0 || treebc.isInside(p) || treei.isInside(p)))
                            merged = false;
                    }
                    if (merged) {
                        /*for (unsigned int pj = 0; pj < he.getHeightfield(j).getNumberVertices(); pj++){
							if (he.getHeightfield(j).vertex(pj)[iohf%3] == basej){
								Point3d p = he.getHeightfield(j).vertex(pj);
                                p[iohf%3] = basei;
                                he.getHeightfield(j).setVertex(pj, p);
                            }
                        }*/
                        bringBaseDown(he.getHeightfield(j), he.getTarget(j), basej, basei);
                        he.getHeightfield(j).updateBoundingBox();

						Color ci = he.getHeightfield(i).faceColor(0);
                        EigenMesh res = libigl::union_(he.getHeightfield(i), he.getHeightfield(j));
                        baseComplex = libigl::difference(baseComplex, he.getHeightfield(i));
                        res.setFaceColor(ci);

                        he.setHeightfield(res, i);
                        he.removeHeightfield(j);
						solutions.changeBoxLimits(res.boundingBox(), i);
                        solutions.removeBox(j);
                    }
                }
                ///
                if (merged){
                    std::set<int> tmp = adjacences[j];
                    tmp.erase(i);
                    if (i > j)
                        i--;

                    //update adjacences
                    std::set<int> ntmp;
                    for (int a : tmp){
                        ntmp.insert((unsigned int)a > j ? a-1 : a);
                    }

                    adjacences.erase(adjacences.begin()+j);
                    for (unsigned int k = 0; k < adjacences.size(); k++){
                        const std::set<int>& adk = adjacences[k];
                        std::set<int> nadk;
                        for (int a : adk){
                            if ((unsigned int)a > j) // j is removed, ids higher than j must be decremented
                                nadk.insert(a-1);
                            else if ((unsigned int)a == j && k != i) // j is merged to i, everithing that was adjacent to j now is adjacent to i
                                nadk.insert(i);
                            else if ((unsigned int)a != j)
                                nadk.insert(a);

                        }
                        if (k == i){
                            nadk = cg3::union_(nadk, ntmp);
                        }
                        adjacences[k] = nadk;
                    }
                    br = true;
                }
            }
        }
        if (br)
            i--;
    }
}
