#include "engine.h"

#ifdef GUROBI_DEFINED
#include <gurobi_c++.h>
#endif

Vec3 Engine::getClosestTarget(const Vec3& n) {
    double angle = n.dot(XYZ[0]);
    int k = 0;
    for (unsigned int i = 1; i < 6; i++){
        if (n.dot(XYZ[i]) > angle){
            angle = n.dot(XYZ[i]);
            k = i;
        }
    }
    return XYZ[k];
}

void Engine::serializeAsEngineManager(std::ofstream& binaryfile, const Grid& g, const Dcel &d, const BoxList &bl) {
    g.serialize(binaryfile);
    d.serialize(binaryfile);
    bool b = true;
    Serializer::serialize(b, binaryfile);
    bl.serialize(binaryfile);
}

Eigen::Matrix3d Engine::rotateDcelAlreadyScaled(Dcel& d, unsigned int rot) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (rot > 0){
        switch (rot){
            case 1:
                Common::getRotationMatrix(Vec3(0,0,1), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
                Common::getRotationMatrix(Vec3(0,0,-1), M_PI/4, m);
                break;
            case 2:
                Common::getRotationMatrix(Vec3(0,1,0), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
                Common::getRotationMatrix(Vec3(0,-1,0), M_PI/4, m);
                break;
            case 3:
                Common::getRotationMatrix(Vec3(1,0,0), M_PI/4, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                //m.transpose();
                Common::getRotationMatrix(Vec3(-1,0,0), M_PI/4, m);
                break;
            default:
                assert(0);
        }
    }
    return m;
}

Eigen::Matrix3d Engine::scaleAndRotateDcel(Dcel& d, unsigned int rot, double factor) {
    BoundingBox bb = d.getBoundingBox();
    double avg = 0;
    for (Dcel::HalfEdgeIterator heit = d.halfEdgeBegin(); heit != d.halfEdgeEnd(); ++heit){
        avg += (*heit)->getLength();
    }
    avg /= d.getNumberHalfEdges();
    double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
    maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
    int resolution = (maxl / avg + 1)*factor;
    double av = maxl / resolution;
    BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);
    d.scale(nBB);
    return rotateDcelAlreadyScaled(d, rot);
}

void Engine::getFlippedFaces(std::set<const Dcel::Face*> &flippedFaces, std::set<const Dcel::Face*> &savedFaces, const Dcel& d, const Vec3& target, double angleThreshold, double areaThreshold) {
    double dot = - angleThreshold;
    for (const Dcel::Face* f : d.faceIterator()){
        if (f->getNormal().dot(target) < 0){
            if (f->getNormal().dot(target) < dot)
                flippedFaces.insert(f);
            else
                savedFaces.insert(f);
        }
    }
    if (areaThreshold > 0) { //if areathreshold = 0, flipped charts are ignored
        double totalArea = d.getSurfaceArea();
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
                    area += f->getArea();
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

void Engine::generateGridAndDistanceField(Array3D<Pointd> &grid, Array3D<gridreal> &distanceField, const IGLInterface::SimpleIGLMesh &m, double gridUnit, bool integer){
    assert(gridUnit > 0);
    // Bounding Box
    Eigen::RowVector3d Vmin, Vmax;
    m.getBoundingBox(Vmin, Vmax);

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
    Eigen::RowVector3i res = (nGmax.cast<int>() - nGmin.cast<int>())/2; res(0)+=1; res(1)+=1; res(2)+=1;
    std::vector<double> distances;
    Array3D<int> mapping(res(0), res(1), res(2), -1);
    std::vector<Pointd> insidePoints;
    int inside = 0;

    grid.resize(res(0), res(1), res(2));
    distanceField.resize(res(0), res(1), res(2));
    distanceField.setConstant(1);
    CGALInterface::AABBTree tree(m, true);
    Array3D<unsigned char> isInside(res(0), res(1), res(2));
    isInside.setConstant(false);

    int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
    for (int i = 0; i < res(0); ++i){
        yi = nGmin(1);
        for (int j = 0; j < res(1); ++j){
            zi = nGmin(2);
            for (int k = 0; k < res(2); ++k){
                grid(i,j,k) = Pointd(xi,yi,zi);
                zi+=gridUnit;
            }
            yi+=gridUnit;
        }
        xi += gridUnit;
    }

    unsigned int rr =  res(0)*res(1)*res(2);

    #pragma omp parallel for
    for (unsigned int n = 0; n < rr; n++){
        unsigned int k = (n % (res(1)*res(2)))%res(2);
        unsigned int j = ((n-k)/res(2))%res(1);
        unsigned int i = ((n-k)/res(2) - j)/res(1);
        isInside(i,j,k) = tree.isInside(grid(i,j,k), 3);
    }

    for (unsigned int i = 0; i < res(0); i++){
        for (unsigned int j = 0; j < res(1); j++){
            for (unsigned int k = 0; k < res(2); k++){
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



    distances = CGALInterface::SignedDistances::getSignedDistances(insidePoints, tree);

    for (int i = 0; i < res(0); i++){
        for (int j = 0; j < res(1); j++){
            for (int k = 0; k < res(2); k++){
                if (isInside(i,j,k)){
                    assert(mapping(i,j,k) >= 0);
                    distanceField(i,j,k) = -distances[mapping(i,j,k)];
                }
            }
        }
    }
}

void Engine::generateGrid(Grid& g, const Dcel& d, double kernelDistance, bool tolerance, const Vec3 &target, std::set<const Dcel::Face*>& savedFaces) {
    IGLInterface::SimpleIGLMesh m(d);
    Array3D<Pointd> grid;
    Array3D<gridreal> distanceField;
    Timer t("Distance field");
    Engine::generateGridAndDistanceField(grid, distanceField, m);
    t.stopAndPrint();
    Pointi res(grid.getSizeX(), grid.getSizeY(), grid.getSizeZ());
    Pointd nGmin(grid(0,0,0));
    Pointd nGmax(grid(res.x()-1, res.y()-1, res.z()-1));
    g = Grid(res, grid, distanceField, nGmin, nGmax);
    g.setTarget(target);
    g.calculateWeightsAndFreezeKernel(d, kernelDistance, tolerance, savedFaces);
    Energy e(g);
    e.calculateFullBoxValues(g);
}

void Engine::setTrianglesTargets(Dcel scaled[]) {
    for (Dcel::FaceIterator fit = scaled[0].faceBegin(); fit != scaled[0].faceEnd(); ++fit){
        Vec3 n = (*fit)->getNormal();
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
            scaled[1].getFace((*fit)->getId())->setFlag(0);
            scaled[2].getFace((*fit)->getId())->setFlag(0);
            scaled[3].getFace((*fit)->getId())->setFlag(0);
        }
        else if (k < 10) {
            (*fit)->setFlag(1);
            scaled[1].getFace((*fit)->getId())->setFlag(1);
            scaled[2].getFace((*fit)->getId())->setFlag(1);
            scaled[3].getFace((*fit)->getId())->setFlag(1);
        }
        else if (k < 14) {
            (*fit)->setFlag(2);
            scaled[1].getFace((*fit)->getId())->setFlag(2);
            scaled[2].getFace((*fit)->getId())->setFlag(2);
            scaled[3].getFace((*fit)->getId())->setFlag(2);
        }
        else if (k < 18) {
            (*fit)->setFlag(3);
            scaled[1].getFace((*fit)->getId())->setFlag(3);
            scaled[2].getFace((*fit)->getId())->setFlag(3);
            scaled[3].getFace((*fit)->getId())->setFlag(3);
        }
    }
}

void Engine::addBox(BoxList& boxList, const Vec3 target, const Dcel::Face* f, const Eigen::Matrix3d& rot){
    Box3D box;
    box.setTarget(target);
    Pointd p1 = f->getOuterHalfEdge()->getFromVertex()->getCoordinate();
    Pointd p2 = f->getOuterHalfEdge()->getToVertex()->getCoordinate();
    Pointd p3 = f->getOuterHalfEdge()->getNext()->getToVertex()->getCoordinate();
    Pointd bmin = p1;
    bmin = bmin.min(p2);
    bmin = bmin.min(p3);
    bmin = bmin - 1;
    Pointd bmax = p1;
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

void Engine::calculateDecimatedBoxes(BoxList& boxList, const Dcel& d, const Eigen::VectorXi &mapping, const std::set<int>& coveredFaces, const Eigen::Matrix3d& rot, int orientation, bool onlyTarget, const Vec3& target) {
    std::vector<int> facesToCover;
    for (unsigned int i = 0; i < mapping.size(); i++){
        if (coveredFaces.find(mapping(i)) == coveredFaces.end()){
            facesToCover.push_back(mapping(i));
        }
    }

    for (unsigned int i = 0; i < facesToCover.size(); i++){
        const Dcel::Face* f = d.getFace(facesToCover[i]);
        Vec3 n =f->getNormal();
        Vec3 closestTarget = getClosestTarget(n);
        if (!onlyTarget || (onlyTarget && closestTarget == target)){
            if (orientation<0 || f->getFlag()==orientation){
                addBox(boxList, closestTarget, f, rot);
            }
        }
    }
}

void Engine::calculateInitialBoxes(BoxList& boxList, const Dcel& d, const Eigen::Matrix3d &rot, bool onlyTarget, const Vec3& target) {
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        Vec3 n =f->getNormal();
        Vec3 closestTarget = getClosestTarget(n);
        if (!onlyTarget || (onlyTarget && closestTarget == target)){
            addBox(boxList, closestTarget, f, rot);
        }
    }
}

void Engine::expandBoxes(BoxList& boxList, const Grid& g, bool limit, const Pointd& limits, bool printTimes) {
    Energy e(g);
    Timer total("Boxlist expanding");
    int np = boxList.getNumberBoxes();
    Timer t("");
    #pragma omp parallel for
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
            Pointd actualLimits(limits.x(), limits.x(), limits.x());
            bool find = false;
            for (unsigned int i = 0; i < 3 && !find; i++){
                if (XYZ[i] == b.getTarget() || XYZ[i+3] == b.getTarget()){
                    actualLimits[i] = limits.z();
                    find = true;
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
    CGALInterface::AABBTree t(d);


    // creating vector of pairs

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
        t.getIntersectedDcelFaces(covered, b);

        std::list<const Dcel::Face*>::iterator it = covered.begin();
        while (it != covered.end()) {
            const Dcel::Face* f = *it;
            Pointd p1 = f->getVertex1()->getCoordinate(), p2 = f->getVertex2()->getCoordinate(), p3 = f->getVertex3()->getCoordinate();

            if (!b.isIntern(p1) || !b.isIntern(p2) || !b.isIntern(p3)) {
                it =covered.erase(it);
            }
            else ++it;
        }

        std::vector<bool> v;
        int n = covered.size();
        v.resize(d.getNumberFaces(), false);
        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* f = *it;
            v[f->getId()] = true;
        }
        std::tuple<int, Box3D, std::vector<bool> > triple (n, boxList.getBox(i), v);
        vectorTriples.push_back(triple);
    }
}


int Engine::deleteBoxesOld(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces){

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

int Engine::deleteBoxesOld(BoxList& boxList, const Dcel& d) {
    Dcel scaled0(d);
    Eigen::Matrix3d m[ORIENTATIONS];
    m[0] = Eigen::Matrix3d::Identity();
    CGALInterface::AABBTree t0(scaled0);
    #if ORIENTATIONS > 1
    getRotationMatrix(Vec3(0,0,1), 0.785398, m[1]);
    Dcel scaled1(d);
    scaled1.rotate(m[1]);
    getRotationMatrix(Vec3(0,0,-1), 0.785398, m[1]);
    CGALInterface::AABBTree t1(scaled1);
    getRotationMatrix(Vec3(1,0,0), 0.785398, m[2]);
    Dcel scaled2(d);
    scaled2.rotate(m[2]);
    getRotationMatrix(Vec3(-1,0,0), 0.785398, m[2]);
    CGALInterface::AABBTree t2(scaled2);
    getRotationMatrix(Vec3(0,1,0), 0.785398, m[3]);
    Dcel scaled3(d);
    scaled3.rotate(m[3]);
    getRotationMatrix(Vec3(0,-1,0), 0.785398, m[3]);
    CGALInterface::AABBTree t3(scaled3);
    #endif

    std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples;

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
        if (b.getRotationMatrix() == m[0])
            t0.getIntersectedDcelFaces(covered, b);
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
            Pointd p1 = f->getVertex1()->getCoordinate(), p2 = f->getVertex2()->getCoordinate(), p3 = f->getVertex3()->getCoordinate();

            if (!b.isIntern(p1) || !b.isIntern(p2) || !b.isIntern(p3)) {
                it =covered.erase(it);
            }
            else ++it;
        }

        std::vector<bool> v;
        int n = covered.size();
        v.resize(d.getNumberFaces(), false);
        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* f = *it;
            v[f->getId()] = true;
        }
        std::tuple<int, Box3D, std::vector<bool> > triple (n, boxList.getBox(i), v);
        vectorTriples.push_back(triple);
    }

    return deleteBoxesOld(boxList, vectorTriples, d.getNumberFaces());
}


int Engine::deleteBoxes(BoxList& boxList, const Dcel& d) {
    #ifdef GUROBI_DEFINED
    unsigned int nBoxes = boxList.getNumberBoxes();
    unsigned int nTris = d.getNumberFaces();
    Array2D<int> B(nBoxes, nTris, 0);
    CGALInterface::AABBTree aabb(d);
    for (unsigned int i = 0; i < nBoxes; i++){
        std::list<const Dcel::Face*> containedFaces = aabb.getCompletelyContainedDcelFaces(boxList.getBox(i));
        for (const Dcel::Face* f : containedFaces){
            B(i,f->getId()) = 1;
        }
    }
    try {
        GRBEnv env;
        GRBModel model(env);

        //x
        GRBVar* x = nullptr;
        x = model.addVars(nBoxes, GRB_BINARY);

        //constraints
        for (unsigned int j = 0; j < nTris; j++){
            GRBLinExpr line = 0;
            for (unsigned int i = 0; i < nBoxes; i++){
                line += B(i,j) * x[i];
            }
            model.addConstr(line >= 1);
        }
        //obj
        GRBLinExpr obj = 0;
        for (unsigned int i = 0; i < nBoxes; i++)
            obj += x[i];
        model.setObjective(obj, GRB_MINIMIZE);
        model.optimize();

        unsigned int deleted = 0;
        for (int i = nBoxes-1; i >= 0; i--){
            if (x[i].get(GRB_DoubleAttr_X) == 0){
                boxList.removeBox(i);
                deleted++;
            }
        }
        return nBoxes - deleted;


    }
    catch (GRBException e) {
        std::cerr << "Gurobi Exception\n" << e.getErrorCode() << " : " << e.getMessage() << std::endl;
        return false;
    }
    catch (...) {
        std::cerr << "Unknown Gurobi Optimization error!" << std::endl;
        return false;
    }
    return -1;
    #else
    return deleteBoxesOld(boxList, d);
    #endif
}


void Engine::optimize(BoxList& solutions, Dcel& d, double kernelDistance, bool limit, Pointd limits, bool tolerance, bool onlyNearestTarget, double areaTolerance, double angleTolerance, bool file, bool decimate) {
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

    unsigned int numberFaces = d.getNumberFaces();
    if (decimate){
        while (numberFaces/factor  < STARTING_NUMBER_FACES && factor != 1)
            factor/=2;
        numberFaces/=factor;
    }
    CGALInterface::AABBTree aabb[ORIENTATIONS];
    for (unsigned int i = 0; i < ORIENTATIONS; i++)
        aabb[i] = CGALInterface::AABBTree(scaled[i]);
    for (unsigned int i = 0; i < ORIENTATIONS; ++i){
        if (file) {
            double totalTimeGG = 0;
            for (unsigned int j = 0; j < TARGETS; ++j) {
                //if (j != 1 && j != 4){
                    std::set<const Dcel::Face*> flippedFaces, savedFaces;
                    Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                    Grid g;
                    Timer gg("Generating Grid");
                    Engine::generateGrid(g, scaled[i], kernelDistance, tolerance, XYZ[j], savedFaces);
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
                //}
            }
            std::cerr << "Total time generating Grids: " << totalTimeGG << "\n";
        }
        else {
            # pragma omp parallel for
            for (unsigned int j = 0; j < TARGETS; ++j) {
                //if (j != 1 && j != 4){
                    std::set<const Dcel::Face*> flippedFaces, savedFaces;
                    Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                    Engine::generateGrid(g[i][j], scaled[i], kernelDistance, tolerance, XYZ[j], savedFaces);
                    g[i][j].resetSignedDistances();
                    std::cerr << "Generated grid or " << i << " t " << j << "\n";
                //}
            }
        }
    }
    bool end = false;

    double totalTbg = 0;
    while (coveredFaces.size() < scaled[0].getNumberFaces() && !end){
        BoxList tmp[ORIENTATIONS][TARGETS];
        Eigen::VectorXi faces[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            IGLInterface::IGLMesh m(scaled[i]);
            IGLInterface::IGLMesh dec;
            bool b = m.getDecimatedMesh(dec, numberFaces, faces[i]);
            std::cerr << "Decimated mesh " << i << ": " << b <<"\n";
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                //if (j != 1 && j != 4){
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
                            Engine::expandBoxes(tmp[i][j], g[i][j], limit, limits);
                            std::cerr << "Orientation: " << i << " Target: " << j << " completed.\n";
                        }
                    }
                    else {
                        std::cerr << "Orientation: " << i << " Target: " << j << " no boxes to expand.\n";
                    }
                //}
            }
        }

        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                //if (j != 1 && j != 4){
                    for (unsigned int k = 0; k < tmp[i][j].getNumberBoxes(); ++k){
                        std::list<const Dcel::Face*> list;
                        aabb[i].getCompletelyContainedDcelFaces(list, tmp[i][j].getBox(k));
                        for (std::list<const Dcel::Face*>::iterator it = list.begin(); it != list.end(); ++it){
                            coveredFaces.insert((*it)->getId());
                        }
                    }
                //}
            }
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                //if (j != 1 && j != 4){
                    bl[i][j].insert(tmp[i][j]);
                //}
            }
        }

        std::cerr << "Starting Number Faces: " << numberFaces << "; Total Covered Faces: " << coveredFaces.size() << "\n";
        std::cerr << "Target: " << scaled[0].getNumberFaces() << "\n";
        if (numberFaces == scaled[0].getNumberFaces()) {
            end = true;
            if (coveredFaces.size() != scaled[0].getNumberFaces()){
                std::cerr << "WARNING: Not every face has been covered by a box.\n";
                std::cerr << "Number uncovered faces: " << scaled[0].getNumberFaces() - coveredFaces.size() << "\n";
                for (Dcel::Face* f : d.faceIterator()){
                    if (coveredFaces.find(f->getId()) == coveredFaces.end()){
                        std::cerr << "Uncovered face id: " << f->getId() << "\n";
                        f->setColor(Color(0,0,0));
                    }
                }
            }
        }
        numberFaces*=2;
        if (numberFaces > scaled[0].getNumberFaces())
            numberFaces = scaled[0].getNumberFaces();
    }
    std::cerr << "Total time Boxes Growth: " << totalTbg << "\n";

    if (file){
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                //if (j != 1 && j != 4){
                    std::stringstream ss ;
                    ss << "grid" << i << "_" << j << ".bin";
                    std::remove(ss.str().c_str());
                //}
            }
        }
    }
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        for (unsigned int j = 0; j < TARGETS; ++j){
            //if (j != 1 && j != 4){
                solutions.insert(bl[i][j]);
            //}
        }
    }
    solutions.generatePieces(d.getAverageHalfEdgesLength()*7);
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
void Engine::optimizeAndDeleteBoxes(BoxList& solutions, Dcel& d, double kernelDistance, bool limit, Pointd limits, bool tolerance, bool onlyNearestTarget, double areaTolerance, double angleTolerance, bool file, bool decimate, BoxList& allSolutions) {
    optimize(solutions, d, kernelDistance, limit, limits, tolerance, onlyNearestTarget, areaTolerance, angleTolerance, file, decimate);
    allSolutions=solutions;
    Timer tGurobi("Gurobi");
    deleteBoxes(solutions, d);
    tGurobi.stopAndPrint();
    /*assert(kernelDistance >= 0 && kernelDistance <= 1);
    solutions.clearBoxes();
    Dcel scaled[ORIENTATIONS];
    Eigen::Matrix3d m[ORIENTATIONS];
    std::vector< std::tuple<int, Box3D, std::vector<bool> > > allVectorTriples;

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

    unsigned int numberFaces = d.getNumberFaces();
    if (decimante){
        while (numberFaces/factor  < STARTING_NUMBER_FACES && factor != 1)
            factor/=2;
        numberFaces/=factor;
    }
    CGALInterface::AABBTree aabb[ORIENTATIONS];
    for (unsigned int i = 0; i < ORIENTATIONS; i++)
        aabb[i] = CGALInterface::AABBTree(scaled[i]);
    for (unsigned int i = 0; i < ORIENTATIONS; ++i){
        if (file) {
            for (unsigned int j = 0; j < TARGETS; ++j) {
                std::set<const Dcel::Face*> flippedFaces, savedFaces;
                Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                Grid g;
                Engine::generateGrid(g, scaled[i], kernelDistance, tolerance, XYZ[j], savedFaces);
                g.resetSignedDistances();
                std::stringstream ss ;
                ss << "grid" << i << "_" << j << ".bin";
                std::ofstream myfile;
                myfile.open (ss.str(), std::ios::out | std::ios::binary);
                g.serialize(myfile);
                myfile.close();
                std::cerr << "Generated grid or " << i << " t " << j << "\n";
            }
        }
        else {
            # pragma omp parallel for
            for (unsigned int j = 0; j < TARGETS; ++j) {
                std::set<const Dcel::Face*> flippedFaces, savedFaces;
                Engine::getFlippedFaces(flippedFaces, savedFaces, scaled[i], XYZ[j], angleTolerance, areaTolerance);
                Engine::generateGrid(g[i][j], scaled[i], kernelDistance, tolerance, XYZ[j], savedFaces);
                g[i][j].resetSignedDistances();
                std::cerr << "Generated grid or " << i << " t " << j << "\n";
            }
        }
    }
    bool end = false;

    while (coveredFaces.size() < scaled[0].getNumberFaces() && !end){
        BoxList tmp[ORIENTATIONS][TARGETS];
        Eigen::VectorXi faces[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            IGLInterface::IGLMesh m(scaled[i]);
            IGLInterface::IGLMesh dec;
            bool b = m.getDecimatedMesh(dec, numberFaces, faces[i]);
            std::cerr << "Decimated mesh " << i << ": " << b <<"\n";
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
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
                        Engine::expandBoxes(tmp[i][j], g);
                        std::cerr << "Orientation: " << i << " Target: " << j << " completed.\n";
                    }
                    else {
                        std::cerr << "Starting boxes growth\n";
                        Engine::expandBoxes(tmp[i][j], g[i][j]);
                        std::cerr << "Orientation: " << i << " Target: " << j << " completed.\n";
                    }
                }
                else {
                    std::cerr << "Orientation: " << i << " Target: " << j << " no boxes to expand.\n";
                }
            }
        }

        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                for (unsigned int k = 0; k < tmp[i][j].getNumberBoxes(); ++k){
                    std::list<const Dcel::Face*> list;
                    aabb[i].getCompletelyContainedDcelFaces(list, tmp[i][j].getBox(k));
                    for (std::list<const Dcel::Face*>::iterator it = list.begin(); it != list.end(); ++it){
                        coveredFaces.insert((*it)->getId());
                    }
                }
            }
        }
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                bl[i][j].insert(tmp[i][j]);
            }
        }

        std::cerr << "Starting Number Faces: " << numberFaces << "; Total Covered Faces: " << coveredFaces.size() << "\n";
        std::cerr << "Target: " << scaled[0].getNumberFaces() << "\n";
        if (numberFaces == scaled[0].getNumberFaces()) {
            end = true;
            if (coveredFaces.size() != scaled[0].getNumberFaces()){
                std::cerr << "WARNING: Not every face has been covered by a box.\n";
                std::cerr << "Number uncovered faces: " << scaled[0].getNumberFaces() - coveredFaces.size() << "\n";
            }

        }
        numberFaces*=2;
        if (numberFaces > scaled[0].getNumberFaces())
            numberFaces = scaled[0].getNumberFaces();
    }

    if (file){
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned int j = 0; j < TARGETS; ++j){
                std::stringstream ss ;
                ss << "grid" << i << "_" << j << ".bin";
                std::remove(ss.str().c_str());
            }
        }
    }
    #ifndef GUROBI_DEFINED
    std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples[ORIENTATIONS][TARGETS];
    #endif
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        for (unsigned int j = 0; j < TARGETS; ++j){
            solutions.insert(bl[i][j]);
            #ifndef GUROBI_DEFINED
            Engine::createVectorTriples(vectorTriples[i][j], bl[i][j], scaled[i]);
            allVectorTriples.insert(allVectorTriples.end(), vectorTriples[i][j].begin(), vectorTriples[i][j].end());
            #endif
        }
    }

    allSolutions = solutions;
    allSolutions.generatePieces(d.getAverageHalfEdgesLength()*7);

    #ifndef GUROBI_DEFINED
    Engine::deleteBoxesOld(solutions, allVectorTriples, d.getNumberFaces());
    #else
    Engine::deleteBoxes(solutions, d);
    #endif
    solutions.generatePieces(d.getAverageHalfEdgesLength()*7);*/
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

void Engine::booleanOperations(HeightfieldsList &he, IGLInterface::SimpleIGLMesh &bc, BoxList &solutions) {
    deleteDuplicatedBoxes(solutions);
    Timer timer("Boolean Operations");
    he.resize(solutions.getNumberBoxes());
    for (unsigned int i = 0; i <solutions.getNumberBoxes() ; i++){
        IGLInterface::SimpleIGLMesh box;
        IGLInterface::SimpleIGLMesh intersection;
        box = solutions.getBox(i).getIGLMesh();
        IGLInterface::SimpleIGLMesh::intersection(intersection, bc, box);
        IGLInterface::SimpleIGLMesh::difference(bc, bc, box);
        IGLInterface::DrawableIGLMesh dimm(intersection);
        he.addHeightfield(dimm, solutions.getBox(i).getRotatedTarget(), i);
        std::cerr << i << "\n";
    }
    timer.stopAndPrint();
    for (int i = he.getNumHeightfields()-1; i >= 0 ; i--) {
        if (he.getNumberVerticesHeightfield(i) == 0) {
            he.removeHeightfield(i);
            solutions.removeBox(i);
        }
    }
}

void Engine::splitConnectedComponents(HeightfieldsList& he, BoxList& solutions) {
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        IGLInterface::IGLMesh m = he.getHeightfield(i);
        std::vector<IGLInterface::SimpleIGLMesh> cc;
        m.getConnectedComponents(cc);
        if (cc.size() > 1){
            ///
            std::cerr << "Split: " << i << "; Number: " << cc.size() << "\n";
            ///
            Box3D box = solutions.getBox(i);
            Vec3 target = he.getTarget(i);
            he.setHeightfield(cc[0], i, true);
            for (unsigned int j = 1; j < cc.size(); j++){
                he.insertHeightfield(cc[j], target, i+j);
                solutions.insert(box, i+j);
            }
            i = i+cc.size()-1;
        }
    }
}

void Engine::glueInternHeightfieldsToBaseComplex(HeightfieldsList& he, BoxList& solutions, IGLInterface::SimpleIGLMesh& bc, const Dcel& inputMesh) {
    CGALInterface::AABBTree aabb(inputMesh, true);
    for (int i = (int)he.getNumHeightfields()-1; i >= 0; i--){
        IGLInterface::IGLMesh m = he.getHeightfield(i);
        bool inside = true;
        for (unsigned int j = 0; j < m.getNumberVertices() && inside; j++){
            if (aabb.getSquaredDistance(m.getVertex(j)) < EPSILON)
                inside = false;
        }
        if (inside){
            IGLInterface::SimpleIGLMesh::unionn(bc, bc, m);
            he.removeHeightfield(i);
            solutions.removeBox(i);
        }
    }
}

void Engine::reduceHeightfields(HeightfieldsList& he, IGLInterface::SimpleIGLMesh& bc, const Dcel& inputMesh) {
    CGALInterface::AABBTree aabb(inputMesh, true);
    double lEdge = inputMesh.getAverageHalfEdgesLength()*7;
    for (unsigned int i = he.getNumHeightfields()-1; i >= 1; i--){
        BoundingBox realBoundingBox;
        bool first = true;
        for (unsigned int j = 0; j < he.getNumberVerticesHeightfield(i); j++){
            Pointd p = he.getVertexOfHeightfield(i,j);
            if (aabb.getSquaredDistance(p) < EPSILON){
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
        for (unsigned int t = 0; t < 6; t++){
            if (he.getTarget(i) == XYZ[t]){
                if (t < 3)
                    realBoundingBox[t] = std::max(realBoundingBox[t] - 1, he.getHeightfield(i).getBoundingBox()[t]);
                else
                    realBoundingBox[t] = std::min(realBoundingBox[t] + 1, he.getHeightfield(i).getBoundingBox()[t]);
            }
        }


        if (! Common::epsilonEqual(realBoundingBox.min(), he.getHeightfield(i).getBoundingBox().min()) ||
            ! Common::epsilonEqual(realBoundingBox.max(), he.getHeightfield(i).getBoundingBox().max()) ){
            IGLInterface::SimpleIGLMesh box = IGLInterface::makeBox(realBoundingBox, lEdge);
            IGLInterface::SimpleIGLMesh oldHeightfield = he.getHeightfield(i);
            IGLInterface::SimpleIGLMesh gluePortion = IGLInterface::SimpleIGLMesh::difference(oldHeightfield, box);
            IGLInterface::SimpleIGLMesh newHeightfield = IGLInterface::SimpleIGLMesh::intersection(oldHeightfield, box);
            IGLInterface::SimpleIGLMesh::unionn(bc, bc, gluePortion);
            he.setHeightfield(newHeightfield,i,true);
        }
    }
}

void Engine::gluePortionsToBaseComplex(HeightfieldsList& he, IGLInterface::SimpleIGLMesh& bc, BoxList& solutions, const Dcel& inputMesh) {
    CGALInterface::AABBTree aabb(inputMesh, true);
    for (unsigned int i = solutions.getNumberBoxes()-1; i >= 1; i--){
        IGLInterface::SimpleIGLMesh heightfield = he.getHeightfield(i);
        std::vector<Pointd> pointsOnSurface;
        for (unsigned int j = 0; j < heightfield.getNumberVertices(); j++){
            Pointd p = heightfield.getVertex(j);
            if (aabb.getSquaredDistance(p) < EPSILON) pointsOnSurface.push_back(p);
        }
        Eigen::Matrix3d m = solutions.getBox(i).getRotationMatrix(), mt;
        Eigen::Matrix3d arr[4];
        arr[0] = Eigen::Matrix3d::Identity();
        Common::getRotationMatrix(Vec3(0,0,-1), M_PI/4, arr[1]);
        Common::getRotationMatrix(Vec3(0,-1,0), M_PI/4, arr[2]);
        Common::getRotationMatrix(Vec3(-1,0,0), M_PI/4, arr[3]);
        if (m == arr[0])
            mt = arr[0];
        else if (m == arr[1])
            Common::getRotationMatrix(Vec3(0,0,1), M_PI/4, mt);
        else if (m == arr[2])
            Common::getRotationMatrix(Vec3(0,1,0), M_PI/4, mt);
        else if (m == arr[3])
            Common::getRotationMatrix(Vec3(1,0,0), M_PI/4, mt);
        else assert(0);
        pointsOnSurface[0].rotate(mt);
        Pointd min = pointsOnSurface[0], max = pointsOnSurface[0];
        for (unsigned int j = 1; j < pointsOnSurface.size(); j++){
            pointsOnSurface[j].rotate(mt);
            min = min.min(pointsOnSurface[j]);
            max = max.max(pointsOnSurface[j]);
        }
        Vec3 target = solutions.getBox(i).getTarget();
        Box3D b;
        if (target == XYZ[0] || target == XYZ[3]){
            b = solutions.getBox(i);
            b.setMinY(min.y());
            b.setMinZ(min.z());
            b.setMaxY(max.y());
            b.setMaxZ(max.z());
            b.setMinX(b.getMinX()-5*EPSILON);
            b.setMaxX(b.getMaxX()+5*EPSILON);
        }
        else if (target == XYZ[1] || target == XYZ[4]) {
            b = solutions.getBox(i);
            b.setMinX(min.x());
            b.setMinZ(min.z());
            b.setMaxX(max.x());
            b.setMaxZ(max.z());
            b.setMinY(b.getMinY()-5*EPSILON);
            b.setMaxY(b.getMaxY()+5*EPSILON);
        }
        else if (target == XYZ[2] || target == XYZ[5]) {
            b = solutions.getBox(i);
            b.setMinY(min.y());
            b.setMinX(min.x());
            b.setMaxY(max.y());
            b.setMaxX(max.x());
            b.setMinZ(b.getMinZ()-5*EPSILON);
            b.setMaxZ(b.getMaxZ()+5*EPSILON);
        }
        else assert(0);
        IGLInterface::SimpleIGLMesh inters;
        IGLInterface::SimpleIGLMesh diff;
        IGLInterface::SimpleIGLMesh box;
        //solutions.setBox(i,b);
        box = b.getIGLMesh();
        IGLInterface::SimpleIGLMesh::intersection(inters, heightfield, box);
        IGLInterface::SimpleIGLMesh::difference(diff, heightfield, box);
        IGLInterface::SimpleIGLMesh::unionn(bc, bc, diff);
        he.addHeightfield(IGLInterface::DrawableIGLMesh(inters), solutions.getBox(i).getRotatedTarget(),  i);
        std::cerr << i << "\n";
    }

}

/*void Engine::largeScaleFabrication(const Dcel& input, double kernelDistance, bool heightfields) {
    Dcel scaled[ORIENTATIONS];
    Eigen::Matrix3d m[ORIENTATIONS];
    std::vector< std::tuple<int, Box3D, std::vector<bool> > > allVectorTriples;
    BoxList allBoxes;
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        scaled[i] = input;
        m[i] = scaleAndRotateDcel(scaled[i], i);
    }
    if (!heightfields){
        Grid g[ORIENTATIONS];
        BoxList bl[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            generateGrid(g[i], scaled[i], kernelDistance);
            calculateInitialBoxes(bl[i],scaled[i], m[i], false);
            expandBoxes(bl[i], g[i]);

            //Salvataggio
            std::stringstream ss;
            ss << "Engine" << i << ".bin";
            std::ofstream myfile;
            myfile.open (ss.str(), std::ios::out | std::ios::binary);
            serializeAsEngineManager(myfile, g[i], scaled[i], bl[i]);
            myfile.close();
        }

        std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            allBoxes.insert(bl[i]);
            createVectorTriples(vectorTriples[i], bl[i], scaled[i]);
            allVectorTriples.insert(allVectorTriples.end(), vectorTriples[i].begin(), vectorTriples[i].end());
        }

        deleteBoxesOld(allBoxes, allVectorTriples, input.getNumberFaces());

        std::ofstream myfile;
        myfile.open ("allSolutions.bin", std::ios::out | std::ios::binary);
        allBoxes.serialize(myfile);
        myfile.close();


        myfile.open ("engineManagerAllSolutions.bin", std::ios::out | std::ios::binary);
        serializeAsEngineManager(myfile, g[0], scaled[0], allBoxes);
        myfile.close();


    }
    else {
        Grid g[ORIENTATIONS][TARGETS];
        BoxList bl[ORIENTATIONS][TARGETS];
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                generateGrid(g[i][j], scaled[i], kernelDistance, true, XYZ[j]);
                calculateInitialBoxes(bl[i][j],scaled[i], m[i], true, XYZ[j]);
                expandBoxes(bl[i][j], g[i][j]);

                //Salvataggio
                std::stringstream ss;
                ss << "Engine" << i << "Target" << j<<".bin";
                std::ofstream myfile;
                myfile.open (ss.str(), std::ios::out | std::ios::binary);
                serializeAsEngineManager(myfile, g[i][j], scaled[i], bl[i][j]);
                myfile.close();
            }
        }

        std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples[ORIENTATIONS][TARGETS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            for (unsigned int j = 0; j < TARGETS; ++j){
                allBoxes.insert(bl[i][j]);
                createVectorTriples(vectorTriples[i][j], bl[i][j], scaled[i]);
                allVectorTriples.insert(allVectorTriples.end(), vectorTriples[i][j].begin(), vectorTriples[i][j].end());
            }
        }

        deleteBoxesOld(allBoxes, allVectorTriples, input.getNumberFaces());

        std::ofstream myfile;
        myfile.open ("allSolutions.bin", std::ios::out | std::ios::binary);
        allBoxes.serialize(myfile);
        myfile.close();


        myfile.open ("engineManagerAllSolutions.bin", std::ios::out | std::ios::binary);
        serializeAsEngineManager(myfile, g[0][0], scaled[0], allBoxes);
        myfile.close();

    }

}*/

IGLInterface::SimpleIGLMesh Engine::getMarkerMesh(const HeightfieldsList& he, const Dcel &d) {
    CGALInterface::AABBTree tree(d, true);
    std::set< std::pair<Pointd, Pointd> > edges;
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const IGLInterface::IGLMesh& mesh = he.getHeightfield(i);
        Eigen::MatrixXi TT = mesh.getFacesAdjacences();
        for (unsigned int f = 0; f < mesh.getNumberFaces(); f++){
            Vec3 n1 = mesh.getFaceNormal(f);
            if (n1.dot(he.getTarget(i))<=EPSILON){
                for (unsigned int k = 0; k < 3; k++){
                    int adj = TT(f,k);
                    if (adj >= 0){
                        Vec3 n2 = mesh.getFaceNormal(adj);
                        if (n2.dot(he.getTarget(i))>=-EPSILON){
                            Pointi f1 = mesh.getFace(f);
                            Pointi f2 = mesh.getFace(adj);
                            std::set<Pointd> allPoints;
                            for (unsigned int i = 0; i < 3; i++){
                                allPoints.insert(mesh.getVertex(f1[i]));
                                allPoints.insert((mesh.getVertex(f1[i]) + mesh.getVertex(f1[(i+1)%3]))/2);
                            }
                            for (unsigned int i = 0; i < 3; i++){
                                allPoints.insert(mesh.getVertex(f2[i]));
                                allPoints.insert((mesh.getVertex(f2[i]) + mesh.getVertex(f2[(i+1)%3]))/2);
                            }
                            bool allNear = true;
                            bool allDist = true;
                            for (Pointd  p : allPoints){
                                if (tree.getSquaredDistance(p) > EPSILON)
                                    allNear = false;
                                else
                                    allDist = false;
                            }
                            if (!allNear && !allDist){
                                int v1, v2;
                                bool finded = false;
                                unsigned int t1, tmp;
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
                                Pointd p1 = mesh.getVertex(v1);
                                Pointd p2 = mesh.getVertex(v2);
                                std::pair<Pointd, Pointd> edge;
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
    IGLInterface::SimpleIGLMesh marked;
    std::map<Pointd, int> mapVertices;
    int n = 0;
    for (std::pair<Pointd, Pointd> edge : edges){
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
        Pointd midPoint = (edge.first + edge.second) / 2.0;
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

void Engine::saveObjs(const QString& foldername, const IGLInterface::IGLMesh &originalMesh, const Dcel& inputMesh, const IGLInterface::IGLMesh &baseComplex, const HeightfieldsList &he) {
    QString originalMeshString = foldername + "/OriginalMesh.obj";
    QString inputMeshString = foldername + "/InputMesh.obj";
    QString baseComplexString = foldername + "/BaseComplex.obj";
    QString heightfieldString = foldername + "/Heightfield";
    QString markerString = foldername + "/Marker.obj";
    //CGALInterface::AABBTree tree(inputMesh);
    if (originalMesh.getNumberVertices() > 0)
        originalMesh.saveOnObj(originalMeshString.toStdString());
    inputMesh.saveOnObjFile(inputMeshString.toStdString());
    baseComplex.saveOnObj(baseComplexString.toStdString());
    IGLInterface::SimpleIGLMesh marker = getMarkerMesh(he, inputMesh);
    marker.saveOnObj(markerString.toStdString());
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        IGLInterface::IGLMesh h = he.getHeightfield(i);
        Dcel d(h);
        //updatePieceNormals(tree, d);
        std::stringstream ss;
        ss << heightfieldString.toStdString() << i << ".obj";
        d.saveOnObjFile(ss.str());
    }
}

void Engine::updatePieceNormals(const CGALInterface::AABBTree& tree, Dcel& piece) {
    for (Dcel::Vertex* v : piece.vertexIterator()){
        if (tree.getSquaredDistance(v->getCoordinate() < EPSILON)){
            const Dcel::Vertex* n = tree.getNearestDcelVertex(v->getCoordinate());
            v->setNormal(n->getNormal());
        }
    }
}
