#include "engine.h"

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

void Engine::generateGrid(Grid& g, const Dcel& d, double kernelDistance, bool tolerance, const Vec3 &target, std::set<const Dcel::Face*>& savedFaces) {
    IGLInterface::SimpleIGLMesh m(d);
    Array3D<Pointd> grid;
    Array3D<gridreal> distanceField;
    IGLInterface::generateGridAndDistanceField(grid, distanceField, m);
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

void Engine::expandBoxes(BoxList& boxList, const Grid& g, bool printTimes) {
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
        e.BFGS(b);
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


int Engine::deleteBoxes(BoxList& boxList, std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, unsigned int numberFaces){

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

int Engine::deleteBoxes(BoxList& boxList, const Dcel& d) {
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

    return deleteBoxes(boxList, vectorTriples, d.getNumberFaces());
}


int Engine::deleteBoxesMemorySafe(BoxList& boxList, const Dcel& d) {
    std::vector<BoxList> boxLists;
    boxList.getSubBoxLists(boxLists, d.getNumberFaces());
    for (unsigned int i = 0; i < boxLists.size(); i++){
        deleteBoxes(boxLists[i], d);
    }
    boxList.clearBoxes();
    for (unsigned int i = 0; i < boxLists.size(); i++){
        boxList.insert(boxLists[i]);
    }
    return deleteBoxes(boxList, d);
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
void Engine::createAndMinimizeAllBoxes(BoxList& solutions, const Dcel& d, double kernelDistance, bool tolerance, bool onlyNearestTarget, double areaTolerance, double angleTolerance, bool file, bool decimante) {
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
    unsigned int numberFaces = 200;
    if (!decimante)
        numberFaces = d.getNumberFaces();
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

    std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples[ORIENTATIONS][TARGETS];
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        for (unsigned int j = 0; j < TARGETS; ++j){
            solutions.insert(bl[i][j]);
            Engine::createVectorTriples(vectorTriples[i][j], bl[i][j], scaled[i]);
            allVectorTriples.insert(allVectorTriples.end(), vectorTriples[i][j].begin(), vectorTriples[i][j].end());
        }
    }

    Engine::deleteBoxes(solutions, allVectorTriples, d.getNumberFaces());
    solutions.generatePieces(d.getAverageHalfEdgesLength()*7);
}

void Engine::booleanOperations(HeightfieldsList &he, IGLInterface::SimpleIGLMesh &bc, BoxList &solutions, const Dcel& inputMesh) {
    double average = 0;
    for (const Dcel::HalfEdge* he : inputMesh.halfEdgeIterator())
        average += he->getLength();
    average /= inputMesh.getNumberHalfEdges();
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

void Engine::largeScaleFabrication(const Dcel& input, double kernelDistance, bool heightfields) {
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

        deleteBoxes(allBoxes, allVectorTriples, input.getNumberFaces());

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

        deleteBoxes(allBoxes, allVectorTriples, input.getNumberFaces());

        std::ofstream myfile;
        myfile.open ("allSolutions.bin", std::ios::out | std::ios::binary);
        allBoxes.serialize(myfile);
        myfile.close();


        myfile.open ("engineManagerAllSolutions.bin", std::ios::out | std::ios::binary);
        serializeAsEngineManager(myfile, g[0][0], scaled[0], allBoxes);
        myfile.close();

    }

}
