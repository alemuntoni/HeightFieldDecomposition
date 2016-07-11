#include "engine.h"
#include "lib/igl/iglinterface.h"

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

Eigen::Matrix3d Engine::rotateDcelAlreadyScaled(Dcel& d, unsigned int rot) {
    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (rot > 0){
        switch (rot){
            case 1:
                getRotationMatrix(Vec3(0,0,1), 0.785398, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                getRotationMatrix(Vec3(0,0,-1), 0.785398, m);
                break;
            case 2:
                getRotationMatrix(Vec3(1,0,0), 0.785398, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                getRotationMatrix(Vec3(-1,0,0), 0.785398, m);
                break;
            case 3:
                getRotationMatrix(Vec3(0,1,0), 0.785398, m);
                d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                getRotationMatrix(Vec3(0,-1,0), 0.785398, m);
                break;
            default:
                assert(0);
        }
    }
    return m;
}

Eigen::Matrix3d Engine::scaleAndRotateDcel(Dcel& d, int resolution, unsigned int rot) {
    BoundingBox bb = d.getBoundingBox();
    double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
    maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
    double av = maxl / resolution;
    BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);
    d.scale(nBB);

    /*Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (rot > 0){
        switch (rot){
            case 1:
                getRotationMatrix(Vec3(0,0,1), 0.785398, m);
                d.rotate(m);
                getRotationMatrix(Vec3(0,0,-1), 0.785398, m);
                break;
            case 2:
                getRotationMatrix(Vec3(1,0,0), 0.785398, m);
                d.rotate(m);
                getRotationMatrix(Vec3(-1,0,0), 0.785398, m);
                break;
            case 3:
                getRotationMatrix(Vec3(0,1,0), 0.785398, m);
                d.rotate(m);
                getRotationMatrix(Vec3(0,-1,0), 0.785398, m);
                break;
            default:
                assert(0);
        }
    }
    return m;*/
    return rotateDcelAlreadyScaled(d, rot);
}

void Engine::generateGrid(Grid& g, const Dcel& d, double kernelDistance, bool heightfields, const Vec3 &target) {


    //d.saveOnObjFile("tmp.obj");
    IGLInterface::generateGridAndDistanceField("tmp.obj");
    exec("./grid_generator tmp.obj");


    Eigen::RowVector3i nGmin;
    Eigen::RowVector3i nGmax;
    Eigen::VectorXd S;
    Eigen::MatrixXd GV;
    Eigen::RowVector3i res;

    std::ifstream file;
    file.open ("tmp.bin", std::ios::in | std::ios::binary);
    Serializer::deserialize(nGmin, file);
    Serializer::deserialize(nGmax, file);
    Serializer::deserialize(res, file);
    Serializer::deserialize(GV, file);
    Serializer::deserialize(S, file);
    file.close();

    g = Grid(res, GV, S, nGmin, nGmax);
    g.setTarget(target);
    g.calculateWeightsAndFreezeKernel(d, kernelDistance, heightfields);
    Energy e(g);
    e.calculateFullBoxValues(g);

    std::remove("tmp.bin");
    std::remove("tmp.obj");
    std::remove("tmp.mtu");
}

void Engine::calculateInitialBoxes(BoxList& boxList, const Dcel& d, const Eigen::Matrix3d &rot, bool onlyTarget, const Vec3& target) {
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        Vec3 n =f->getNormal();
        Vec3 closestTarget = getClosestTarget(n);
        if (!onlyTarget || (onlyTarget && closestTarget == target)){
            Box3D box;
            box.setTarget(closestTarget);
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
            box.setColor(colorOfNormal(closestTarget));
            box.setConstraint1(p1);
            box.setConstraint2(p2);
            box.setConstraint3(p3);
            box.setRotationMatrix(rot);
            boxList.addBox(box);
        }
    }
}

void Engine::expandBoxes(BoxList& boxList, const Grid& g) {
    Energy e(g);
    Timer total("Minimization All Boxes");
    int np = boxList.getNumberBoxes();
    # pragma omp parallel for if(np>10)
    for (int i = 0; i < np; i++){
        Box3D b = boxList.getBox(i);
        std::stringstream ss;
        ss << "Minimization " << i << " box";
        Timer t(ss.str());
        e.gradientDiscend(b);
        t.stopAndPrint();
        boxList.setBox(i, b);
        std::cerr << "Total: " << total.delay() << "\n\n";
    }
    total.stopAndPrint();
    std::ofstream myfile;
    myfile.open ("solutions.bin", std::ios::out | std::ios::binary);
    boxList.serialize(myfile);
    myfile.close();

}

void Engine::createVectorTriples(std::vector< std::tuple<int, Box3D, std::vector<bool> > > &vectorTriples, const BoxList& boxList, const Dcel& d) {
    CGALInterface::AABBTree t(d);


    // creating vector of pairs

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
        t.getIntersectedPrimitives(covered, b);

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
    std::sort(vectorTriples.begin(), vectorTriples.end(), triplesOrdering());


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

    std::vector< std::tuple<int, Box3D, std::vector<bool> > > vectorTriples;

    vectorTriples.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); ++i){
        Box3D b = boxList.getBox(i);
        std::list<const Dcel::Face*> covered;
        if (b.getRotationMatrix() == m[0])
            t0.getIntersectedPrimitives(covered, b);
        else if (b.getRotationMatrix() == m[1])
            t1.getIntersectedPrimitives(covered, b);
        else if (b.getRotationMatrix() == m[2])
            t2.getIntersectedPrimitives(covered, b);
        else if (b.getRotationMatrix() == m[3])
            t3.getIntersectedPrimitives(covered, b);
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

}


void Engine::makePreprocessingAndSave(const Dcel& input, const std::__cxx11::string& filename, int resolution, double kernelDistance, bool heightfields) {
    Dcel scaled[ORIENTATIONS];
    Eigen::Matrix3d m[ORIENTATIONS];
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        scaled[i] = input;
        m[i] = scaleAndRotateDcel(scaled[i], resolution, i);
    }

    if (!heightfields){
        Grid g[ORIENTATIONS];
        BoxList bl[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            generateGrid(g[i], scaled[i], kernelDistance);
            calculateInitialBoxes(bl[i],scaled[i], m[i], false);

        }
        std::ofstream myfile;
        myfile.open (filename, std::ios::out | std::ios::binary);
        scaled[0].serialize(myfile);
        Serializer::serialize(heightfields, myfile);
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            g[i].serialize(myfile);
            bl[i].serialize(myfile);
        }

        myfile.close();

    }
    else {
        Grid g[ORIENTATIONS][TARGETS];
        BoxList bl[ORIENTATIONS][TARGETS];
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                generateGrid(g[i][j], scaled[i], kernelDistance, true, XYZ[j]);
                calculateInitialBoxes(bl[i][j],scaled[i], m[i], true, XYZ[j]);
            }
        }
        std::ofstream myfile;
        myfile.open (filename, std::ios::out | std::ios::binary);
        scaled[0].serialize(myfile);
        Serializer::serialize(heightfields, myfile);
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            for (unsigned j = 0; j < TARGETS; ++j){
                g[i][j].serialize(myfile);
                bl[i][j].serialize(myfile);
            }
        }
        myfile.close();
    }
}

void Engine::expandBoxesFromPreprocessing(const std::__cxx11::string& inputFile, const std::__cxx11::string& outputFile) {
    Dcel d;
    bool heightfields;

    std::ifstream input;
    input.open (inputFile, std::ios::in | std::ios::binary);
    d.deserialize(input);
    Serializer::deserialize(heightfields, input);
    if (!heightfields){
        Grid g[ORIENTATIONS];
        BoxList bl[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            g[i].deserialize(input);
            bl[i].deserialize(input);
        }
        input.close();
        Timer t("Total Time Entire Process");
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            expandBoxes(bl[i], g[i]);
        }
        t.stopAndPrint();
        std::ofstream myfile;
        myfile.open (outputFile, std::ios::out | std::ios::binary);
        d.serialize(myfile);
        Serializer::serialize(heightfields, myfile);
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            g[i].serialize(myfile);
            bl[i].serialize(myfile);
        }
        myfile.close();
    }
    else {
        Grid g[ORIENTATIONS][TARGETS];
        BoxList bl[ORIENTATIONS][TARGETS];
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                g[i][j].deserialize(input);
                bl[i][j].deserialize(input);
            }
        }
        input.close();
        Timer t("Total Time Entire Process");
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                expandBoxes(bl[i][j], g[i][j]);
            }
        }
        t.stopAndPrint();
        std::ofstream myfile;
        myfile.open (outputFile, std::ios::out | std::ios::binary);
        d.serialize(myfile);
        Serializer::serialize(heightfields, myfile);
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                g[i][j].serialize(myfile);
                bl[i][j].serialize(myfile);
            }
        }
        myfile.close();;
    }
}

void Engine::afterExpandedBoxes(const std::__cxx11::string& inputFile) {
    Dcel d;
    bool heightfields;

    std::ifstream input;
    input.open (inputFile, std::ios::in | std::ios::binary);
    d.deserialize(input);
    Serializer::deserialize(heightfields, input);
    if (!heightfields){
        Grid g[ORIENTATIONS];
        BoxList bl[ORIENTATIONS];
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            g[i].deserialize(input);
            bl[i].deserialize(input);
        }
        input.close();


    }
    else {
        Grid g[ORIENTATIONS][TARGETS];
        BoxList bl[ORIENTATIONS][TARGETS];
        for (unsigned int i = 0; i < ORIENTATIONS; ++i){
            for (unsigned j = 0; j < TARGETS; ++j){
                g[i][j].deserialize(input);
                bl[i][j].deserialize(input);
            }
        }
        input.close();


    }
}

void Engine::largeScaleFabrication(const Dcel& input, int resolution, double kernelDistance, bool heightfields) {
    Dcel scaled[ORIENTATIONS];
    Eigen::Matrix3d m[ORIENTATIONS];
    std::vector< std::tuple<int, Box3D, std::vector<bool> > > allVectorTriples;
    BoxList allBoxes;
    for (unsigned int i = 0; i < ORIENTATIONS; i++){
        scaled[i] = input;
        m[i] = scaleAndRotateDcel(scaled[i], resolution, i);
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

void Engine::serializeAsEngineManager(std::ofstream& binaryfile, const Grid& g, const Dcel &d, const BoxList &bl) {
    g.serialize(binaryfile);
    d.serialize(binaryfile);
    bool b = true;
    Serializer::serialize(b, binaryfile);
    bl.serialize(binaryfile);
}
