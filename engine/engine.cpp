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



Eigen::Matrix3d Engine::scaleAndRotateDcel(Dcel& d, int resolution, int rot) {
    BoundingBox bb = d.getBoundingBox();
    double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
    maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
    double av = maxl / resolution;
    BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);
    d.scale(nBB);

    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    if (rot > 0){
        switch (rot){
            case 1:
                getRotationMatrix(Vec3(0,0,1), 0.785398, m);
                d.rotate(m);
                break;
            case 2:
                getRotationMatrix(Vec3(1,0,0), 0.785398, m);
                d.rotate(m);
                break;
            case 3:
                getRotationMatrix(Vec3(0,1,0), 0.785398, m);
                d.rotate(m);
                break;
            default:
                assert(0);
        }
    }
    /**
     * @todo is transpose right?
     */
    m.transpose();
    return m;
}

void Engine::generateGrid(Grid& g, const Dcel& d, double kernelDistance, const Vec3 &target, bool heightfields) {


    d.saveOnObjFile("tmp.obj");
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

int Engine::deleteBoxes(BoxList& boxList, const Dcel &d){
    CGALInterface::AABBTree t(d);


    // creating vector of pairs
    std::vector< std::tuple<int, Box3D, std::vector<unsigned int> > > vectorTriples;
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

        std::vector<unsigned int> v;
        int n = covered.size();
        v.resize(d.getNumberFaces(), 0);
        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* f = *it;
            v[f->getId()] = 1;
        }
        std::tuple<int, Box3D, std::vector<unsigned int> > triple (n, boxList.getBox(i), v);
        vectorTriples.push_back(triple);

    }

    //ordering vector of triples
    struct triplesOrdering {
        bool operator ()(const std::tuple<int, Box3D, std::vector<unsigned int> >& a, const std::tuple<int, Box3D, std::vector<unsigned int> >& b) {
            if (std::get<0>(a) < std::get<0>(b))
                return true;
            if (std::get<0>(a) == std::get<0>(b))
                return ((std::get<1>(a).getVolume()) < (std::get<1>(b).getVolume()));
            return false;
        }
    };
    std::sort(vectorTriples.begin(), vectorTriples.end(), triplesOrdering());


    //create m
    std::vector< std::vector<unsigned int>> m;
    m.reserve(boxList.getNumberBoxes());
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++)
        m.push_back(std::get<2>(vectorTriples[i]));

    //create sums
    std::vector<unsigned int> sums;
    sums.resize(d.getNumberFaces(), 0);
    for (unsigned j = 0; j < d.getNumberFaces(); j++){
        for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++){
            sums[j] += m[i][j];
        }
    }

    //calculating erasable elements
    std::vector<unsigned int> eliminate;
    for (unsigned int i = 0; i < boxList.getNumberBoxes(); i++){
        bool b = true;
        for (unsigned int j = 0; j < d.getNumberFaces(); j++)
            if (sums[j]-m[i][j] < 1)
                b = false;
        if (b){
            for (unsigned int j = 0; j < d.getNumberFaces(); j++)
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

void Engine::largeScaleFabrication(const Dcel& input, int resolution, double kernelDistance, bool heightfields) {
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
            expandBoxes(bl[i], g[i]);

            //Salvataggio
            std::stringstream ss;
            ss << "Engine" << i << ".bin";
            std::ofstream myfile;
            myfile.open (ss.str(), std::ios::out | std::ios::binary);
            serializeAsEngineManager(myfile, g[i], scaled[i], bl[i]);
            myfile.close();
        }
        BoxList allBoxes;
        for (unsigned int i = 0; i < ORIENTATIONS; i++){
            allBoxes.insert(bl[i]);
        }

        std::ofstream myfile;
        myfile.open ("allSolutions.bin", std::ios::out | std::ios::binary);
        allBoxes.serialize(myfile);
        myfile.close();

        deleteBoxes(allBoxes, input);
        myfile.open ("simplifiedSolutionsAndMesh.bin", std::ios::out | std::ios::binary);
        allBoxes.serialize(myfile);
        input.serialize(myfile);
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
