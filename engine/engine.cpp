#include "engine.h"


void Engine::generateGrid(Grid& g, Dcel& d, int resolution, const Vec3 &target, double kernelDistance, bool heightfields) {
    BoundingBox bb = d.getBoundingBox();
    double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
    maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
    double av = maxl / resolution;
    BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);
    d.scale(nBB);

    /*Eigen::Matrix3d m;
    getRotationMatrix(Vec3(0,0,1), 0.785398, m);
    d.rotate(m);*/

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
