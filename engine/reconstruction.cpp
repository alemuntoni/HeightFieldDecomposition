#include "reconstruction.h"

#include "common.h"
#include "common/timer.h"
#include "lib/graph/directedgraph.h"
#include <map>

void Reconstruction::compactSet(std::set<double>& set, double epsilon) {
    std::set<double>::iterator it = set.begin();
    std::set<double> toDelete;
    double last = *it;
    ++it;
    double actual;
    for (; it != set.end(); ++it) {
        actual = *it;
        if (Common::epsilonEqual(last, actual, epsilon)){
            toDelete.insert(actual);
        }
        else {
            last = actual;
        }
    }
    for (double actual : toDelete){
        set.erase(actual);
    }
}

void Reconstruction::createIrregularGrid(IrregularGrid& grid, const BoxList& solutions, const Dcel &d, double epsilon) {
    CGALInterface::AABBTree aabb(d);
    std::set<double> xCoord, yCoord, zCoord;
    for (unsigned int i = 0; i < solutions.getNumberBoxes(); ++i){
        Box3D b = solutions.getBox(i);
        xCoord.insert(b.getMinX());
        xCoord.insert(b.getMaxX());
        yCoord.insert(b.getMinY());
        yCoord.insert(b.getMaxY());
        zCoord.insert(b.getMinZ());
        zCoord.insert(b.getMaxZ());
    }
    // compacting
    compactSet(xCoord, epsilon);
    compactSet(yCoord, epsilon);
    compactSet(zCoord, epsilon);


    grid.reset(xCoord.size(), yCoord.size(), zCoord.size());
    unsigned int i = 0, j = 0, k = 0;
    for (double x : xCoord){
        j = 0;
        for (double y : yCoord) {
            k = 0;
            for (double z: zCoord) {
                grid.addPoint(i,j,k, Pointd(x,y,z));
                k++;
            }
            j++;
        }
        i++;
    }
    int nBoxes = solutions.getNumberBoxes();
    #pragma omp parallel for
    for (int sol = 0; sol < nBoxes; ++sol){
        Box3D b = solutions.getBox(sol);
        for (unsigned int i = 0; i < xCoord.size()-1; i++) {
            for (unsigned int j = 0; j < yCoord.size()-1; j++) {
                for (unsigned int k = 0; k < zCoord.size()-1; k++) {
                    Pointd min = grid.getPoint(i,j,k);
                    Pointd max = grid.getPoint(i+1, j+1, k+1);
                    //se almeno un punto è interno alla shape in input
                    Pointd p1 = grid.getPoint(i+1,j  ,k  );
                    Pointd p2 = grid.getPoint(i  ,j+1,k  );
                    Pointd p3 = grid.getPoint(i+1,j+1,k  );
                    Pointd p4 = grid.getPoint(i  ,j  ,k+1);
                    Pointd p5 = grid.getPoint(i+1,j  ,k+1);
                    Pointd p6 = grid.getPoint(i  ,j+1,k+1);
                    if (aabb.getNumberIntersectedPrimitives(BoundingBox(min,max)) > 0
                            || aabb.isInside(min) || aabb.isInside(max) || aabb.isInside(p1)  || aabb.isInside(p2)
                            || aabb.isInside(p3)  || aabb.isInside(p4)  || aabb.isInside(p5)  || aabb.isInside(p6)) {
                        if (b.isEpsilonIntern(min, epsilon) && b.isEpsilonIntern(max, epsilon)){
                            #pragma omp critical
                            {
                                grid.addPossibleTarget(i,j,k, b.getTarget());
                            }
                        }
                    }
                }
            }
        }
    }
}

Pointi Reconstruction::getGrowthStep(const Vec3& target) {
    Pointi step;
    if (target == XYZ[0])
        step.setX(1);
    else if (target == XYZ[1])
        step.setY(1);
    else if (target == XYZ[2])
        step.setZ(1);
    else if (target == XYZ[3])
        step.setX(-1);
    else if (target == XYZ[4])
        step.setY(-1);
    else if (target == XYZ[5])
        step.setZ(-1);
    else assert(0);
    return step;
}

void Reconstruction::getAdjacents(Pointi& s1, Pointi& s2, Pointi& s3, Pointi& s4, const Pointi& base, const Vec3& target) {
    if (target == XYZ[0] || target == XYZ[3]){
        s1 = base + Pointi( 0, 1, 0);
        s2 = base + Pointi( 0, 0, 1);
        s3 = base + Pointi( 0,-1, 0);
        s4 = base + Pointi( 0, 0,-1);
    }
    else if (target == XYZ[1] || target == XYZ[4]){
        s1 = base + Pointi( 1, 0, 0);
        s2 = base + Pointi( 0, 0, 1);
        s3 = base + Pointi(-1, 0, 0);
        s4 = base + Pointi( 0, 0,-1);
    }
    else if (target == XYZ[2] || target == XYZ[5]){
        s1 = base + Pointi( 0, 1, 0);
        s2 = base + Pointi( 1, 0, 0);
        s3 = base + Pointi( 0,-1, 0);
        s4 = base + Pointi(-1, 0, 0);
    }
    else assert(0);
}

bool Reconstruction::isBounded(const Pointi &box, const IrregularGrid& g){
    return (box.x() >= 0 && box.y() >= 0 && box.z() >= 0) && (box.x() < (int)g.getResolutionX()-1 && box.y() < (int)g.getResolutionY()-1 && box.z() < (int)g.getResolutionZ()-1);
}

Pointi Reconstruction::getBase(const IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    assert(g.boxHasPossibleTarget(startingBox.x(), startingBox.y(), startingBox.z(), target) && !(g.isDefinitiveTarget(startingBox.x(), startingBox.y(), startingBox.z())));
    Pointi step = getGrowthStep(target);

    Pointi base = startingBox;
    do {
        base -= step;
    } while (isBounded(base, g) && g.boxHasPossibleTarget(base.x(), base.y(), base.z(), target) && !(g.isDefinitiveTarget(base.x(), base.y(), base.z())));
    base += step;

    return base;
}

void Reconstruction::growTarget(std::set<Pointi>& conqueredBoxes, const IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    assert(g.boxHasPossibleTarget(startingBox.x(), startingBox.y(), startingBox.z(), target) && !(g.isDefinitiveTarget(startingBox.x(), startingBox.y(), startingBox.z())));
    Pointi step = getGrowthStep(target);
    Pointi base = startingBox;
    do {
        conqueredBoxes.insert(base);
        base += step;
    } while (isBounded(base, g) && g.boxHasPossibleTarget(base.x(), base.y(), base.z(), target));
}

void Reconstruction::recursiveGrowth(std::set<Pointi>& conqueredBoxes, const IrregularGrid& g, const Pointi& startingBaseBox, const Vec3& target) {
    growTarget(conqueredBoxes, g, startingBaseBox, target);
    Pointi s1, s2, s3, s4;
    getAdjacents(s1, s2, s3, s4, startingBaseBox, target);

    if (isBounded(s1, g))
        if (g.boxHasPossibleTarget(s1.x(), s1.y(), s1.z(), target) && conqueredBoxes.find(s1) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s1.x(), s1.y(), s1.z())))
            recursiveGrowth(conqueredBoxes, g, s1, target);
    if (isBounded(s2, g))
        if (g.boxHasPossibleTarget(s2.x(), s2.y(), s2.z(), target) && conqueredBoxes.find(s2) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s2.x(), s2.y(), s2.z())))
            recursiveGrowth(conqueredBoxes, g, s2, target);
    if (isBounded(s3, g))
        if (g.boxHasPossibleTarget(s3.x(), s3.y(), s3.z(), target) && conqueredBoxes.find(s3) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s3.x(), s3.y(), s3.z())))
            recursiveGrowth(conqueredBoxes, g, s3, target);
    if (isBounded(s4, g))
        if (g.boxHasPossibleTarget(s4.x(), s4.y(), s4.z(), target) && conqueredBoxes.find(s4) == conqueredBoxes.end() && !(g.isDefinitiveTarget(s4.x(), s4.y(), s4.z())))
            recursiveGrowth(conqueredBoxes, g, s4, target);
}

std::set<Pointi> Reconstruction::growPiece(const IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    Pointi base = getBase(g, startingBox, target);
    std::set<Pointi> conqueredBoxes;
    recursiveGrowth(conqueredBoxes, g, base, target);
    return conqueredBoxes;
}

void Reconstruction::setDefinitivePiece(IrregularGrid& g, const std::set<Pointi>& piece, const Vec3& target) {
    for (Pointi box : piece){
        g.setDefinitiveTarget(box.x(), box.y(), box.z(), target);
    }
}

/// Brute Force
void Reconstruction::generateAllPossibleTargets(const IrregularGrid& g) {
    std::vector<Pointi> subdivisions;
    std::vector< std::vector<int> > possibleTargets;
    for (unsigned int i = 0; i < g.getResolutionX()-1; i++){
        for (unsigned int j = 0; j < g.getResolutionY()-1; j++){
            for (unsigned int k = 0; k < g.getResolutionZ()-1; k++){
                if (g.getNumberPossibleTargets(i,j,k) > 1){
                    subdivisions.push_back(Pointi(i,j,k));
                    std::vector<Vec3> tmpv = g.getPossibleTargets(i,j,k);
                    std::vector<int> tmpi;

                    for (Vec3 n: tmpv){
                        bool fl = false;
                        for (int l = 0; l < 6 && fl == false; l++){
                            if (n == XYZ[l]) {
                                tmpi.push_back(l);
                                fl = true;
                            }
                        }
                        assert(fl);
                    }
                    possibleTargets.push_back(tmpi);
                }
            }
        }
    }

    long int numberPossibleCombinations = 1;
    for (unsigned int i = 0; i < possibleTargets.size(); i++){
        numberPossibleCombinations *= possibleTargets[i].size();
    }
    std::cerr << "Number possible Combinations: "<< numberPossibleCombinations << "\n";

    /*std::vector< std::vector<int> > allPossibleCombinations(numberPossibleCombinations);
    std::vector<int> indices(subdivisions.size(), 0);
    int actualIndex = 0;
    for (long int i = 0; i < numberPossibleCombinations; i++){
        std::vector<int> actualCombination(subdivisions.size());
        for (unsigned int j = 0; j < actualCombination.size(); j++){
            actualCombination[j] = possibleTargets[j][indices[j]];
        }
        allPossibleCombinations[i] = actualCombination;

        //aggiornamento indici

    }*/

}

IGLInterface::IGLMesh Reconstruction::getSurfaceOfPiece(const std::set<Pointi>& boxes, const IrregularGrid& g) {
    std::vector< std::array<Pointd, 4> > quads;
    std::vector<Pointd> points;
    std::map<Pointd, int> indices_points;
    int index = 0;
    for (Pointi box : boxes){
        if (boxes.find(Pointi(box.x()-1, box.y(), box.z())) == boxes.end()){
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x(), box.y()  , box.z()  );
            quad[1] = g.getPoint(box.x(), box.y()  , box.z()+1);
            quad[2] = g.getPoint(box.x(), box.y()+1, box.z()+1);
            quad[3] = g.getPoint(box.x(), box.y()+1, box.z()  );
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
        if (boxes.find(Pointi(box.x(), box.y()-1, box.z())) == boxes.end()) {
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x()  , box.y(), box.z()  );
            quad[1] = g.getPoint(box.x()+1, box.y(), box.z()  );
            quad[2] = g.getPoint(box.x()+1, box.y(), box.z()+1);
            quad[3] = g.getPoint(box.x()  , box.y(), box.z()+1);
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
        if (boxes.find(Pointi(box.x(), box.y(), box.z()-1)) == boxes.end()) {
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x()  , box.y()  , box.z());
            quad[1] = g.getPoint(box.x()  , box.y()+1, box.z());
            quad[2] = g.getPoint(box.x()+1, box.y()+1, box.z());
            quad[3] = g.getPoint(box.x()+1, box.y()  , box.z());
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
        if (boxes.find(Pointi(box.x()+1, box.y(), box.z())) == boxes.end()) {
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x()+1, box.y()  , box.z()  );
            quad[1] = g.getPoint(box.x()+1, box.y()+1, box.z()  );
            quad[2] = g.getPoint(box.x()+1, box.y()+1, box.z()+1);
            quad[3] = g.getPoint(box.x()+1, box.y()  , box.z()+1);
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
        if (boxes.find(Pointi(box.x(), box.y()+1, box.z())) == boxes.end()) {
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x()  , box.y()+1, box.z()  );
            quad[1] = g.getPoint(box.x()  , box.y()+1, box.z()+1);
            quad[2] = g.getPoint(box.x()+1, box.y()+1, box.z()+1);
            quad[3] = g.getPoint(box.x()+1, box.y()+1, box.z()  );
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
        if (boxes.find(Pointi(box.x(), box.y(), box.z()+1)) == boxes.end()) {
            std::array<Pointd, 4> quad;
            quad[0] = g.getPoint(box.x()  , box.y()  , box.z()+1);
            quad[1] = g.getPoint(box.x()+1, box.y()  , box.z()+1);
            quad[2] = g.getPoint(box.x()+1, box.y()+1, box.z()+1);
            quad[3] = g.getPoint(box.x()  , box.y()+1, box.z()+1);
            for (int i = 0; i < 4; i++){
                if (indices_points.find(quad[i]) == indices_points.end()){
                    indices_points[quad[i]] = index++;
                    points.push_back(quad[i]);
                }
            }
            quads.push_back(quad);
        }
    }
    IGLInterface::IGLMesh mesh;
    for (Pointd point : points){
        mesh.addVertex(point.x(), point.y(), point.z());
    }
    for (std::array<Pointd, 4> quad : quads){
        assert(indices_points.find(quad[0]) != indices_points.end());
        assert(indices_points.find(quad[1]) != indices_points.end());
        assert(indices_points.find(quad[2]) != indices_points.end());
        assert(indices_points.find(quad[3]) != indices_points.end());
        mesh.addFace(indices_points[quad[0]], indices_points[quad[1]], indices_points[quad[2]]);
        mesh.addFace(indices_points[quad[2]], indices_points[quad[3]], indices_points[quad[0]]);
    }
    mesh.updateVertexAndFaceNormals();
    return mesh;
}

std::vector<IGLInterface::IGLMesh> Reconstruction::getPieces(IrregularGrid& g, std::vector<Vec3> &targets) {
    std::vector<IGLInterface::IGLMesh> meshes;
    for (unsigned int i = 0; i < g.getResolutionX()-1; i++) {
        for (unsigned int j = 0; j < g.getResolutionY()-1; j++) {
            for (unsigned int k = 0; k < g.getResolutionZ()-1; k++) {
                if (!(g.isDefinitiveTarget(i,j,k)) && g.getNumberPossibleTargets(i,j,k) > 0) {
                    std::vector<Vec3> possibleTargets = g.getPossibleTargets(i,j,k);
                    std::vector<std::set<Pointi> > pieces(possibleTargets.size());
                    double maxVol = -1;
                    int maxVolId = -1;
                    for (unsigned int l = 0; l < possibleTargets.size(); l++){
                        Timer t("Recursive Growth");
                        pieces[l] = growPiece(g, Pointi(i,j,k), possibleTargets[l]);
                        t.stopAndPrint();
                        double volume = 0;
                        for (Pointi box : pieces[l]){
                            volume += g.getVolumeOfBox(box.x(), box.y(), box.z());
                        }
                        std::cout << "Volume: " << volume << "\n";
                        if (volume > maxVol){
                            maxVol = volume;
                            maxVolId = l;
                        }
                    }
                    std::set<Pointi> finalPiece = pieces[maxVolId];
                    Vec3 finalTarget = possibleTargets[maxVolId];
                    for (Pointi box : finalPiece){
                        g.setDefinitiveTarget(box.x(), box.y(), box.z(), finalTarget);
                    }
                    IGLInterface::IGLMesh mesh = getSurfaceOfPiece(finalPiece, g);
                    meshes.push_back(mesh);
                    targets.push_back(finalTarget);
                }
            }
        }
    }
    return meshes;
}


void Reconstruction::booleanOperations(HeightfieldsList &heightfields, IGLInterface::SimpleIGLMesh &baseComplex, HeightfieldsList &polycubes) {
    Timer timer("Boolean Operations");

    for (unsigned int i = 0; i < polycubes.getNumHeightfields(); i++){
        IGLInterface::SimpleIGLMesh intersection;
        IGLInterface::SimpleIGLMesh polycube = polycubes.getHeightfield(i);
        IGLInterface::SimpleIGLMesh::intersection(intersection, baseComplex, polycube);
        if (intersection.getNumberVertices() > 0) {
            IGLInterface::SimpleIGLMesh::difference(baseComplex, baseComplex, polycube);
            heightfields.addHeightfield(intersection, polycubes.getTarget(i));
        }
        else {
            polycubes.removeHeightfield(i);
            i--;
        }
    }

    timer.stopAndPrint();
}

bool Reconstruction::boxesIntersect(const Box3D& b1, const Box3D& b2) {
    if (b1.getMaxX() <= b2.getMinX()) return false; // a is left of b
    if (b1.getMinX() >= b2.getMaxX()) return false; // a is right of b
    if (b1.getMaxY() <= b2.getMinY()) return false; // a is above b
    if (b1.getMinY() >= b2.getMaxY()) return false; // a is below b
    if (b1.getMaxZ() <= b2.getMinZ()) return false; // a is behind b
    if (b1.getMinZ() >= b2.getMaxZ()) return false; // a is in front b
    return true; //boxes overlap
}

bool Reconstruction::isDangerousIntersection(const Box3D& b1, const Box3D& b2, const CGALInterface::AABBTree &tree, bool checkMeshes) {
    Vec3 target2 = b2.getTarget();
    BoundingBox bb = b1;
    for (unsigned int t = 0; t < 6; t++){
        if (t < 3){
            if (target2 == XYZ[t]){ //+x
                //if (!(b1.min()[t] < b2.min()[t] && b1.max()[t] > b2.max()[t])){
                if (b1.max()[t] < b2.max()[t]){
                    bool b = false;
                    for (unsigned int i = 0; i < 3; i++){
                        if (i != t){
                            if (b1.min()[i] > b2.min()[i])
                                b = true;
                            if (b1.max()[i] < b2.max()[i])
                                b = true;
                        }
                    }
                    if (b){
                        //check if bb is empty
                        bb.min()[t]= b1.max()[t]-EPSILON;
                        for (unsigned int i = 0; i < 3; i++){
                            if (i != t){
                                bb.min()[i] = std::max(b1.min()[i], b2.min()[i]);
                                bb.max()[i] = std::min(b1.max()[i], b2.max()[i]);
                                assert(bb.min()[i] < bb.max()[i]);
                            }
                        }
                        if (tree.getNumberIntersectedPrimitives(bb) > 0){
                            if (!checkMeshes){
                                return true;
                            }
                            else {
                                IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                                if (intersection.getNumberVertices() != 0){
                                    ////
                                    //intersection.saveOnObj("int.obj");
                                    ////
                                    return true;
                                }
                            }
                        }
                    }
                }
                //if (!(b1.getMinY() < b2.getMinY() && b1.getMinZ() < b2.getMinZ() && b1.getMaxY() > b2.getMaxY() && b1.getMaxZ() > b2.getMaxZ())){

                //}
            }
        }
        else {
            if (target2 == XYZ[t]){ //-x
                unsigned int ot = t-3;
                //if (b1.min()[ot] < base && b1.max()[ot] > base && !(b1.min()[ot] < b2.min()[ot])){
                if(b1.min()[ot] > b2.min()[ot]){
                    bool b = false;
                    for (unsigned int i = 0; i < 3; i++){
                        if (i != ot){
                            if (b1.min()[i] > b2.min()[i])
                                b = true;
                            if (b1.max()[i] < b2.max()[i])
                                b = true;
                        }
                    }
                    if (b){
                        //check if bb is empty
                        bb.max()[ot] = (b1.min()[ot]+EPSILON);
                        for (unsigned int i = 0; i < 3; i++){
                            if (i != ot){
                                bb.min()[i] = std::max(b1.min()[i], b2.min()[i]);
                                bb.max()[i] = std::min(b1.max()[i], b2.max()[i]);
                                assert(bb.min()[i] < bb.max()[i]);
                            }
                        }
                        if (tree.getNumberIntersectedPrimitives(bb) > 0){
                            if (!checkMeshes){
                                return true;
                            }
                            else {
                                IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                                if (intersection.getNumberVertices() != 0){
                                    ////
                                    //intersection.saveOnObj("int.obj");
                                    ////
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
    /*if (target2 == XYZ[0]){ //+x
        base = b2.getMinX();
        if (b1.getMinX() < base && b1.getMaxX() > base){
            if (!(b1.getMinY() < b2.getMinY() && b1.getMinZ() < b2.getMinZ() && b1.getMaxY() > b2.getMaxY() && b1.getMaxZ() > b2.getMaxZ())){
                //check if bb is empty
                bb.setMinX(b1.getMaxX()-EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else if (target2 == XYZ[1]){ //+y
        base = b2.getMinY();
        if (b1.getMinY() < base && b1.getMaxY() > base){
            if (!(b1.getMinX() < b2.getMinX() && b1.getMinZ() < b2.getMinZ() && b1.getMaxX() > b2.getMaxX() && b1.getMaxZ() > b2.getMaxZ())){
                bb.setMinY(b1.getMaxY()-EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else if (target2 == XYZ[2]){ //+z
        base = b2.getMinZ();
        if (b1.getMinZ() < base && b1.getMaxZ() > base){
            if (!(b1.getMinX() < b2.getMinX() && b1.getMinY() < b2.getMinY() && b1.getMaxX() > b2.getMaxX() && b1.getMaxY() > b2.getMaxY())){
                bb.setMinZ(b1.getMaxZ()-EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else if (target2 == XYZ[3]){ //-x
        base = b2.getMaxX();
        if (b1.getMinX() < base && b1.getMaxX() > base){
            if (!(b1.getMinY() < b2.getMinY() && b1.getMinZ() < b2.getMinZ() && b1.getMaxY() > b2.getMaxY() && b1.getMaxZ() > b2.getMaxZ())){
                bb.setMaxX(b1.getMinX()+EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else if (target2 == XYZ[4]){ //-y
        base = b2.getMaxY();
        if (b1.getMinY() < base && b1.getMaxY() > base){
            if (!(b1.getMinX() < b2.getMinX() && b1.getMinZ() < b2.getMinZ() && b1.getMaxX() > b2.getMaxX() && b1.getMaxZ() > b2.getMaxZ())){
                bb.setMaxY(b1.getMinY()+EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else if (target2 == XYZ[5]){ //-z
        base = b2.getMaxZ();
        if (b1.getMinZ() < base && b1.getMaxZ() > base){
            if (!(b1.getMinX() < b2.getMinX() && b1.getMinY() < b2.getMinY() && b1.getMaxX() > b2.getMaxX() && b1.getMaxY() > b2.getMaxY())){
                bb.setMaxZ(b1.getMinZ()+EPSILON);
                if (tree.getNumberIntersectedPrimitives(bb) > 0){
                    if (!checkMeshes){
                        return true;
                    }
                    else {
                        IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                        if (intersection.getNumberVertices() != 0){
                            ////
                            intersection.saveOnObj("int.obj");
                            ////
                            return true;
                        }
                    }
                }
            }
        }
    } else
        assert(0);
    return false;*/
}

double Reconstruction::getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3) {
    Box3D b4;
    b3.setColor(b2.getColor());
    b3.setTarget(b2.getTarget());
    Vec3 target = b2.getTarget();
    for (unsigned int t = 0; t < 6; t++){
        if (target == XYZ[t]){
            if (t < 3){
                assert(b1.max()[t] < b2.max()[t]);
                b3.min()[t] = b1.max()[t];
                b3.max()[t] = b2.max()[t];
                b4.min()[t] = b2.min()[t];
                b4.max()[t] = b1.max()[t];
                for (unsigned u = 0; u < 3; u++){
                    if (u != t){
                        b3.min()[u] = b4.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                        b3.max()[u] = b4.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                        assert(b3.min()[u] < b3.max()[u]);
                    }
                }
            }
            else if (t >= 3){
                unsigned int i = t-3;
                b1.getIGLMesh().saveOnObj("b1.obj");
                b2.getIGLMesh().saveOnObj("b2.obj");
                assert(b1.min()[i] > b2.min()[i]);
                b3.min()[i] = b2.min()[i];
                b3.max()[i] = b1.min()[i];
                b4.min()[i] = b1.min()[i];
                b4.max()[i] = b2.max()[i];
                for (unsigned int u = 0; u < 3; u++){
                    if (u != i){
                        b3.min()[u] = b4.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                        b3.max()[u] = b4.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                        assert(b3.min()[u] < b3.max()[u]);
                    }
                }

            }
        }
    }
    return b4.getVolume();
}

void Reconstruction::splitBox(const Box3D& b1, Box3D& b2, Box3D & b3) {
    getSplits(b1, b2, b3);
    //
    Box3D b3tmp = b3;
    for (unsigned int i = 0; i < 6; i++){
        if (b3tmp[i] == b2[i]){
            if (i < 3)
                b3tmp[i]-=EPSILON;
            else
                b3tmp[i]+=EPSILON;
        }
    }
    b3tmp.generatePiece();
    //
    b3.generatePiece();
    IGLInterface::SimpleIGLMesh oldBox = b2.getIGLMesh();
    oldBox = IGLInterface::SimpleIGLMesh::difference(oldBox, b1.getIGLMesh());
    IGLInterface::SimpleIGLMesh tmp = oldBox;
    tmp = IGLInterface::SimpleIGLMesh::intersection(tmp, b3.getIGLMesh());
    if (tmp.getNumberVertices() > 0){
        oldBox = IGLInterface::SimpleIGLMesh::difference(oldBox, b3tmp.getIGLMesh());
        b2.setIGLMesh(oldBox);
        BoundingBox newBBb2 = oldBox.getBoundingBox();
        b2.setMin(newBBb2.min());
        b2.setMax(newBBb2.max());
        b3.setIGLMesh(tmp);
        BoundingBox newBBb3 = tmp.getBoundingBox();
        b3.setMin(newBBb3.min());
        b3.setMax(newBBb3.max());
    }
    else
        b3.min() = b3.max() = Pointd();
}

double Reconstruction::minimumSplit(const Box3D &b1, const Box3D &b2){
    Box3D b3;
    double volumeb4 = getSplits(b1, b2, b3);
    double volumeb2 = b2.getVolume();
    double volumeb3 = b3.getVolume();
    double remainingSplit = volumeb2 - volumeb3 - volumeb4;
    assert (remainingSplit >= 0);
    return std::min(remainingSplit, volumeb3);
}

Array2D<int> Reconstruction::getOrdering(BoxList& bl, const Dcel& d) {
    CGALInterface::AABBTree tree(d);
    bl.calculateTrianglesCovered(tree);
    bl.sortByTrianglesCovered();
    bl.setIds();
    std::cerr << "Triangles Covered: \n";
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::cerr << bl.getBox(i).getTrianglesCovered() << "; ";
    }
    std::cerr << "\n";
    std::vector<std::vector<unsigned int> > loops;
    DirectedGraph g(bl.getNumberBoxes());
    //
    //d.saveOnObjFile("tmp.obj");
    //
    for (unsigned int i = 0; i < bl.getNumberBoxes()-1; i++){
        Box3D b1 = bl.getBox(i);
        for (unsigned int j = i+1; j < bl.getNumberBoxes(); j++){
            Box3D b2 = bl.getBox(j);
            if (boxesIntersect(b1,b2)){
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                if (isDangerousIntersection(b1, b2, tree)){
                    g.addEdge(i,j);
                    std::cerr << i << " -> " << j << "\n";
                }
                if (isDangerousIntersection(b2, b1, tree)){
                    g.addEdge(j,i);
                    std::cerr << j << " -> " << i << "\n";
                }
            }
        }
    }
    ///Detect and delete cycles on graph (modifying bl)

    do {
        g.getLoops(loops);
        std::cerr << "Number loops: " << loops.size() << "\n";
        if (loops.size() > 0){ // I need to modify bl

            std::map<std::pair<unsigned int, unsigned int>, int> arcs;
            for (std::vector<unsigned int> loop : loops){
                unsigned int node = 0;
                for (node = 0; node < loop.size()-1; node++){
                    std::cerr << loop[node] << " -> ";
                    std::pair<unsigned int, unsigned int> arc(loop[node], loop[node+1]);
                    std::map<std::pair<unsigned int, unsigned int>, int>::iterator it = arcs.find(arc);
                    if (it == arcs.end())
                        arcs[arc] = 1;
                    else
                        arcs[arc]++;
                }
                std::cerr << loop[node] << " \n";
            }
            std::multimap<int,std::pair<unsigned int, unsigned int> > rev = Common::flipMap(arcs);

            for (std::multimap<int,std::pair<unsigned int, unsigned int> >::reverse_iterator it = rev.rbegin(); it != rev.rend(); ++it){
                std::cerr << "K : " << (*it).first << "; V: (" << (*it).second.first << " -> " << (*it).second.second << ")\n";
            }

            std::multimap<int,std::pair<unsigned int, unsigned int> >::reverse_iterator it = rev.rbegin();

            unsigned int size = rev.count((*it).first); // number of arcs that belongs with the maximum number of loops
            std::pair<unsigned int, unsigned int> arcToRemove;
            if (size == 1){ // if I remove this arc, I will remove the maximum number of loops
                arcToRemove = (*it).second;
                std::cerr << "Eliminate Arc: " << arcToRemove.first << " -> " << arcToRemove.second << "\n";
            }
            else { // I need to choose wich arc I want to eliminate -> maximum split
                std::cerr << "Eliminate Arcs: \n";
                std::vector<std::pair<unsigned int, unsigned int> > candidateArcs;
                for (auto i=rev.equal_range((*it).first).first; i!=rev.equal_range((*it).first).second; ++i){
                    std::pair<unsigned int, unsigned int> arc = (*i).second;
                    std::cerr << "\t" << arc.first << " -> " << arc.second << "\n";
                    candidateArcs.push_back(arc);
                }
                double volmax = minimumSplit(bl.getBox(candidateArcs[0].first), bl.getBox(candidateArcs[0].second));
                int maxarc = 0;

                for (unsigned int i = 1; i < candidateArcs.size(); i++) {
                    std::pair<unsigned int, unsigned int> arc = candidateArcs[i];
                    double tmp = minimumSplit(bl.getBox(arc.first), bl.getBox(arc.second));
                    if (tmp > volmax){
                        volmax = tmp;
                        maxarc = i;
                    }
                }
                arcToRemove = candidateArcs[maxarc];
            }
            // now I can remove "arcToRemove"
            Box3D b1 = bl.getBox(arcToRemove.first), b2 = bl.getBox(arcToRemove.second), b3;
            splitBox(b1, b2, b3);
            if (!(b3.min() == Pointd() && b3.max() == Pointd())){
                b3.setId(bl.getNumberBoxes());
                b3.setTrianglesCovered(tree.getNumberIntersectedPrimitives(b3));
                b2.setTrianglesCovered(b2.getTrianglesCovered()-b3.getTrianglesCovered());
                bl.setBox(b2.getId(), b2);
                bl.addBox(b3);
                ///
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                //b3.getIGLMesh().saveOnObj("b3.obj");
                ///
                g.removeEdge(arcToRemove.first, arcToRemove.second);
                g.removeEdge(arcToRemove.second, arcToRemove.first);
                std::vector<unsigned int> incomingb2 = g.getIncomingNodes(b2.getId());
                std::vector<unsigned int> outgoingb2 = g.getOutgoingNodes(b2.getId());
                g.deleteAllIncomingNodes(b2.getId());
                g.deleteAllOutgoingNodes(b2.getId());
                unsigned int tmp = g.addNode();
                assert(tmp == (unsigned int)b3.getId());
                for (unsigned int incoming : incomingb2){
                    Box3D other = bl.getBox(incoming);
                    ///
                    //other.getIGLMesh().saveOnObj("checkother.obj");
                    //b2.getIGLMesh().saveOnObj("checkb2.obj");
                    //b3.getIGLMesh().saveOnObj("checkb3.obj");
                    ///
                    if (boxesIntersect(other,b2)){
                        if (isDangerousIntersection(other, b2, tree, true)){
                            g.addEdge(incoming,b2.getId());
                            std::cerr << incoming << " -> " << b2.getId() << "\n";
                        }
                    }
                    if (boxesIntersect(other,b3)){
                        if (isDangerousIntersection(other, b3, tree, true)){
                            g.addEdge(incoming,b3.getId());
                            std::cerr << incoming << " -> " << b3.getId() << "\n";
                        }
                    }
                }
                for (unsigned int outgoing : outgoingb2){
                    Box3D other = bl.getBox(outgoing);
                    ///
                    //other.getIGLMesh().saveOnObj("checkother.obj");
                    //b2.getIGLMesh().saveOnObj("checkb2.obj");
                    //b3.getIGLMesh().saveOnObj("checkb3.obj");
                    ///
                    if (boxesIntersect(b2, other)){
                        if (isDangerousIntersection(b2, other, tree, true)){
                            g.addEdge(b2.getId(), outgoing);
                            std::cerr << b2.getId() << " -> " << outgoing << "\n";
                        }
                    }
                    if (boxesIntersect(b3, other)){
                        if (isDangerousIntersection(b3, other, tree, true)){
                            g.addEdge(b3.getId(), outgoing);
                            std::cerr << b3.getId() << " -> " << outgoing << "\n";
                        }
                    }
                }
            }
            else {
                g.removeEdge(arcToRemove.first, arcToRemove.second);
                g.removeEdge(arcToRemove.second, arcToRemove.first);
            }
        }
    }while (loops.size() > 0);

    //resorting and map old with new graph
    //this is necessary because small indices prevails if
    //no arcs are on the graph
    bl.sortByTrianglesCovered();
    std::map<unsigned int, unsigned int> mapping;
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        mapping[bl.getBox(i).getId()] = i;
    }
    DirectedGraph newGraph(bl.getNumberBoxes());
    for (unsigned int node = 0; node < g.size(); ++node){
        std::vector<unsigned int> outgoing = g.getOutgoingNodes(node);
        for (unsigned int o : outgoing){
            newGraph.addEdge(mapping[node], mapping[o]);
        }
    }

    //works only if graph has no cycles
    newGraph.getLoops(loops);
    assert(loops.size() == 0);
    Array2D<int> ordering(bl.getNumberBoxes(), bl.getNumberBoxes(), -1); // true -> "<", false -> ">=", undefined -> "-1"
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::set<unsigned int> visited;
        newGraph.visit(visited, i);
        for (unsigned int node : visited){ // i must be > than all nodes in visited
            ordering(node,i) = true;
            ordering(i,node) = false;
        }
    }
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        for (unsigned int j = 0; j < i; j++){
            if (ordering(i,j) == -1) {
                ordering(i,j) = false;
                ordering(j,i) = true;
                for (unsigned int k = 0; k < bl.getNumberBoxes(); k++){
                    //j now is the row
                    if (ordering(j,k) == false){
                        assert(ordering(i,k) == 0 || ordering(i,k) == -1);
                        ordering(i,k) = false;
                        ordering(k,i) = true;
                    }
                }
            }
        }
    }

    return ordering;

}
