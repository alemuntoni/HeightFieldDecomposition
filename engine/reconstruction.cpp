#include "reconstruction.h"

#include "common.h"
#include "common/timer.h"
#include "cgal/aabbtree.h"

void Reconstruction::compactSet(std::set<double>& set, double epsilon) {
    std::set<double>::iterator it = set.begin();
    std::set<double> toDelete;
    double last = *it;
    ++it;
    double actual;
    for (; it != set.end();++it){
        actual = *it;
        if (epsilonEqual(last, actual, epsilon)){
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
    assert(aabb.isInside(Pointd(0,0,0)));
    for (unsigned int sol = 0; sol < solutions.getNumberBoxes(); ++sol){
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
                            grid.addPossibleTarget(i,j,k, b.getTarget());
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
std::set<Pointi> Reconstruction::findConnectedComponent(IrregularGrid& g, const Pointi& startingBox, const Vec3& target) {
    assert(g.boxHasPossibleTarget(startingBox.x(), startingBox.y(), startingBox.z(), target));
    assert(g.flag(startingBox.x(), startingBox.y(), startingBox.z()) == 0);
    g.flag(startingBox.x(), startingBox.y(), startingBox.z()) = 1;


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
                        pieces[l] = growPiece(g, Pointi(i,j,k), possibleTargets[l]);
                        double volume = 0;
                        for (Pointi box : pieces[l]){
                            volume += g.getVolumeOfBox(box.x(), box.y(), box.z());
                        }
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
