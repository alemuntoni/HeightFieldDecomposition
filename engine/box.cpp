#include "box.h"
#include "common.h"
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>

using namespace cg3;

Box3D::Box3D(): visible(true), splitted(false){
    rotation = Eigen::Matrix3d::Identity();
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const Pointd& c1, const Pointd& c2, const Pointd& c3, const Color c) : BoundingBox(min, max), c1(c1), c2(c2), c3(c3), color(c), visible(true), splitted(false){
    rotation = Eigen::Matrix3d::Identity();
}

Box3D::Box3D(const Pointd& min, const Pointd& max, const Color c) :BoundingBox(min, max), color(c), visible(true), splitted(false){
    rotation = Eigen::Matrix3d::Identity();
}

void Box3D::setConstraint1(const Pointd& p) {
    c1 = p;
}

void Box3D::setConstraint2(const Pointd& p) {
    c2 = p;
}

void Box3D::setConstraint3(const Pointd& p) {
    c3 = p;
}

void Box3D::setRotationMatrix(const Eigen::Matrix3d& rot) {
    rotation = rot;
}

void Box3D::getRotatedExtremes(std::vector<Pointd>& v) const {
    v.resize(8);
    Pointd p = minCoord;
    p.rotate(rotation);
    v[0] = p;
    p.set(maxCoord.x(), minCoord.y(), minCoord.z());
    p.rotate(rotation);
    v[1] = p;
    p.set(maxCoord.x(), minCoord.y(), maxCoord.z());
    p.rotate(rotation);
    v[2] = p;
    p.set(minCoord.x(), minCoord.y(), maxCoord.z());
    p.rotate(rotation);
    v[3] = p;
    p.set(minCoord.x(), maxCoord.y(), minCoord.z());
    p.rotate(rotation);
    v[4] = p;
    p.set(maxCoord.x(), maxCoord.y(), minCoord.z());
    p.rotate(rotation);
    v[5] = p;
    p = maxCoord;
    p.rotate(rotation);
    v[6] = p;
    p.set(minCoord.x(), maxCoord.y(), maxCoord.z());
    p.rotate(rotation);
    v[7] = p;
}

SimpleEigenMesh Box3D::calculateEigenMesh(double minimumEdge) const {
    SimpleEigenMesh box;
    if (minimumEdge <= 0) {
        std::vector<Pointd> extremes;
        getRotatedExtremes(extremes);
        box.resizeVertices(8);
        for (unsigned int i = 0; i < 8; i++){
            box.setVertex(i, extremes[i].x(), extremes[i].y(), extremes[i].z());
        }
        box.resizeFaces(12);
        box.setFace(0, 0,1,2);
        box.setFace(1, 0,2,3);
        box.setFace(2, 2,1,5);
        box.setFace(3, 2,5,6);
        box.setFace(4, 5,1,0);
        box.setFace(5, 5,0,4);
        box.setFace(6, 6,5,4);
        box.setFace(7, 6,4,7);
        box.setFace(8, 7,4,0);
        box.setFace(9, 7,0,3);
        box.setFace(10, 7,3,2);
        box.setFace(11, 7,2,6);
    }
    else {
        unsigned int nSubdX, nSubdY, nSubdZ;
        //minimumEdge /= std::sqrt(2.f);
        nSubdX = getLengthX() / minimumEdge; nSubdX++;
        nSubdY = getLengthY() / minimumEdge; nSubdY++;
        nSubdZ = getLengthZ() / minimumEdge; nSubdZ++;
        double edgeLengthX = getLengthX() / nSubdX;
        double edgeLengthY = getLengthY() / nSubdY;
        double edgeLengthZ = getLengthZ() / nSubdZ;
        //creation vertices
        std::map<Pointi, Pointd> vertices;
        double x, y, z;
        unsigned int i, j , k;

        // fix z - k = 0;
        k = 0; z = getMinZ();
        x = getMinX();
        for (i = 0; i <= nSubdX; i++){
            y = getMinY();
            for (j = 0; j <= nSubdY; j++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                y+=edgeLengthY;
            }
            x+=edgeLengthX;
        }
        // fix z - k = nSubdZ;
        k = nSubdZ; z = getMaxZ();
        x = getMinX();
        for (i = 0; i <= nSubdX; i++){
            y = getMinY();
            for (j = 0; j <= nSubdY; j++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                y+=edgeLengthY;
            }
            x+=edgeLengthX;
        }
        // fix y - j = 0;
        j = 0; y = getMinY();
        x = getMinX();
        for (i = 0; i <= nSubdX; i++){
            z = getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            x+=edgeLengthX;
        }
        // fix y - j = nSubdY;
        j = nSubdY; y = getMaxY();
        x = getMinX();
        for (i = 0; i <= nSubdX; i++){
            z = getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            x+=edgeLengthX;
        }
        // fix x - i = 0;
        i = 0; x = getMinX();
        y = getMinY();
        for (j = 0; j <= nSubdY; j++){
            z = getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            y+=edgeLengthY;
        }
        // fix x - i = nSubdX;
        i = nSubdX; x = getMaxX();
        y = getMinY();
        for (j = 0; j <= nSubdY; j++){
            z = getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            y+=edgeLengthY;
        }

        std::map<Pointi, int> indices;
        int index = 0;
        box.resizeVertices(vertices.size());
        for (std::pair<Pointi, Pointd> pair : vertices) {
            indices[pair.first] = index;
            box.setVertex(index, pair.second.x(), pair.second.y(), pair.second.z());
            index++;

        }

        //triangles
        // fix z - k = 0;
        k = 0;
        for (i = 0; i < nSubdX; i++){
            for (j = 0; j < nSubdY; j++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j+1,k);
                Pointi pi4(i,j+1,k);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix z - k = nSubdZ;
        k = nSubdZ;
        for (i = 0; i < nSubdX; i++){
            for (j = 0; j < nSubdY; j++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j+1,k);
                Pointi pi4(i,j+1,k);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
        // fix y - j = 0;
        j = 0;
        for (i = 0; i < nSubdX; i++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
        // fix y - j = nSubdY;
        j = nSubdY;
        for (i = 0; i < nSubdX; i++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix x - i = 0;
        i = 0;
        for (j = 0; j < nSubdY; j++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i,j+1,k);
                Pointi pi3(i,j+1,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix x - i = nSubdX;
        i = nSubdX;
        for (j = 0; j < nSubdY; j++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i,j+1,k);
                Pointi pi3(i,j+1,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
        box.rotate(rotation);

    }
    return box;
}

SimpleEigenMesh Box3D::getEigenMesh() const {
    return piece;
}

void Box3D::setEigenMesh(const SimpleEigenMesh& piece) {
    this->piece = piece;
}

void Box3D::generateEigenMesh(double minimumEdge) {
    piece = calculateEigenMesh(minimumEdge);
}

void Box3D::draw() const {
    #ifdef CG3_VIEWER_DEFINED
    if (visible){
        Pointd c1 = this->c1, c2 = this->c2, c3 = this->c3;
        c1.rotate(rotation);
        c2.rotate(rotation);
        c3.rotate(rotation);
        cg3::viewer::drawBox(minCoord, maxCoord, color);

        /*cylinder(min, Pointd(max.x(), min.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), max.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), Pointd(min.x(), max.y(), min.z()), 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), min, 0.05, 0.05, color);

        cylinder(Pointd(min.x(), min.y(), max.z()), Pointd(max.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), max.z()), max, 0.05, 0.05, color);
        cylinder(max, Pointd(min.x(), max.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), max.z()), Pointd(min.x(), min.y(), max.z()), 0.05, 0.05, color);

        cylinder(min, Pointd(min.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), min.y(), min.z()), Pointd(max.x(), min.y(), max.z()), 0.05, 0.05, color);
        cylinder(Pointd(max.x(), max.y(), min.z()), max, 0.05, 0.05, color);
        cylinder(Pointd(min.x(), max.y(), min.z()), Pointd(min.x(), max.y(), max.z()), 0.05, 0.05, color);*/

        cg3::viewer::drawSphere(c1, 0.15, QColor(255,0,255));
        cg3::viewer::drawSphere(c2, 0.15, QColor(255,0,255));
        cg3::viewer::drawSphere(c3, 0.15, QColor(255,0,255));
    }
    #endif
}

Pointd Box3D::sceneCenter() const {
    return (maxCoord+minCoord)/2;
}

double Box3D::sceneRadius() const {
    //return -1;
    return diag();
}

bool Box3D::isVisible() const {
    return visible;
}

void Box3D::setVisible(bool b) {
    visible = b;
}

void Box3D::serialize(std::ofstream& binaryFile) const {
    BoundingBox::serialize(binaryFile);
    serializeObjectAttributes("box", binaryFile, c1, c2, c3, color, visible, target, rotation, id, piece, trianglesCovered);
}

void Box3D::deserialize(std::ifstream& binaryFile) {
    BoundingBox::deserialize(binaryFile);
    deserializeObjectAttributes("box", binaryFile, c1, c2, c3, color, visible, target, rotation, id, piece, trianglesCovered);
}

const Vec3& Box3D::getTarget() const {
    return target;
}

void Box3D::setTarget(const Vec3& value) {
    target = value;
}

Vec3 Box3D::getRotatedTarget() const {
    Vec3 r = target;
    r.rotate(rotation);
    r.normalize();
    return r;
}

bool Box3D::operator <(const Box3D& other) const {
    return id < other.id;
}

bool Box3D::isSplitted() const
{
    return splitted;
}

void Box3D::setSplitted(bool value)
{
    splitted = value;
}

std::string Box3D::typeSplitToString(const Box3D::Split& s) {
    std::string out;
    if (s.hs & LOW){
        out += "Low; ";
    }
    if (s.hs & MIDDLE){
        out += "Middle; ";
    }
    if (s.hs & HIGH){
        out += "High; ";
    }
    if (s.hs & ALL){
        out += "All; ";
    }
    if (s.ts & ONE_CORNER) {
        out += "One Corner.";
    }
    if (s.ts & TWO_CORNERS) {
        out += "Two Corners.";
    }
    if (s.ts & ONE_EDGE) {
        out += "One Edge.";
    }
    if (s.ts & TWO_EDGES) {
        out += "Two Edges.";
    }
    if (s.ts & TOTALLY_INSIDE) {
        out += "Totally Inside.";
    }
    if (s.ts & TOTALLY_OUTSIDE) {
        out += "Totally Outside.";
    }
    return out;
}

Box3D::Split Box3D::getSplit(const Box& other) {
    unsigned int ind = getTargetIndex();
    unsigned int oi1 = (ind+1)%3;
    unsigned int oi2 = (oi1+1)%3;
    Split split;
    if (other.min()[ind] <= minCoord[ind] && other.max()[ind] >= maxCoord[ind])
        split.hs = ALL;
    else if (other.min()[ind] <= minCoord[ind] && other.max()[ind] < maxCoord[ind])
        split.hs = LOW;
    else if (other.min()[ind] > minCoord[ind] && other.max()[ind] >= maxCoord[ind])
        split.hs = HIGH;
    else if (other.min()[ind] > minCoord[ind] && other.max()[ind] < maxCoord[ind])
        split.hs = MIDDLE;
    else assert(0);

    if ((other.min()[oi1] <= minCoord[oi1] && other.max()[oi1] >= maxCoord[oi1]) &&
        (other.min()[oi2] <= minCoord[oi2] && other.max()[oi2] >= maxCoord[oi2]))
        split.ts = TOTALLY_OUTSIDE;
    else if ((other.min()[oi1] > minCoord[oi1] && other.max()[oi1] < maxCoord[oi1]) &&
             (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] < maxCoord[oi2]))
        split.ts = TOTALLY_INSIDE;
    else if (((other.min()[oi1] <= minCoord[oi1] && other.max()[oi1] >= maxCoord[oi1]) &&
              (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] < maxCoord[oi2]))     ||
             ((other.min()[oi1] > minCoord[oi1] && other.max()[oi1] < maxCoord[oi1]) &&
              (other.min()[oi2] <= minCoord[oi2] && other.max()[oi2] >= maxCoord[oi2])))
        split.ts = TWO_EDGES;
    else {
        if ((other.min()[oi1] < minCoord[oi1] && other.max()[oi1] < maxCoord[oi1]) ||
            (other.min()[oi1] > minCoord[oi1] && other.max()[oi1] > maxCoord[oi1])){

            if ((other.min()[oi2] < minCoord[oi2] && other.max()[oi2] < maxCoord[oi2]) ||
                (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] > maxCoord[oi2]))
                split.ts = ONE_CORNER;

            else if (other.min()[oi2] < minCoord[oi2] && other.max()[oi2] > maxCoord[oi2])
                split.ts = TWO_CORNERS;
            else if (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] < maxCoord[oi2])
                split.ts =  ONE_EDGE;
            else assert(0);
        }
        else {
            if (other.min()[oi1] < minCoord[oi1] && other.max()[oi1] > maxCoord[oi1]){
                assert((other.min()[oi2] < minCoord[oi2] && other.max()[oi2] < maxCoord[oi2]) ||
                        (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] > maxCoord[oi2]));
                split.ts = TWO_CORNERS;
            }
            else if (other.min()[oi1] > minCoord[oi1] && other.max()[oi1] < maxCoord[oi1]){
                assert((other.min()[oi2] < minCoord[oi2] && other.max()[oi2] < maxCoord[oi2]) ||
                        (other.min()[oi2] > minCoord[oi2] && other.max()[oi2] > maxCoord[oi2]));
                split.ts = ONE_EDGE;
            }
            else assert(0);
        }
    }
    return split;
}

const std::set<unsigned int>& Box3D::getTrianglesCovered() const {
    return trianglesCovered;
}

void Box3D::setTrianglesCovered(const std::set<unsigned int>& value) {
    trianglesCovered = value;
}

void Box3D::addTrianglesCovered(const std::set<unsigned int>& value) {
    trianglesCovered.insert(value.begin(), value.end());
}

double Box3D::getBaseLevel() const {
    for (unsigned int i = 0; i < 6; i++){
        if (target == XYZ[i]){
            if (i < 3)
                return minCoord[i];
            else
                return maxCoord[i-3];
        }
    }
    assert(0);
    return -1;
}

void Box3D::setBaseLevel(double newBase) {
    for (unsigned int i = 0; i < 6; i++)
        if (target == XYZ[i]){
            double oldBase;
            if (i < 3){
                oldBase = minCoord[i];
                minCoord[i] = newBase;
                for (unsigned  int j = 0; j < piece.getNumberVertices(); j++){
                    if (piece.getVertex(j)[i] == oldBase){
                        Pointd p = piece.getVertex(j);
                        p[i] = newBase;
                        piece.setVertex(j, p);
                    }
                }
            }
            else{
                oldBase = maxCoord[i-3];
                maxCoord[i-3] = newBase;
                for (unsigned int j = 0; j < piece.getNumberVertices(); j++){
                    if (piece.getVertex(j)[i-3] == oldBase){
                        Pointd p = piece.getVertex(j);
                        p[i-3] = newBase;
                        piece.setVertex(j, p);
                    }
                }
            }
        }
}

#ifdef VIEWER_DEFINED
void Box3D::drawLine(const Pointd &a, const Pointd &b, const Color& c) const {
    glBegin(GL_LINES);
    glColor3f(c.redF(), c.greenF(), c.blueF());
    glLineWidth(3);
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glEnd();
}

void Box3D::drawCube() const {
    std::vector<Pointd> p;
    getRotatedExtremes(p);
    drawLine(p[0], p[1], color);
    drawLine(p[1], p[2], color);
    drawLine(p[2], p[3], color);
    drawLine(p[0], p[3], color);

    drawLine(p[4], p[5], color);
    drawLine(p[5], p[6], color);
    drawLine(p[6], p[7], color);
    drawLine(p[4], p[7], color);

    drawLine(p[0], p[4], color);
    drawLine(p[1], p[5], color);
    drawLine(p[2], p[6], color);
    drawLine(p[3], p[7], color);
}
#endif

unsigned int Box3D::getTargetIndex() {
    for (unsigned int i = 0; i < 6; i++){
        if (target == XYZ[i])
            return i%3;
    }
    assert(0);
    return -1;
}
