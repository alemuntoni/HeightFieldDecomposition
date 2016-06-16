/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "common.h"

int WINDOW_MANAGER_ID;
int DCEL_MANAGER_ID;
int ENGINE_MANAGER_ID;

QColor colorOfNormal(const Vec3 &normal) {
    for (unsigned int i = 0; i < XYZ.size(); i++)
        if (epsilonEqual(XYZ[i], normal)) return colors[i];
    return QColor();
}

std::string exec(const char* cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

void getEigenVerteicesFacesFromDcel(Eigen::MatrixXd& V, Eigen::MatrixXi& F, Eigen::MatrixXd& C, Eigen::MatrixXd& NV, Eigen::MatrixXd& NF, Eigen::MatrixXd &BB, const Dcel& d) {
    std::map<int, int> vertices;
    int i = 0;
    V.resize(d.getNumberVertices(), 3);
    NV.resize(d.getNumberVertices(), 3);
    for (Dcel::ConstVertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
        const Dcel::Vertex* v = *vit;
        vertices[v->getId()] = i;
        V(i,0) = v->getCoordinate().x();
        V(i,1) = v->getCoordinate().y();
        V(i,2) = v->getCoordinate().z();
        Vec3 n = v->getNormal();
        NV(i,0) = n.x();
        NV(i,1) = n.y();
        NV(i,2) = n.x();
        i++;
    }
    i= 0;
    F = Eigen::MatrixXi(d.getNumberFaces(), 3);
    C = Eigen::MatrixXd(d.getNumberFaces(), 3);
    NF = Eigen::MatrixXd(d.getNumberFaces(), 3);
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        Dcel::Face::ConstIncidentVertexIterator vit = f->incidentVertexBegin();
        F(i,0) = vertices[(*vit)->getId()];
        vit++;
        F(i,1) = vertices[(*vit)->getId()];
        vit++;
        F(i,2) = vertices[(*vit)->getId()];
        QColor c = f->getColor();
        C(i,0) = c.redF();
        C(i,1) = c.greenF();
        C(i,2) = c.blueF();
        Vec3 n = f->getNormal();
        NF(i,0) = n.x();
        NF(i,1) = n.y();
        NF(i,2) = n.x();
        i++;
    }
    BoundingBox bb = d.getBoundingBox();
    BB = Eigen::MatrixXd(8, 3);
    BB(0,0) = bb.getMinX(); BB(0,1) = bb.getMinY(); BB(0,2) = bb.getMinZ();
    BB(1,0) = bb.getMinX(); BB(1,1) = bb.getMinY(); BB(1,2) = bb.getMaxZ();
    BB(2,0) = bb.getMaxX(); BB(2,1) = bb.getMinY(); BB(2,2) = bb.getMinZ();
    BB(3,0) = bb.getMaxX(); BB(3,1) = bb.getMinY(); BB(3,2) = bb.getMaxZ();
    BB(4,0) = bb.getMinX(); BB(4,1) = bb.getMaxY(); BB(4,2) = bb.getMinZ();
    BB(5,0) = bb.getMinX(); BB(5,1) = bb.getMaxY(); BB(5,2) = bb.getMaxZ();
    BB(6,0) = bb.getMaxX(); BB(6,1) = bb.getMaxY(); BB(6,2) = bb.getMinZ();
    BB(7,0) = bb.getMaxX(); BB(7,1) = bb.getMaxY(); BB(7,2) = bb.getMaxZ();
}
