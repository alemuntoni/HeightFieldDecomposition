#include "iglmesh.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

IGLMesh::IGLMesh() {
}

IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : V(V), F(F) {
    C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    igl::per_face_normals(V,F,NF);
    igl::per_vertex_normals(V,F,NV);
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& C) : V(V), F(F), C(C) {
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    igl::per_face_normals(V,F,NF);
    igl::per_vertex_normals(V,F,NV);
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

bool IGLMesh::readFromFile(const std::__cxx11::string& filename) {
    bool b = igl::read_triangle_mesh(filename, V, F);
    if (b){
        C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(V,F,NF);
        igl::per_vertex_normals(V,F,NV);
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
    }
    return b;
}

#ifdef DCEL_DEFINED
IGLMesh& IGLMesh::operator=(const Dcel& dcel) {
    clear();
    V.resize(dcel.getNumberVertices(), 3);
    F.resize(dcel.getNumberFaces(), 3);
    C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    std::map<int, int> vids;
    BBmin(0) = dcel.getBoundingBox().getMinX(); BBmin(1) = dcel.getBoundingBox().getMinY(); BBmin(2) = dcel.getBoundingBox().getMinZ();
    BBmax(0) = dcel.getBoundingBox().getMaxX(); BBmax(1) = dcel.getBoundingBox().getMaxY(); BBmax(2) = dcel.getBoundingBox().getMaxZ();
    unsigned int i = 0;
    for (Dcel::ConstVertexIterator vit = dcel.vertexBegin(); vit != dcel.vertexEnd(); ++vit){
        const Dcel::Vertex* v = *vit;
        vids[v->getId()] = i;
        Pointd p = v->getCoordinate();
        Vec3 n = v->getNormal();
        V(i,0) = p.x(); V(i,1) = p.y(); V(i,2) = p.z();
        NV(i,0) = n.x(); NV(i,1) = n.y(); NV(i,2) = n.z();
        i++;
    }
    i = 0;
    for (Dcel::ConstFaceIterator fit = dcel.faceBegin(); fit != dcel.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        F(i, 0) = vids[f->getVertex1()->getId()]; F(i, 1) = vids[f->getVertex2()->getId()]; F(i, 2) = vids[f->getVertex3()->getId()];
        QColor c = f->getColor();
        Vec3 n = f->getNormal();
        C(i,0) = c.redF(); C(i,1) = c.greenF(); C(i,2) = c.blueF();
        NF(i,0) = n.x(); NF(i,1) = n.y(); NF(i,2) = n.z();
        i++;
    }
    return *this;
}
#endif

void IGLMesh::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(V, binaryFile);
    Serializer::serialize(F, binaryFile);
    Serializer::serialize(C, binaryFile);
    Serializer::serialize(NV, binaryFile);
    Serializer::serialize(NF, binaryFile);
}

void IGLMesh::deserialize(std::ifstream& binaryFile) {
    Serializer::deserialize(V, binaryFile);
    Serializer::deserialize(F, binaryFile);
    Serializer::deserialize(C, binaryFile);
    Serializer::deserialize(NV, binaryFile);
    Serializer::deserialize(NF, binaryFile);
}
