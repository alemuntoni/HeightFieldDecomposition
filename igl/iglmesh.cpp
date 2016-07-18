#include "iglmesh.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

SimpleIGLMesh::SimpleIGLMesh() {
}

SimpleIGLMesh::SimpleIGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : V(V), F(F) {
}

IGLMesh::IGLMesh() {
}

IGLMesh::IGLMesh(const SimpleIGLMesh& m) : SimpleIGLMesh(m) {
    C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    igl::per_face_normals(V,F,NF);
    igl::per_vertex_normals(V,F,NV);
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : SimpleIGLMesh(V,F) {
    C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    igl::per_face_normals(this->V,this->F,NF);
    igl::per_vertex_normals(this->V,this->F,NV);
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& C) : SimpleIGLMesh(V,F), C(C) {
    NV.resize(V.rows(), 3);
    NF.resize(F.rows(), 3);
    igl::per_face_normals(this->V,this->F,NF);
    igl::per_vertex_normals(this->V,this->F,NV);
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

#ifdef DCEL_DEFINED
IGLMesh::IGLMesh(const Dcel& dcel) {
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
}
#endif

bool IGLMesh::readFromFile(const std::__cxx11::string& filename) {
    bool b = SimpleIGLMesh::readFromFile(filename);
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

bool SimpleIGLMesh::readFromFile(const std::__cxx11::string& filename) {
    bool b = igl::read_triangle_mesh(filename, V, F);
    return b;
}

bool SimpleIGLMesh::saveOnObj(const std::__cxx11::string& filename) const {
    return igl::writeOBJ(filename, V, F);
}

bool SimpleIGLMesh::saveOnPly(const std::__cxx11::string& filename) const {
    return igl::writePLY(filename, V, F);
}

#ifdef CGAL_DEFINED
void SimpleIGLMesh::intersection(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
    igl::copyleft::cgal::CSGTree M;
    M = {{m1.V,m1.F},{m2.V,m2.F},"i"};
    result.V = M.cast_V<Eigen::MatrixXd>();
    result.F = M.F();
}

void SimpleIGLMesh::difference(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
    igl::copyleft::cgal::CSGTree M;
    M = {{m1.V,m1.F},{m2.V,m2.F},"m"};
    result.V = M.cast_V<Eigen::MatrixXd>();
    result.F = M.F();
}
#endif

void SimpleIGLMesh::serialize(std::ofstream& binaryFile) const {
    Serializer::serialize(V, binaryFile);
    Serializer::serialize(F, binaryFile);
}

void SimpleIGLMesh::deserialize(std::ifstream& binaryFile) {
    Serializer::deserialize(V, binaryFile);
    Serializer::deserialize(F, binaryFile);
}

#ifdef CGAL_DEFINED
void IGLMesh::intersection(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
    SimpleIGLMesh sres;
    SimpleIGLMesh::intersection(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
    result = IGLMesh(sres);
    result.C = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
    result.NV.resize(result.V.rows(), 3);
    result.NF.resize(result.F.rows(), 3);
    result.updateVertexAndFaceNormals();
}

void IGLMesh::difference(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
    SimpleIGLMesh sres;
    SimpleIGLMesh::difference(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
    result = IGLMesh(sres);
    result.C = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
    result.NV.resize(result.V.rows(), 3);
    result.NF.resize(result.F.rows(), 3);
    result.updateVertexAndFaceNormals();
}
#endif

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
    updateBoundingBox();
}
