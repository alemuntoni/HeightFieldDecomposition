#include "iglmesh.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

namespace IGLInterface {
    SimpleIGLMesh::SimpleIGLMesh() {
    }

    SimpleIGLMesh::SimpleIGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : V(V), F(F) {
    }

    #ifdef DCEL_DEFINED
    SimpleIGLMesh::SimpleIGLMesh(const Dcel& dcel) {
        clear();
        V.resize(dcel.getNumberVertices(), 3);
        F.resize(dcel.getNumberFaces(), 3);
        std::map<int, int> vids;
        unsigned int i = 0;
        for (Dcel::ConstVertexIterator vit = dcel.vertexBegin(); vit != dcel.vertexEnd(); ++vit){
            const Dcel::Vertex* v = *vit;
            vids[v->getId()] = i;
            Pointd p = v->getCoordinate();
            V(i,0) = p.x(); V(i,1) = p.y(); V(i,2) = p.z();
            i++;
        }
        i = 0;
        for (Dcel::ConstFaceIterator fit = dcel.faceBegin(); fit != dcel.faceEnd(); ++fit){
            const Dcel::Face* f = *fit;
            F(i, 0) = vids[f->getVertex1()->getId()]; F(i, 1) = vids[f->getVertex2()->getId()]; F(i, 2) = vids[f->getVertex3()->getId()];
            i++;
        }
    }
    #endif

    void SimpleIGLMesh::decimate(unsigned int numberDesiredFaces) {
        Eigen::MatrixXd VV;
        Eigen::MatrixXi FF;
        Eigen::VectorXi J;
        igl::decimate(V, F, numberDesiredFaces, VV, FF, J);
        V = VV;
        F = FF;
    }

    bool SimpleIGLMesh::getDecimatedMesh(SimpleIGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi& mapping) {
        Eigen::MatrixXd VV;
        Eigen::MatrixXi FF;
        bool b = igl::decimate(V, F, numberDesiredFaces, VV, FF, mapping);
        decimated.V = VV;
        decimated.F = FF;
        return b;
    }

    Eigen::VectorXd SimpleIGLMesh::getSignedDistance(const Eigen::MatrixXd& points) const{
        Eigen::VectorXd S;
        Eigen::VectorXi I;
        Eigen::MatrixXd C,N;
        igl::signed_distance(points,V,F,igl::SIGNED_DISTANCE_TYPE_PSEUDONORMAL,S,I,C,N);
        return S;
    }

    void SimpleIGLMesh::translate(const Pointd& p) {
        Eigen::RowVector3d v;
        v << p.x(), p.y(), p.z();
        V.rowwise() += v;
    }

    void SimpleIGLMesh::translate(const Eigen::Vector3d& p) {
        V.rowwise() += p.transpose();
    }

    void SimpleIGLMesh::rotate(const Eigen::Matrix3d& m, const Eigen::Vector3d& centroid) {
        V.rowwise() -= centroid.transpose();
        for (unsigned int i = 0; i < V.rows(); i++){
            V.row(i) =  m * V.row(i).transpose();
        }
        V.rowwise() += centroid.transpose();
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

    void SimpleIGLMesh::unionn(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"u"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
    }
    #endif


    IGLMesh::IGLMesh() {
    }

    IGLMesh::IGLMesh(const SimpleIGLMesh& m) : SimpleIGLMesh(m) {
        C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(V,F,NF);
        igl::per_vertex_normals(V,F,NV);
        if (V.rows() > 0){
            BBmin = V.colwise().minCoeff();
            BBmax = V.colwise().maxCoeff();
        }
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

    void IGLMesh::setColor(double red, double green, double blue, int f) {
        if (f < 0){
            C.resize(F.rows(), 3);
            for (unsigned int i = 0; i < C.rows(); i++)
                C.row(i) << red, green, blue;
        }
        else{
            assert(f < F.rows());
            C.row(f) << red, green, blue;
        }
    }

    void IGLMesh::decimate(int numberDesiredFaces) {
        SimpleIGLMesh::decimate(numberDesiredFaces);
        C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        updateVertexAndFaceNormals();
        updateBoundingBox();
    }

    bool IGLMesh::getDecimatedMesh(IGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi& mapping) {
        bool b = SimpleIGLMesh::getDecimatedMesh(decimated, numberDesiredFaces, mapping);
        decimated.C = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        decimated.updateVertexAndFaceNormals();
        decimated.updateBoundingBox();
        return b;
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

    void IGLMesh::unionn(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh sres;
        SimpleIGLMesh::unionn(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
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
}
