#ifndef IGLMESH_H
#define IGLMESH_H

#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>
#include "../common/serialize.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

class IGLMesh : public SerializableObject {
    public:
        IGLMesh();
        IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
        IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXd &C);
        void setVertex(unsigned int i, const Eigen::VectorXd &p);
        void setVertex(unsigned int i, double x, double y, double z);
        void addVertex(const Eigen::VectorXd &p);
        void addVertex(double x, double y, double z);
        void resizeVertices(unsigned int nv);
        void setFace(unsigned int i, const Eigen::VectorXi &f);
        void setFace(unsigned int i, int t1, int t2, int t3);
        void addFace(const Eigen::VectorXi &f);
        void addFace(int t1, int t2, int t3);
        void resizeFaces(unsigned int nf);
        bool readFromFile(const std::string &filename);
        void updateBoundingBox();
        void clear();

        #ifdef DCEL_DEFINED
        IGLMesh& operator= (const Dcel& dcel);
        #endif

        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

    protected:
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        Eigen::MatrixXd C;
        Eigen::MatrixXd NV;
        Eigen::MatrixXd NF;

        Eigen::RowVector3d BBmin, BBmax;
};

inline void IGLMesh::setVertex(unsigned int i, const Eigen::VectorXd& p) {
    assert (i < V.rows());
    assert (p.size() == 3);
    V.row(i) =  p;
}

inline void IGLMesh::setVertex(unsigned int i, double x, double y, double z) {
    assert (i < V.rows());
    V(i, 0) = x; V(i, 1) = y; V(i, 2) = z;
}

inline void IGLMesh::addVertex(const Eigen::VectorXd& p) {
    assert (p.size() == 3);
    V.resize(V.rows()+1, 3);
    V.row(V.rows()-1) = p;
}

inline void IGLMesh::addVertex(double x, double y, double z) {
    V.resize(V.rows()+1, 3);
    V(V.rows()-1, 0) = x; V(V.rows()-1, 1) = y; V(V.rows()-1, 2) = z;
}

inline void IGLMesh::resizeVertices(unsigned int nv) {
    V.resize(nv,3);
    NV.resize(nv,3);
}

inline void IGLMesh::setFace(unsigned int i, const Eigen::VectorXi& f) {
    assert (i < F.rows());
    assert (f.size() == 3);
    F.row(i) =  f;
}

inline void IGLMesh::setFace(unsigned int i, int t1, int t2, int t3) {
    assert (i < F.rows());
    F(i, 0) = t1; F(i, 1) = t2; F(i, 2) = t3;
}

inline void IGLMesh::addFace(const Eigen::VectorXi& f) {
    assert (f.size() == 3);
    F.resize(F.rows()+1, 3);
    F.row(F.rows()-1) = f;
}

inline void IGLMesh::addFace(int t1, int t2, int t3) {
    F.resize(F.rows()+1, 3);
    F(F.rows()-1, 0) = t1; F(F.rows()-1, 1) = t2; F(F.rows()-1, 2) = t3;
}

inline void IGLMesh::resizeFaces(unsigned int nf) {
    F.resize(nf,3);
    NF.resize(nf,3);
    C.resize(nf,3);
}

inline void IGLMesh::updateBoundingBox() {
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

inline void IGLMesh::clear() {
    V.resize(0,3);
    F.resize(0,3);
    C.resize(0,3);
    NV.resize(0,3);
    NF.resize(0,3);
}

#endif // IGLMESH_H
