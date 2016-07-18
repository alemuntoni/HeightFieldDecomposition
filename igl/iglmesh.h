#ifndef IGLMESH_H
#define IGLMESH_H

#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>

#ifdef CGAL_DEFINED
#include <igl/copyleft/cgal/CSGTree.h>
#endif
#include <igl/jet.h>
#include "../common/serialize.h"

#ifdef DCEL_DEFINED
class Dcel;
#endif

class SimpleIGLMesh : public SerializableObject {
    public:
        SimpleIGLMesh();
        SimpleIGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
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
        bool saveOnObj(const std::string &filename) const;
        bool saveOnPly(const std::string &filename) const;
        void clear();
        #ifdef CGAL_DEFINED
        static void intersection(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
        static void difference(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
        #endif

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);
    protected:
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> V;
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> F;
};

class IGLMesh : public SimpleIGLMesh {
    public:
        IGLMesh();
        IGLMesh(const SimpleIGLMesh &m);
        IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
        IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXd &C);
        #ifdef DCEL_DEFINED
        IGLMesh(const Dcel& dcel);
        #endif
        void updateBoundingBox();
        void updateVertexNormals();
        void updateFaceNormals();
        void updateVertexAndFaceNormals();
        void clear();
        bool readFromFile(const std::string &filename);

        #ifdef CGAL_DEFINED
        static void intersection(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
        static void difference(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
        #endif

        #ifdef DCEL_DEFINED
        IGLMesh& operator= (const Dcel& dcel);
        #endif

        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

    protected:

        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> C;
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> NV;
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> NF;
        Eigen::RowVector3d BBmin, BBmax;
};

inline void SimpleIGLMesh::setVertex(unsigned int i, const Eigen::VectorXd& p) {
    assert (i < V.rows());
    assert (p.size() == 3);
    V.row(i) =  p;
}

inline void SimpleIGLMesh::setVertex(unsigned int i, double x, double y, double z) {
    assert (i < V.rows());
    V(i, 0) = x; V(i, 1) = y; V(i, 2) = z;
}

inline void SimpleIGLMesh::addVertex(const Eigen::VectorXd& p) {
    assert (p.size() == 3);
    V.resize(V.rows()+1, 3);
    V.row(V.rows()-1) = p;
}

inline void SimpleIGLMesh::addVertex(double x, double y, double z) {
    V.resize(V.rows()+1, 3);
    V(V.rows()-1, 0) = x; V(V.rows()-1, 1) = y; V(V.rows()-1, 2) = z;
}

inline void SimpleIGLMesh::resizeVertices(unsigned int nv) {
    V.resize(nv,3);
}

inline void SimpleIGLMesh::setFace(unsigned int i, const Eigen::VectorXi& f) {
    assert (i < F.rows());
    assert (f.size() == 3);
    F.row(i) =  f;
}

inline void SimpleIGLMesh::setFace(unsigned int i, int t1, int t2, int t3) {
    assert (i < F.rows());
    F(i, 0) = t1; F(i, 1) = t2; F(i, 2) = t3;
}

inline void SimpleIGLMesh::addFace(const Eigen::VectorXi& f) {
    assert (f.size() == 3);
    F.resize(F.rows()+1, 3);
    F.row(F.rows()-1) = f;
}

inline void SimpleIGLMesh::addFace(int t1, int t2, int t3) {
    F.resize(F.rows()+1, 3);
    F(F.rows()-1, 0) = t1; F(F.rows()-1, 1) = t2; F(F.rows()-1, 2) = t3;
}

inline void SimpleIGLMesh::resizeFaces(unsigned int nf) {
    F.resize(nf,3);
}

inline void SimpleIGLMesh::clear() {
    V.resize(0,3);
    F.resize(0,3);
}

inline void IGLMesh::updateBoundingBox() {
    BBmin = V.colwise().minCoeff();
    BBmax = V.colwise().maxCoeff();
}

inline void IGLMesh::updateVertexNormals() {
    igl::per_vertex_normals(V,F,NV);
}

inline void IGLMesh::updateFaceNormals() {
    igl::per_face_normals(V,F,NF);
}

inline void IGLMesh::updateVertexAndFaceNormals() {
    updateFaceNormals();
    updateVertexNormals();
}

inline void IGLMesh::clear() {
    V.resize(0,3);
    F.resize(0,3);
    C.resize(0,3);
    NV.resize(0,3);
    NF.resize(0,3);
}

#endif // IGLMESH_H
