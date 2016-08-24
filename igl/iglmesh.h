#ifndef IGLMESH_H
#define IGLMESH_H

#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>
#include <igl/signed_distance.h>
#include <igl/decimate.h>

#ifdef CGAL_DEFINED
#include <igl/copyleft/cgal/CSGTree.h>
#endif
#include <igl/jet.h>
#include "../common/serialize.h"
#include "../common/point.h"

#ifdef DCEL_DEFINED
class Dcel;
#endif

namespace IGLInterface {
    class SimpleIGLMesh : public SerializableObject {
        public:
            SimpleIGLMesh();
            SimpleIGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
            #ifdef DCEL_DEFINED
            SimpleIGLMesh(const Dcel& dcel);
            #endif
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
            unsigned int getNumberFaces() const;
            unsigned int getNumberVertices() const;
            Pointd getVertex(unsigned int i) const;
            Pointi getFace(unsigned int i) const;
            void getBoundingBox(Eigen::RowVector3d &BBmin, Eigen::RowVector3d &BBmax) const;
            void decimate(unsigned int numberDesiredFaces);
            bool getDecimatedMesh(SimpleIGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi &mapping);
            Eigen::VectorXd getSignedDistance(const Eigen::MatrixXd &points) const;

            void translate(const Pointd &p);
            void translate(const Eigen::Vector3d &p);
            void rotate(const Eigen::Matrix3d &m, const Eigen::Vector3d& centroid = Eigen::Vector3d::Zero());
            #ifdef CGAL_DEFINED
            static void intersection(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static void difference(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static void unionn(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
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
            void resizeVertices(unsigned int nv);
            void resizeFaces(unsigned int nf);
            void updateBoundingBox();
            void updateVertexNormals();
            void updateFaceNormals();
            void updateVertexAndFaceNormals();
            void clear();
            bool readFromFile(const std::string &filename);
            void setColor(double red, double green, double blue, int f = -1);
            Vec3 getNormal(unsigned int f) const;
            QColor getColor(unsigned int f) const;
            void getBoundingBox(Eigen::RowVector3d &BBmin, Eigen::RowVector3d &BBmax) const;
            void decimate(int numberDesiredFaces);
            bool getDecimatedMesh(IGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi &mapping);

            #ifdef CGAL_DEFINED
            static void intersection(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static void difference(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static void unionn(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
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
        V.conservativeResize(V.rows()+1, Eigen::NoChange);
        V.row(V.rows()-1) = p;
    }

    inline void SimpleIGLMesh::addVertex(double x, double y, double z) {
        V.conservativeResize(V.rows()+1, Eigen::NoChange);
        V(V.rows()-1, 0) = x; V(V.rows()-1, 1) = y; V(V.rows()-1, 2) = z;
    }

    inline void SimpleIGLMesh::resizeVertices(unsigned int nv) {
        V.conservativeResize(nv,Eigen::NoChange);
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
        F.conservativeResize(F.rows()+1, Eigen::NoChange);
        F.row(F.rows()-1) = f;
    }

    inline void SimpleIGLMesh::addFace(int t1, int t2, int t3) {
        F.conservativeResize(F.rows()+1, Eigen::NoChange);
        F(F.rows()-1, 0) = t1; F(F.rows()-1, 1) = t2; F(F.rows()-1, 2) = t3;
    }

    inline void SimpleIGLMesh::resizeFaces(unsigned int nf) {
        F.conservativeResize(nf,Eigen::NoChange);
    }

    inline bool SimpleIGLMesh::readFromFile(const std::__cxx11::string& filename) {
        return igl::read_triangle_mesh(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnObj(const std::__cxx11::string& filename) const {
        return igl::writeOBJ(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnPly(const std::__cxx11::string& filename) const {
        return igl::writePLY(filename, V, F);
    }

    inline void SimpleIGLMesh::clear() {
        V.resize(0,Eigen::NoChange);
        F.resize(0,Eigen::NoChange);
    }

    inline unsigned int SimpleIGLMesh::getNumberFaces() const {
        return F.rows();
    }

    inline unsigned int SimpleIGLMesh::getNumberVertices() const {
        return V.rows();
    }

    inline Pointd SimpleIGLMesh::getVertex(unsigned int i) const {
        assert(i < V.rows());
        return Pointd(V(i,0), V(i,1), V(i,2));
    }

    inline Pointi SimpleIGLMesh::getFace(unsigned int i) const {
        assert (i < F.rows());
        return Pointi(F(i,0), F(i,1), F(i,2));
    }

    inline void SimpleIGLMesh::getBoundingBox(Eigen::RowVector3d& BBmin, Eigen::RowVector3d& BBmax) const {
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
    }

    inline void SimpleIGLMesh::serialize(std::ofstream& binaryFile) const {
        Serializer::serialize(V, binaryFile);
        Serializer::serialize(F, binaryFile);
    }

    inline void SimpleIGLMesh::deserialize(std::ifstream& binaryFile) {
        Serializer::deserialize(V, binaryFile);
        Serializer::deserialize(F, binaryFile);
    }

    inline void IGLMesh::resizeVertices(unsigned int nv){
        SimpleIGLMesh::resizeVertices(nv);
        NV.conservativeResize(nv, Eigen::NoChange);
    }

    inline void IGLMesh::resizeFaces(unsigned int nf) {
        SimpleIGLMesh::resizeFaces(nf);
        NF.conservativeResize(nf, Eigen::NoChange);
        C.conservativeResize(nf, Eigen::NoChange);
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
        V.resize(0,Eigen::NoChange);
        F.resize(0,Eigen::NoChange);
        C.resize(0,Eigen::NoChange);
        NV.resize(0,Eigen::NoChange);
        NF.resize(0,Eigen::NoChange);
    }

    inline Vec3 IGLMesh::getNormal(unsigned int f) const {
        assert (f < F.rows());
        return Vec3(NF(f,0), NF(f,1), NF(f,2));
    }

    inline QColor IGLMesh::getColor(unsigned int f) const {
        assert (f < F.rows());
        QColor c;
        c.setRedF((float)C(f,0));
        c.setGreenF((float)C(f,1));
        c.setBlueF((float)C(f,2));
        return c;
    }

    inline void IGLMesh::getBoundingBox(Eigen::RowVector3d& BBmin, Eigen::RowVector3d& BBmax) const {
        BBmin = this->BBmin;
        BBmax = this->BBmax;
    }
}

#endif // IGLMESH_H
