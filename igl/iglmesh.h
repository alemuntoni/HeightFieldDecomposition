#ifndef IGLMESH_H
#define IGLMESH_H

#include <Eigen/Core>
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
#include <igl/writeOBJ.h>
#include <igl/writePLY.h>
#include <igl/writeOFF.h>
#include <igl/writeSTL.h>
#include <igl/signed_distance.h>
#include <igl/decimate.h>
#include <igl/remove_duplicates.h>
#include <igl/remove_unreferenced.h>
#include <igl/facet_components.h>

#ifdef CGAL_DEFINED
#include <igl/copyleft/cgal/CSGTree.h>
#endif
#include <igl/jet.h>
#include "../common/serialize.h"
#include "../common/point.h"
#include "../common/bounding_box.h"

#ifdef DCEL_DEFINED
class Dcel;
#endif

#ifdef TRIMESH_DEFINED
template<typename T>
    class Trimesh;
#endif

namespace IGLInterface {
    class SimpleIGLMesh : public SerializableObject {
        public:
            SimpleIGLMesh();
            SimpleIGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
            #ifdef DCEL_DEFINED
            SimpleIGLMesh(const Dcel& dcel);
            #endif
            #ifdef TRIMESH_DEFINED
            template<typename T>
            SimpleIGLMesh(const Trimesh<T>& trimesh);
            #endif
            void setVertex(unsigned int i, const Eigen::VectorXd &p);
            void setVertex(unsigned int i, const Pointd &p);
            void setVertex(unsigned int i, double x, double y, double z);
            void addVertex(const Eigen::VectorXd &p);
            void addVertex(double x, double y, double z);
            void resizeVertices(unsigned int nv);
            void setFace(unsigned int i, const Eigen::VectorXi &f);
            void setFace(unsigned int i, int t1, int t2, int t3);
            void addFace(const Eigen::VectorXi &f);
            void addFace(int t1, int t2, int t3);
            void resizeFaces(unsigned int nf);
            void removeDuplicatedVertices(Eigen::Matrix<int, Eigen::Dynamic, 1> &I = dummy);
            bool readFromFile(const std::string &filename);
            bool saveOnObj(const std::string &filename) const;
            bool saveOnPly(const std::string &filename) const;
            bool saveOnOff(const std::string &filename) const;
            bool saveOnStl(const std::string &filename) const;
            void clear();
            unsigned int getNumberFaces() const;
            unsigned int getNumberVertices() const;
            Pointd getVertex(unsigned int i) const;
            Pointi getFace(unsigned int i) const;
            void getBoundingBox(Eigen::RowVector3d &BBmin, Eigen::RowVector3d &BBmax) const;
            BoundingBox getBoundingBox() const;
            void decimate(unsigned int numberDesiredFaces);
            bool getDecimatedMesh(SimpleIGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi &mapping);
            Eigen::VectorXd getSignedDistance(const Eigen::MatrixXd &points) const;
            Eigen::MatrixXd getVerticesMatrix() const;
            Eigen::MatrixXi getFacesMatrix() const;
            void translate(const Pointd &p);
            void translate(const Eigen::Vector3d &p);
            void rotate(const Eigen::Matrix3d &m, const Eigen::Vector3d& centroid = Eigen::Vector3d::Zero());
            void scale(const BoundingBox& newBoundingBox);
            void scale(const BoundingBox& oldBoundingBox, const BoundingBox& newBoundingBox);
            void scale(const Vec3 &scaleFactor);
            #ifdef CGAL_DEFINED
            static void intersection(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static SimpleIGLMesh intersection(const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static void difference(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static SimpleIGLMesh difference(const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static void unionn(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static SimpleIGLMesh unionn(const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            #endif
            static void merge(SimpleIGLMesh &result, const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            static SimpleIGLMesh merge(const SimpleIGLMesh &m1, const SimpleIGLMesh &m2);
            void removeUnreferencedVertices(Eigen::Matrix<int, Eigen::Dynamic, 1> &I = dummy);
            void getConnectedComponents(std::vector<SimpleIGLMesh>& connectedComponents);

            // SerializableObject interface
            void serialize(std::ofstream& binaryFile) const;
            bool deserialize(std::ifstream& binaryFile);


        protected:
            Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> V;
            Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> F;
            static Eigen::Matrix<int, Eigen::Dynamic, 1> dummy;
    };

    class IGLMesh : public SimpleIGLMesh {
        public:
            IGLMesh();
            IGLMesh(const SimpleIGLMesh &m);
            IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
            IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, const Eigen::MatrixXf& CV, const Eigen::MatrixXf& CF);
            #ifdef DCEL_DEFINED
            IGLMesh(const Dcel& dcel);
            #endif
            #ifdef TRIMESH_DEFINED
            template<typename T>
            IGLMesh(const Trimesh<T>& trimesh);
            #endif
            void resizeVertices(unsigned int nv);
            void resizeFaces(unsigned int nf);
            void updateBoundingBox();
            void updateVertexNormals();
            void updateFaceNormals();
            void updateVertexAndFaceNormals();
            void removeDuplicatedVertices();
            void clear();
            bool readFromFile(const std::string &filename);
            void setFaceColor(double red, double green, double blue, int f = -1);
            Vec3 getNormal(unsigned int f) const;
            QColor getColor(unsigned int f) const;
            void getBoundingBox(Eigen::RowVector3d &BBmin, Eigen::RowVector3d &BBmax) const;
            BoundingBox getBoundingBox() const;
            void decimate(int numberDesiredFaces);
            bool getDecimatedMesh(IGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi &mapping);
            void scale(const BoundingBox& newBoundingBox);
            void scale(const BoundingBox& oldBoundingBox, const BoundingBox& newBoundingBox);
            void scale(const Vec3 &scaleFactor);
            Eigen::MatrixXf getVerticesColorMatrix() const;

            bool saveOnPly(const std::string &filename) const;
            void deleteVerticesLowerThanY(double y);


            #ifdef CGAL_DEFINED
            static void intersection(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static IGLMesh intersection(const IGLMesh &m1, const IGLMesh &m2);
            static void difference(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static IGLMesh difference(const IGLMesh &m1, const IGLMesh &m2);
            static void unionn(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static IGLMesh unionn(const IGLMesh &m1, const IGLMesh &m2);
            #endif
            static void merge(IGLMesh &result, const IGLMesh &m1, const IGLMesh &m2);
            static IGLMesh merge(const IGLMesh &m1, const IGLMesh &m2);

            #ifdef DCEL_DEFINED
            IGLMesh& operator= (const Dcel& dcel);
            #endif

            void serialize(std::ofstream& binaryFile) const;
            bool deserialize(std::ifstream& binaryFile);

        protected:

            Eigen::RowVector3d BBmin, BBmax;
            Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> NV;
            Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> NF;
            Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> CV;
            Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> CF;
    };

    #ifdef TRIMESH_DEFINED
    template <typename T>
    inline SimpleIGLMesh::SimpleIGLMesh(const Trimesh<T>& trimesh)
    {
        int numV=trimesh.numVertices();
        int numF=trimesh.numTriangles();

        clear();
        V.resize(numV,3);
        F.resize(numF,3);

        for(int i=0;i<numV;++i)
        {
            V(i,0)=trimesh.vertex(i).x();
            V(i,1)=trimesh.vertex(i).y();
            V(i,2)=trimesh.vertex(i).z();
        }

        for(int i=0;i<numF;++i)
        {
            F(i,0)=trimesh.tri_vertex_id(i,0);
            F(i,1)=trimesh.tri_vertex_id(i,1);
            F(i,2)=trimesh.tri_vertex_id(i,2);
        }
    }
    #endif


    inline void SimpleIGLMesh::setVertex(unsigned int i, const Eigen::VectorXd& p) {
        assert (i < V.rows());
        assert (p.size() == 3);
        V.row(i) =  p;
    }

    inline void SimpleIGLMesh::setVertex(unsigned int i, const Pointd& p) {
        assert (i < V.rows());
        V(i,0) = p[0]; V(i,1) = p[1]; V(i,2) = p[2];
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

    inline bool SimpleIGLMesh::readFromFile(const std::string& filename) {
        return igl::read_triangle_mesh(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnObj(const std::string& filename) const {
        return igl::writeOBJ(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnPly(const std::string& filename) const {
        return igl::writePLY(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnOff(const std::string& filename) const {
        return igl::writeOFF(filename, V, F);
    }

    inline bool SimpleIGLMesh::saveOnStl(const std::__cxx11::string& filename) const {
        return igl::writeSTL(filename, V, F);
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
        if (V.rows() > 0){
            BBmin = V.colwise().minCoeff();
            BBmax = V.colwise().maxCoeff();
        }
        else {
            BBmin = Eigen::RowVector3d();
            BBmax = Eigen::RowVector3d();
        }
    }

    inline BoundingBox SimpleIGLMesh::getBoundingBox() const {
        BoundingBox  bb;
        if (V.rows() > 0){
            Eigen::RowVector3d BBmin, BBmax;
            BBmin = V.colwise().minCoeff();
            BBmax = V.colwise().maxCoeff();
            bb.setMin(BBmin(0), BBmin(1), BBmin(2));
            bb.setMax(BBmax(0), BBmax(1), BBmax(2));
        }
        return bb;
    }

    inline void SimpleIGLMesh::serialize(std::ofstream& binaryFile) const {
        Serializer::serialize(V, binaryFile);
        Serializer::serialize(F, binaryFile);
    }

    inline bool SimpleIGLMesh::deserialize(std::ifstream& binaryFile) {
        SimpleIGLMesh tmp;
        if (Serializer::deserialize(tmp.V, binaryFile) &&
            Serializer::deserialize(tmp.F, binaryFile)){
            *this = std::move(tmp);
            return true;
        }
        return false;
    }


    #ifdef TRIMESH_DEFINED
    template <typename T>
    inline IGLMesh::IGLMesh(const Trimesh<T>& trimesh) : SimpleIGLMesh(trimesh)
    {
        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(V,F,NF);
        igl::per_vertex_normals(V,F,NV);
        if (V.rows() > 0){
            BBmin = V.colwise().minCoeff();
            BBmax = V.colwise().maxCoeff();
        }
    }
    #endif

    inline void IGLMesh::resizeVertices(unsigned int nv){
        SimpleIGLMesh::resizeVertices(nv);
        NV.conservativeResize(nv, Eigen::NoChange);
    }

    inline void IGLMesh::resizeFaces(unsigned int nf) {
        SimpleIGLMesh::resizeFaces(nf);
        NF.conservativeResize(nf, Eigen::NoChange);
        CF.conservativeResize(nf, Eigen::NoChange);
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
        CF.conservativeResize(F.rows(), Eigen::NoChange);
        CV.conservativeResize(V.rows(), Eigen::NoChange);
    }

    inline void IGLMesh::clear() {
        V.resize(0,Eigen::NoChange);
        F.resize(0,Eigen::NoChange);
        CF.resize(0,Eigen::NoChange);
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
        c.setRedF((float)CF(f,0));
        c.setGreenF((float)CF(f,1));
        c.setBlueF((float)CF(f,2));
        return c;
    }

    inline void IGLMesh::getBoundingBox(Eigen::RowVector3d& BBmin, Eigen::RowVector3d& BBmax) const {
        BBmin = this->BBmin;
        BBmax = this->BBmax;
    }
}

#endif // IGLMESH_H
