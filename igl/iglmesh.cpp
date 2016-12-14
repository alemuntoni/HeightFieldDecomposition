#include "iglmesh.h"
#include <boost/tokenizer.hpp>
#include "../common/timer.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

#ifdef TRIMESH_DEFINED
#include "../trimesh/trimesh.h"
#endif

namespace IGLInterface {
    SimpleIGLMesh::SimpleIGLMesh() {
    }

    SimpleIGLMesh::SimpleIGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : V(V), F(F) {
    }

    void SimpleIGLMesh::deleteDuplicatedVertices(Eigen::Matrix<int, Eigen::Dynamic, 1> &I) {
        //Eigen::MatrixXd V = this->V, NV; //if missing templates in iglstatic
        //Eigen::MatrixXi F = this->F, NF;
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> NV;
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> NF;


        igl::remove_duplicates(V,F,NV,NF,I);

        this->V = NV;
        this->F = NF;
        //TODO: pull request on libigl with new template function remove_duplicates
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

    Eigen::MatrixXd SimpleIGLMesh::getVerticesMatrix() const {
        return V;
    }

    Eigen::MatrixXi SimpleIGLMesh::getFacesMatrix() const {
        return F;
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

    void SimpleIGLMesh::scale(const BoundingBox& newBoundingBox) {
        BoundingBox boundingBox = getBoundingBox();
        Pointd oldCenter = boundingBox.center();
        Pointd newCenter = newBoundingBox.center();
        Pointd deltaOld = boundingBox.getMax() - boundingBox.getMin();
        Pointd deltaNew = newBoundingBox.getMax() - newBoundingBox.getMin();
        for (int i = 0; i < V.rows(); i++){
            Pointd coord = getVertex(i);
            coord -= oldCenter;
            coord *= deltaNew / deltaOld;
            coord += newCenter;
            setVertex(i, coord);
        }
    }

    void SimpleIGLMesh::scale(const BoundingBox& oldBoundingBox, const BoundingBox& newBoundingBox) {
        Pointd oldCenter = oldBoundingBox.center();
        Pointd newCenter = newBoundingBox.center();
        Pointd deltaOld = oldBoundingBox.getMax() - oldBoundingBox.getMin();
        Pointd deltaNew = newBoundingBox.getMax() - newBoundingBox.getMin();
        for (int i = 0; i < V.rows(); i++){
            Pointd coord = getVertex(i);
            coord -= oldCenter;
            coord *= deltaNew / deltaOld;
            coord += newCenter;
            setVertex(i, coord);
        }
    }

    void SimpleIGLMesh::scale(const Vec3& scaleFactor) {
        if (scaleFactor.x() > 0 && scaleFactor.y() > 0 && scaleFactor.z() > 0){
            BoundingBox bb = getBoundingBox();
            Pointd center = bb.center();
            Pointd newMax(bb.min().x() + bb.getLengthX()*scaleFactor.x(), bb.min().y() + bb.getLengthY()*scaleFactor.y(), bb.min().z() + bb.getLengthZ()*scaleFactor.z());
            bb.setMax(newMax);
            Pointd trans = center - bb.center();
            bb.min() += trans;
            bb.max() += trans;
            scale(bb);
        }
    }

    #ifdef CGAL_DEFINED
    void SimpleIGLMesh::intersection(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"i"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
    }

    SimpleIGLMesh SimpleIGLMesh::intersection(const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        SimpleIGLMesh result;
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"i"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
        return result;
    }

    void SimpleIGLMesh::difference(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"m"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
    }

    SimpleIGLMesh SimpleIGLMesh::difference(const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        SimpleIGLMesh result;
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"m"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
        return result;
    }

    void SimpleIGLMesh::unionn(SimpleIGLMesh& result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"u"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
    }

    SimpleIGLMesh SimpleIGLMesh::unionn(const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        SimpleIGLMesh result;
        igl::copyleft::cgal::CSGTree M;
        M = {{m1.V,m1.F},{m2.V,m2.F},"u"};
        result.V = M.cast_V<Eigen::MatrixXd>();
        result.F = M.F();
        return result;
    }
    #endif

    void SimpleIGLMesh::merge(SimpleIGLMesh &result, const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        result.V.resize(m1.V.rows()+m2.V.rows(), 3);
        result.V << m1.V,
                    m2.V;
        result.F = m1.F;
        int start = m1.getNumberVertices();
        for (unsigned int i = 0; i < m2.getNumberFaces(); i++){
            Pointi fi =m2.getFace(i);
            result.addFace(fi.x()+start, fi.y()+start, fi.z()+start);
        }
    }

    SimpleIGLMesh SimpleIGLMesh::merge(const SimpleIGLMesh& m1, const SimpleIGLMesh& m2) {
        SimpleIGLMesh result;
        result.V.resize(m1.V.rows()+m2.V.rows(), 3);
        result.V << m1.V,
                    m2.V;
        result.F = m1.F;
        int start = m1.getNumberVertices();
        for (unsigned int i = 0; i < m2.getNumberFaces(); i++){
            Pointi fi =m2.getFace(i);
            result.addFace(fi.x()+start, fi.y()+start, fi.z()+start);
        }
        return result;
    }


    IGLMesh::IGLMesh() {
    }

    IGLMesh::IGLMesh(const SimpleIGLMesh& m) : SimpleIGLMesh(m) {
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

    IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : SimpleIGLMesh(V,F) {
        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(this->V,this->F,NF);
        igl::per_vertex_normals(this->V,this->F,NV);
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
    }

    IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& C) : SimpleIGLMesh(V,F), CF(C) {
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(this->V,this->F,NF);
        igl::per_vertex_normals(this->V,this->F,NV);
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
    }

    void IGLMesh::deleteDuplicatedVertices() {
        Eigen::Matrix<int, Eigen::Dynamic, 1> I;

        SimpleIGLMesh::deleteDuplicatedVertices(I);
        std::vector<int> K(V.rows());
        for (unsigned int i = 0; i < I.size(); i++)
            K[I[i]] = i;
        for (unsigned int i = 0; i < V.rows(); i++){
            CV.row(i) = CV.row(K[i]);
        }
        CV.conservativeResize(V.rows(), 3);

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

    #ifdef DCEL_DEFINED
    IGLMesh::IGLMesh(const Dcel& dcel) {
        clear();
        V.resize(dcel.getNumberVertices(), 3);
        F.resize(dcel.getNumberFaces(), 3);
        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
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
            CF(i,0) = c.redF(); CF(i,1) = c.greenF(); CF(i,2) = c.blueF();
            NF(i,0) = n.x(); NF(i,1) = n.y(); NF(i,2) = n.z();
            i++;
        }
    }

    #endif

    bool IGLMesh::readFromFile(const std::string& filename) {
        bool b = SimpleIGLMesh::readFromFile(filename);
        if (b){
            CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
            CV = Eigen::MatrixXd::Constant(V.rows(), 3, 0.5);
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
            CF.resize(F.rows(), 3);
            for (unsigned int i = 0; i < CF.rows(); i++)
                CF.row(i) << red, green, blue;
        }
        else{
            assert(f < F.rows());
            CF.row(f) << red, green, blue;
        }
    }

    BoundingBox IGLMesh::getBoundingBox() const {
        BoundingBox  bb;
        bb.setMin(BBmin(0), BBmin(1), BBmin(2));
        bb.setMax(BBmax(0), BBmax(1), BBmax(2));
        return bb;
    }

    void IGLMesh::decimate(int numberDesiredFaces) {
        SimpleIGLMesh::decimate(numberDesiredFaces);
        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        updateVertexAndFaceNormals();
        updateBoundingBox();
    }

    bool IGLMesh::getDecimatedMesh(IGLMesh& decimated, unsigned int numberDesiredFaces, Eigen::VectorXi& mapping) {
        bool b = SimpleIGLMesh::getDecimatedMesh(decimated, numberDesiredFaces, mapping);
        decimated.CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        decimated.updateVertexAndFaceNormals();
        decimated.updateBoundingBox();
        return b;
    }

    void IGLMesh::scale(const BoundingBox& newBoundingBox) {
        SimpleIGLMesh::scale(newBoundingBox);
        BBmin(0) = newBoundingBox.min()[0];
        BBmin(1) = newBoundingBox.min()[1];
        BBmin(2) = newBoundingBox.min()[2];
        BBmax(0) = newBoundingBox.max()[0];
        BBmax(1) = newBoundingBox.max()[1];
        BBmax(2) = newBoundingBox.max()[2];
    }

    void IGLMesh::scale(const BoundingBox& oldBoundingBox, const BoundingBox& newBoundingBox) {
        SimpleIGLMesh::scale(oldBoundingBox, newBoundingBox);
        BBmin(0) = newBoundingBox.min()[0];
        BBmin(1) = newBoundingBox.min()[1];
        BBmin(2) = newBoundingBox.min()[2];
        BBmax(0) = newBoundingBox.max()[0];
        BBmax(1) = newBoundingBox.max()[1];
        BBmax(2) = newBoundingBox.max()[2];
    }

    void IGLMesh::scale(const Vec3& scaleFactor) {
        SimpleIGLMesh::scale(scaleFactor);
        updateBoundingBox();
    }

    Eigen::MatrixXd IGLMesh::getVerticesColorMatrix() const {
        return CV;
    }

    bool IGLMesh::saveOnPly(const std::string &filename) const {
        unsigned int numV=V.rows();
        unsigned int numF=F.rows();

        std::ofstream f;
        f.open(filename);

        if(!f.is_open())
            return false;

        f<<"ply"<<std::endl
         <<"format ascii 1.0"<<std::endl
         <<"comment file created by Batcave"<<std::endl;
        f<<"element vertex "<<numV<<std::endl;
        f<<"property float x"<<std::endl
         <<"property float y"<<std::endl
         <<"property float z"<<std::endl
         <<"property uchar red"<<std::endl
         <<"property uchar green"<<std::endl
         <<"property uchar blue"<<std::endl;
        f<<"element face "<<numF<<std::endl;
        f<<"property list uchar int vertex_index"<<std::endl
         <<"property uchar red"<<std::endl
         <<"property uchar green"<<std::endl
         <<"property uchar blue"<<std::endl
         <<"end_header"<<std::endl;
        for(unsigned int i=0;i<numV;++i) {
            f<<V(i,0)<<" "<<V(i,1)<<" "<<V(i,2)<<" "<<(int)(CV(i,0)*255)<<" "<<(int)(CV(i,1)*255)<<" "<<(int)(CV(i,2)*255)<<std::endl;
        }

        for(unsigned int i=0;i<numF;++i) {
            f<< "3 " << F(i,0)<<" "<<F(i,1)<<" "<<F(i,2) <<(int)(CF(i,0)*255)<<" "<<(int)(CF(i,1)*255)<<" "<<(int)(CF(i,2)*255) <<std::endl;
        }
        f.close();

        return true;
    }

    void IGLMesh::deleteVerticesLowerThanY(double y) {
        std::vector<int> indicesNewToOld;
        std::map<int, int> indicesOldToNew;
        std::set<int> toDelete;
        Eigen::MatrixXd NV;
        Eigen::MatrixXi NF;

        unsigned int j = 0;
        for (unsigned int i = 0; i < V.rows(); ++i){
            if (V(i,1) >= y){
                indicesNewToOld.push_back(i);
                indicesOldToNew[i] = j;
                j++;
            }
            else{
                toDelete.insert(i);
            }
        }

        NV.resize(indicesNewToOld.size(), 3);
        for (unsigned int j = 0; j < indicesNewToOld.size(); j++){
            NV.row(j) = V.row(indicesNewToOld[j]);
            CV.row(j) = CV.row(indicesNewToOld[j]);
        }

        j = 0;
        std::vector<int> indicesFaces;
        for (unsigned int i = 0; i < F.rows(); i++) {
            if (toDelete.find(F(i,0)) == toDelete.end() &&
                toDelete.find(F(i,1)) == toDelete.end() &&
                toDelete.find(F(i,2)) == toDelete.end()){
                indicesFaces.push_back(i);
            }
        }
        NF.resize(indicesFaces.size(), 3);
        for (unsigned int i = 0; i < indicesFaces.size(); i++) {
            NF.row(i) = F.row(indicesFaces[i]);
            assert(indicesOldToNew.find(NF(i,0)) != indicesOldToNew.end());
            assert(indicesOldToNew.find(NF(i,1)) != indicesOldToNew.end());
            assert(indicesOldToNew.find(NF(i,2)) != indicesOldToNew.end());
            NF(i,0) = indicesOldToNew[NF(i,0)];
            NF(i,1) = indicesOldToNew[NF(i,1)];
            NF(i,2) = indicesOldToNew[NF(i,2)];
        }

        V = NV;
        F = NF;

        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        this->NV.resize(V.rows(), 3);
        this->NF.resize(F.rows(), 3);
        igl::per_face_normals(V,F,this->NF);
        igl::per_vertex_normals(V,F,this->NV);
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
    }

    #ifdef CGAL_DEFINED
    void IGLMesh::intersection(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh sres;
        SimpleIGLMesh::intersection(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
    }

    IGLMesh IGLMesh::intersection(const IGLMesh& m1, const IGLMesh& m2) {
        IGLMesh result;
        SimpleIGLMesh sres;
        SimpleIGLMesh::intersection(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
        return result;
    }

    void IGLMesh::difference(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh sres;
        SimpleIGLMesh::difference(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
    }

    IGLMesh IGLMesh::difference(const IGLMesh& m1, const IGLMesh& m2) {
        IGLMesh result;
        SimpleIGLMesh sres;
        SimpleIGLMesh::difference(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
        return result;
    }

    void IGLMesh::unionn(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh sres;
        SimpleIGLMesh::unionn(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
    }

    IGLMesh IGLMesh::unionn(const IGLMesh& m1, const IGLMesh& m2) {
        IGLMesh result;
        SimpleIGLMesh sres;
        SimpleIGLMesh::unionn(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
        return result;
    }
    #endif

    void IGLMesh::merge(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh::merge(result, m1, m2);
        result.CF.resize(m1.CF.rows()+m2.CF.rows(), 3);
        result.CF << m1.CF,
                     m2.CF;
        result.CV.resize(m1.CV.rows()+m2.CV.rows(), 3);
        result.CV << m1.CV,
                     m2.CV;
        result.NV.resize(m1.NV.rows()+m2.NV.rows(), 3);
        result.NV << m1.NV,
                     m2.NV;
        result.NF.resize(m1.NF.rows()+m2.NF.rows(), 3);
        result.NF << m1.NF,
                     m2.NF;
    }

    IGLMesh IGLMesh::merge(const IGLMesh& m1, const IGLMesh& m2) {
        IGLMesh result;
        SimpleIGLMesh::merge(result, m1, m2);
        result.CF.resize(m1.CF.rows()+m2.CF.rows(), 3);
        result.CF << m1.CF,
                     m2.CF;
        result.CV.resize(m1.CV.rows()+m2.CV.rows(), 3);
        result.CV << m1.CV,
                     m2.CV;
        result.NV.resize(m1.NV.rows()+m2.NV.rows(), 3);
        result.NV << m1.NV,
                     m2.NV;
        result.NF.resize(m1.NF.rows()+m2.NF.rows(), 3);
        result.NF << m1.NF,
                     m2.NF;
        return result;
    }

    #ifdef DCEL_DEFINED
    IGLMesh& IGLMesh::operator=(const Dcel& dcel) {
        clear();
        V.resize(dcel.getNumberVertices(), 3);
        F.resize(dcel.getNumberFaces(), 3);
        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
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
            CF(i,0) = c.redF(); CF(i,1) = c.greenF(); CF(i,2) = c.blueF();
            NF(i,0) = n.x(); NF(i,1) = n.y(); NF(i,2) = n.z();
            i++;
        }
        return *this;
    }
    #endif

    void IGLMesh::serialize(std::ofstream& binaryFile) const {
        Serializer::serialize(V, binaryFile);
        Serializer::serialize(F, binaryFile);
        Serializer::serialize(CF, binaryFile);
        Serializer::serialize(NV, binaryFile);
        Serializer::serialize(NF, binaryFile);
    }

    bool IGLMesh::deserialize(std::ifstream& binaryFile) {
        IGLMesh tmp;
        if (Serializer::deserialize(tmp.V, binaryFile) &&
                Serializer::deserialize(tmp.F, binaryFile) &&
                Serializer::deserialize(tmp.CF, binaryFile) &&
                Serializer::deserialize(tmp.NV, binaryFile) &&
                Serializer::deserialize(tmp.NF, binaryFile)){
            tmp.updateBoundingBox();
            *this = std::move(tmp);
            return true;
        }
        return false;
    }
}
