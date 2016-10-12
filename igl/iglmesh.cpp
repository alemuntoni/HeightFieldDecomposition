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

    BoundingBox IGLMesh::getBoundingBox() {
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

    Eigen::MatrixXd IGLMesh::getVerticesColorMatrix() const {
        return CV;
    }

    bool IGLMesh::readFromPly(const std::string &filename) {
        typedef boost::char_separator<char>     CharSeparator;
        typedef boost::tokenizer<CharSeparator> Tokenizer;
        typedef Tokenizer::iterator             TokenizerIterator;

        CharSeparator spaceSeparator(" ");

        int vnum = -1, fnum = -1;

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "ERROR : " << __FILE__ << ", line " << __LINE__ << " : load_PLY() : couldn't open input file " << filename << std::endl;
            return false;
        }

        std::string line;
        unsigned found;

        //Retrieve informations from headers
        while(getline(file, line)) {
            if (line.find("element vertex") != std::string::npos) {
                found = line.find_last_of(" ");
                vnum = atoi(line.substr(found+1).c_str());
            }
            if (line.find("element face") != std::string::npos) {
                found = line.find_last_of(" ");
                fnum = atoi(line.substr(found+1).c_str());
            }
            if (line.find("end_header") != std::string::npos)
                break;
        }

        assert(vnum >= 0 && "Ply file with no vertices number.");
        assert(fnum >= 0 && "Ply file with no faces number.");

        //std::cout << vnum << " " << fnum << endl;

        int vi = 0, fi = 0;

        V.resize(vnum,3);
        CV.resize(vnum,3);
        F.resize(fnum,3);
        while(std::getline(file,line))
        {
            Tokenizer spaceTokenizer(line, spaceSeparator);

            if (spaceTokenizer.begin() == spaceTokenizer.end()) continue;
            TokenizerIterator token = spaceTokenizer.begin();

            if (vi < vnum) {
                std::string x = *(token);
                std::string y = *(++token);
                std::string z = *(++token);
                std::string r = *(++token);
                std::string g = *(++token);
                std::string b = *(++token);
                int ri, gi, bi;

                std::istringstream xstr(x), ystr(y), zstr(z), rstr(r), gstr(g), bstr(b);
                xstr >> V(vi,0);
                ystr >> V(vi,1);
                zstr >> V(vi,2);
                rstr >> ri;
                gstr >> gi;
                bstr >> bi;
                CV(vi,0) = (double) ri/256;
                CV(vi,1) = (double) gi/256;
                CV(vi,2) = (double) bi/256;
                vi++;
            }
            else if (fi < fnum){
                std::string x = *(++token);
                std::string y = *(++token);
                std::string z = *(++token);
                std::istringstream xstr(x), ystr(y), zstr(z);

                xstr >> F(fi,0);
                ystr >> F(fi,1);
                zstr >> F(fi,2);
                fi++;
            }

        }
        file.close();

        CF = Eigen::MatrixXd::Constant(F.rows(), 3, 0.5);
        NV.resize(V.rows(), 3);
        NF.resize(F.rows(), 3);
        igl::per_face_normals(V,F,NF);
        igl::per_vertex_normals(V,F,NV);
        BBmin = V.colwise().minCoeff();
        BBmax = V.colwise().maxCoeff();
        return true;

    }

    bool IGLMesh::saveOnPly(const std::string &filename)
    {
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
         <<"end_header"<<std::endl;
        for(unsigned int i=0;i<numV;++i)
        {
            f<<V(i,0)<<" "<<V(i,1)<<" "<<V(i,2)<<" "<<(int)(CV(i,0)*255)<<" "<<(int)(CV(i,1)*255)<<" "<<(int)(CV(i,2)*255)<<std::endl;
        }

        for(unsigned int i=0;i<numF;++i)
        {
            f<< "3 " << F(i,0)<<" "<<F(i,1)<<" "<<F(i,2)<<std::endl;
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

    void IGLMesh::difference(IGLMesh& result, const IGLMesh& m1, const IGLMesh& m2) {
        SimpleIGLMesh sres;
        SimpleIGLMesh::difference(sres, SimpleIGLMesh(m1.V, m1.F), SimpleIGLMesh(m2.V, m2.F));
        result = IGLMesh(sres);
        result.CF = Eigen::MatrixXd::Constant(result.F.rows(), 3, 0.5);
        result.NV.resize(result.V.rows(), 3);
        result.NF.resize(result.F.rows(), 3);
        result.updateVertexAndFaceNormals();
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
    #endif

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

    void IGLMesh::deserialize(std::ifstream& binaryFile) {
        Serializer::deserialize(V, binaryFile);
        Serializer::deserialize(F, binaryFile);
        Serializer::deserialize(CF, binaryFile);
        Serializer::deserialize(NV, binaryFile);
        Serializer::deserialize(NF, binaryFile);
        updateBoundingBox();
    }
}
