#include "polylinesCheck.h"
#include <cg3/geometry/transformations.h>
#include <cg3/libigl/face_adjacences.h>

#ifdef MULTI_LABEL_OPTIMIZATION_INCLUDED
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <GCoptimization.h>
#pragma GCC diagnostic pop
#endif //__GNUC__
#endif

using namespace std;
using namespace cg3;
using namespace CGAL;

#define SMALL_NUM   0.0000000001 // anything that avoids division overflow

PolylinesCheck::PolylinesCheck(){
}

void PolylinesCheck::minMaxPoints (const Mesh &mesh, int selection){
    Point3 i;
    min = mesh.point((vertex_descriptor) 0);
    max = mesh.point((vertex_descriptor) 0);
    MapPoints map = mesh.points();

    switch(selection){

        case(0):
            for(vertex_descriptor vd : mesh.vertices()){
                i=map[vd];
                if(i.x() < min.x()){
                    min = i;
                }

                if(i.x() > max.x()){
                    max = i;
                }
            }
            break;
        case(1):
            for(vertex_descriptor vd : mesh.vertices()){
                i=map[vd];
                if(i.y() < min.y()){
                    min = i;
                }

                if(i.y() > max.y()){
                    max = i;
                }
            }
            break;
        case(2):
            for(vertex_descriptor vd : mesh.vertices()){
                i=map[vd];
                if(i.z() < min.z()){
                    min = i;
                }

                if(i.z() > max.z()){
                    max = i;
                }
            }
            break;
    }
}

void PolylinesCheck::setMin(Pointd& minP1){
    minP = minP1;
}

void PolylinesCheck::setMax(Pointd& maxP1){
    maxP = maxP1;
}

Pointd PolylinesCheck::getMin(){
    return minP;
}

Pointd PolylinesCheck::getMax(){
    return maxP;
}

void PolylinesCheck::setPoly(const Mesh& mesh, Vec3& norm){
    poly = cg3::cgal::slicer::getPolylines(mesh, norm, d);
}

void PolylinesCheck::convertTo2dPlane (){

    //Rotation of the coordinates
    Vec3 zAxis(0,0,1);
    Vec3 axis = -(normalplane.cross(zAxis));
    axis.normalize();
    double dot = normalplane.dot(zAxis);
    double angle = acos(dot);

    double r[3][3] = {{0}};
    if (normalplane != zAxis){
        if (normalplane == -zAxis){
            axis = Vec3(1,0,0);
        }
        cg3::getRotationMatrix(axis, angle, r);
    }
    else {
        r[0][0] = r[1][1] = r[2][2] = 1;
    }

    //rotate points and make 2D polygon
    for (unsigned int i = 0; i < poly[0].size(); ++i){
        Pointd a = poly[0][i];
        Pointd p1(a.x() * r[0][0] + a.y() * r[1][0] +a.z() * r[2][0],
                  a.x() * r[0][1] + a.y() * r[1][1] +a.z() * r[2][1],
                  a.x() * r[0][2] + a.y() * r[1][2] +a.z() * r[2][2]);
        Point2 p(p1.x(), p1.y());
        poly2d.push_back(p);
    }
}

bool PolylinesCheck::checkPolyline (){
    convertTo2dPlane();
    return true;
}

void PolylinesCheck::setNormal(Vec3& normal){
    normalplane = normal;
}

void PolylinesCheck::setD(){
    d = (-max.x()*normalplane.x() -max.y()*normalplane.y()-max.z()*normalplane.z());
}

void PolylinesCheck::checkIntersect(DrawableEigenMesh* meshEigenOrigin, Pointd p0, Pointd p1, int selection){
    int nFaces = meshEigenOrigin->getNumberFaces();
    int inter=0;
    //bool inter = false;
    bool flag = false;
    for(int i=0; i<nFaces;i++){
        Pointi f = meshEigenOrigin->getFace(i);
        inter = intersect3D_RayTriangle(p0, p1, meshEigenOrigin->getVertex(f.x()),
                                                meshEigenOrigin->getVertex(f.y()),
                                                meshEigenOrigin->getVertex(f.z()));
        if(inter == 1){
            if(flag){
                switch (selection) {
                case 0:
                    if(minP.x() > I.x()){
                        minP = I;
                    }
                    if(maxP.x() < I.x()){
                        maxP = I;
                    }
                    break;
                case 1:
                    if(minP.y() > I.y()){
                        minP = I;
                    }
                    if(maxP.y() < I.y()){
                        maxP = I;
                    }
                    break;
                case 2:
                    if(minP.z() > I.z()){
                        minP = I;
                    }
                    if(maxP.z() < I.z()){
                        maxP = I;
                    }
                    break;
                default:
                    break;
                }
            } else {
                minP = I;
                maxP = I;
                flag = true;
            }
        }
    }
}

int PolylinesCheck::intersect3D_RayTriangle( Pointd p0, Pointd p1, Pointd v0, Pointd v1, Pointd v2){

    //intersect3D_RayTriangle(): find the 3D intersection of a ray with a triangle
    //    Input:  a ray R, and a triangle T
    //    Output: *I = intersection point (when it exists)
    //    Return: -1 = triangle is degenerate (a segment or point)
    //             0 =  disjoint (no intersect)
    //             1 =  intersect in unique point I1
    //             2 =  are in the same plane

    Vec3    e1, e2, n;                      // triangle vectors
    Vec3    dir, w0, w;                     // ray vectors
    double     r, a, b;                     // params to calc ray-plane intersect
    // get triangle edge vectors and plane normal
    e1 = v1 - v0;
    e2 = v2 - v0;
    n = e1.cross(e2);                       // cross product
    if (n == (Vec3)0)                       // triangle is degenerate
        return -1;                          // do not deal with this case

    dir = p1 - p0;                          // ray direction vector
    w0 = p0 - v0;
    a = -n.dot(w0);
    b = n.dot(dir);
    if (fabs(b) < SMALL_NUM) {              // ray is  parallel to triangle plane
        if (a == 0)                         // ray lies in triangle plane
            return 2;
        else return 0;                      // ray disjoint from plane
    }

    // get intersect point of ray with triangle plane
    r = a / b;
    if (r < 0.0)                            // ray goes away from triangle
        return 0;                           // => no intersect
    // for a segment, also test if (r > 1.0) => no intersect

    I = p0 + Pointd(r*dir.x(), r*dir.y(), r*dir.z());                   // intersect point of ray and plane

    // is I inside T?
    float    uu, uv, vv, wu, wv, D;
    uu = e1.dot(e1);
    uv = e1.dot(e2);
    vv = e2.dot(e2);
    w = I - v0;
    wu = w.dot(e1);
    wv = w.dot(e2);
    D = uv * uv - uu * vv;

    // get and test parametric coords
    float s, t;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)                 // I is outside T
        return 0;
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)           // I is outside T
        return 0;

    return 1;                               // I is in T
}

void PolylinesCheck::check(DrawableEigenMesh *meshEigenOrigin, int color, int indexPlane){

    meshPoly = *meshEigenOrigin;

    cg3::cgal::AABBTree eigenTree(*meshEigenOrigin);
    QColor c;
    Pointi f;
    VectI blackList;
    Vec3 e1, e2, e3, n;
    int face;
    double max = meshEigenOrigin->getBoundingBox().maxY()+50;
    double min = meshEigenOrigin->getBoundingBox().minY()-50;
    //int indexNotVisibleVector = checker.size()-1;

    for(unsigned int i = 0; i < checker.getSizeY(); i++){
        /*if(notVisibleFace.size() > 0){
            c.setHsv(0,255,255);
            for(int j : notVisibleFace){
                checker[indexPlane][j]=1;
                meshEigenOrigin->setFaceColor(c.redF(), c.greenF(),c.blueF(),j);
            }
        }*/
        if(checker(indexPlane, i) != 1 && !(std::find(notVisibleFace.begin(),
                                                      notVisibleFace.end(), i)
                                            != notVisibleFace.end())){
            f = meshEigenOrigin->getFace(i);
            e1 = meshEigenOrigin->getVertex(f.x());
            e2 = meshEigenOrigin->getVertex(f.y());
            e3 = meshEigenOrigin->getVertex(f.z());

            Pointd bar((e1+e2+e3)/3);
            //cerco le intersezioni della retta passante per la i-esima faccia
            eigenTree.getIntersectEigenFaces(Pointd(bar.x(), max, bar.z()), Pointd(bar.x(), min, bar.z()), blackList);
            //Prendo quella che si trova più in alto
            face = serchMaxY(blackList,meshEigenOrigin);
            if(checker(indexPlane, face) != 1){
                checker(indexPlane,face) = 1;
                c.setHsv(color, 255,255);
                meshEigenOrigin->setFaceColor(c.redF(), c.greenF(),c.blueF(),face);
            }
            //Prendo quella che si trova più in basso
            face = serchMinY(blackList,meshEigenOrigin);
            if(checker(indexPlane,face) != 1){
                checker(indexPlane,face) = 1;
                c.setHsv(120+color, 255,255);
                meshEigenOrigin->setFaceColor(c.redF(), c.greenF(),c.blueF(),face);
            }
            //Svuoto la lista
            blackList.clear();
        }
    }
    indexPlane++;
}

void PolylinesCheck::rotatePoint(Eigen::Matrix3d rotation, Pointd p){
    minP.rotate(rotation, p);
    maxP.rotate(rotation, p);
}

int PolylinesCheck::serchMaxY (std::vector<int> lista, DrawableEigenMesh *meshEigenOrigin){
    int max = lista.front();
        for(int i : lista){
            if(meshEigenOrigin->getVertex(meshEigenOrigin->getFace(i).y()).y() > meshEigenOrigin->getVertex(meshEigenOrigin->getFace(max).y()).y()){
                max = i;
            }
        }
    return max;
}

int PolylinesCheck::serchMinY (std::vector<int> lista, DrawableEigenMesh *meshEigenOrigin){
        int min = lista.front();
        for(int i : lista){
            if(meshEigenOrigin->getVertex(meshEigenOrigin->getFace(i).y()).y() < meshEigenOrigin->getVertex(meshEigenOrigin->getFace(min).y()).y()){
                min = i;
            }
        }
    return min;
}

void PolylinesCheck::setCheckerDimension (int nplane, int dimension){
    checker.resize(nplane+=1, dimension);
}

void PolylinesCheck::resetChecker(){
    checker.clear();
}

Array2D<int> PolylinesCheck::getChecker(){
    return checker;
}

void PolylinesCheck::searchNoVisibleFace (){

    for(unsigned int i = 0; i < checker.getSizeY();i++){
        if(checker(0,i)!=1)
            notVisibleFace.push_back(i);
    }
}

VectI PolylinesCheck::getNotVisibleFace(){
    return notVisibleFace;
}

void PolylinesCheck::minimizeProblem(){
    int nOrientation = checker.getSizeX();
    int nTriangles = checker.getSizeY();

    //int conto = 0;
    /*for(int i = 0; i < nTriangles; i++){
        conto = 0;
        for(int j = 0; j < nOrientation; j++){
            conto += checker[j][i];
        }
        if (conto == 0){
            cout << "dio cane dio merda " <<  i << endl;
        }
    }*/

    #ifdef GUROBI_DEFINED
    try{
        GRBEnv env = GRBEnv();

        GRBModel model = GRBModel(env);

        GRBVar *orientation = model.addVars(nOrientation, GRB_BINARY);

        //creo le variabili o e t per gli orientamenti e per i triangoli
        for (int i = 0; i < nOrientation; i++) {
          ostringstream vname;
          vname << "o" << i;
          orientation[i].set(GRB_StringAttr_VarName, vname.str());
        }

        model.update();

        for(int i = 0; i < nTriangles; i++){
            GRBLinExpr sum = 0;
            for(int j = 0 ; j < nOrientation ; j++){
                sum+=orientation[j]*checker(j,i);
            }
            model.addConstr(sum >= 1);
        }

        GRBLinExpr expr = 0;

        for (int j = 0; j < nOrientation; j++) {
            expr += orientation[j];
        }

        model.setObjective(expr, GRB_MINIMIZE);

        model.optimize();

        for (int i = 0; i < nOrientation; i++) {
            cout << orientation[i].get(GRB_StringAttr_VarName) << " "
                 << orientation[i].get(GRB_DoubleAttr_X) << endl;
            if(orientation[i].get(GRB_DoubleAttr_X) == 1) orientationSelected.push_back(i);
        }
    }
    catch (GRBException e) {
      cout << "Error code = " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
    }
    catch (...) {
      cout << "Exception during optimization" << endl;
    }
    #endif
}

void PolylinesCheck::updateChecker(bool updateCheckerFlag){

    if(!updateCheckerFlag){
        checker.conservativeResize(checker.getSizeX()+1, checker.getSizeY());
    }

    int nOrientation = checker.getSizeX();
    for(unsigned int i = 0; i < notVisibleFace.size(); i++){
        int id = notVisibleFace[i];
        checker(nOrientation-1,id) = 1;
    }
}

void PolylinesCheck::resetMatrixCheck(){
    for(unsigned int i = 0; i<checker.getSizeY(); i++)
        checker(0,i) = 0;
}

void PolylinesCheck::serchUniqueTriangoForOrientation(){
    int count, id;

    uniqueTriangle.resize(orientationSelected.size());

    for(unsigned int i = 0; i < checker.getSizeY(); i++){
        count = 0;
        id = 0;
        for(unsigned int j = 0; j < orientationSelected.size(); j++){
            count+=checker(orientationSelected[j],i);
            if(checker(orientationSelected[j],i) == 1) {
                id = j;
            }
        }
        if(count == 1) {
            uniqueTriangle[id].push_back(i);
        }
    }
}

MatrixI PolylinesCheck::getUniqueTriangle() const{
    return uniqueTriangle;
}

void PolylinesCheck::setUniqueTriangle(const MatrixI &value){
    uniqueTriangle = value;
}

VectI PolylinesCheck::getOrientationSelected() const{
    return orientationSelected;
}

void PolylinesCheck::addFaceExlude(unsigned int i){
    notVisibleFace.push_back(i);
}
