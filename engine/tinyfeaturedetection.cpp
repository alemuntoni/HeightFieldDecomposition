#include "tinyfeaturedetection.h"

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/polygon2d.h>
#include <cg3/utilities/utils.h>
#include <cg3/cgal/sdf.h>
#include <cg3/cgal/aabb_tree.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include "engine.h"
#include <common.h>

using namespace cg3;

std::vector<unsigned int> TinyFeatureDetection::sdf(const EigenMesh &m, double threshold) {
    std::vector<unsigned int> problematicFaces;
    std::vector<double> sdf = cg3::cgal::sdf::getSDFMap(m);
    for (unsigned int i = 0; i < sdf.size(); i++){
        if (sdf[i] < threshold)
            problematicFaces.push_back(i);
    }
    return problematicFaces;
}



void TinyFeatureDetection::colorSDF(EigenMesh &m, std::vector<unsigned int> &problematicFaces) {
    for (unsigned int& f : problematicFaces)
        m.setFaceColor(Color(255,0,0), f);
}

Array3D<Pointd> TinyFeatureDetection::generateGrid(const EigenMesh &m, double threshold) {
    Array3D<Pointd> grid;
    Array3D<float> distanceField;
    Engine::generateGridAndDistanceField(grid, distanceField, m, false, threshold, false);
    return grid;
}

bool TinyFeatureDetection::tinyFeatureVoxelization(const EigenMesh &hf, const Vec3& target, double threshold) {
    EigenMesh m = hf;

    //hf rotation
    Vec3 zAxis(0,0,1);
    Vec3 axis = target.cross(zAxis);
    axis.normalize();
    double dot = target.dot(zAxis);
    double angle = acos(dot);

    Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
    if (target != zAxis){
        if (target == -zAxis){
            axis = Vec3(1,0,0);
        }
        cg3::getRotationMatrix(axis, angle, r);
    }
    else {
        r = Eigen::Matrix3d::Identity();
    }
    m.rotate(r);
    m.updateBoundingBox();
    m.translate(Pointd(0,0,-m.getBoundingBox().min().z()));
    m.updateBoundingBox();

    // grid generation
    Array3D<Pointd> grid = generateGrid(m, threshold/2);

    cgal::AABBTree tree(m);
    Array3D<short int> isInside(grid.getSizeX(), grid.getSizeY(), grid.getSizeZ(), false);
    for (unsigned int i = 0; i < grid.getSizeX(); i++){
        for (unsigned int j = 0; j < grid.getSizeY(); j++){
            for (unsigned int k = 0; k < grid.getSizeZ(); k++){
                if (tree.isInside(grid(i,j,k)))
                    isInside(i,j,k) = true;
            }
        }
    }
    for (unsigned int k = 0; k < grid.getSizeZ(); k++){
        //for every x, I check the y thickness
        for (unsigned int i = 0; i < grid.getSizeX(); i++){
            unsigned int nInsideConsecutive = 0;
            for (unsigned int j = 0; j < grid.getSizeY(); j++){
                if (isInside(i,j,k)){
                    nInsideConsecutive++;
                }
                else {
                    if (nInsideConsecutive > 0 && nInsideConsecutive < 4) {
                        //tiny feature, I need to check on the height..
                        if (k < grid.getSizeZ()-2){
                            bool alsoInHeigth = false;
                            unsigned int ktmp = k+2;
                            //on the ktmp row, I'll check if is still tiny feature
                            for (unsigned int jtmp = j - nInsideConsecutive; jtmp < j; jtmp++){
                                alsoInHeigth = alsoInHeigth || isInside(i, jtmp, ktmp);
                            }
                            if (alsoInHeigth)
                                return true;
                        }
                    }
                    nInsideConsecutive = 0;
                }
            }
        }

        //for every y, I check the x thickness
        for (unsigned int j = 0; j < grid.getSizeY(); j++){
            unsigned int nInsideConsecutive = 0;
            for (unsigned int i = 0; i < grid.getSizeX(); i++){
                if (isInside(i,j,k)){
                    nInsideConsecutive++;
                }
                else {
                    if (nInsideConsecutive > 0 && nInsideConsecutive < 4) {
                        //tiny feature, I need to check on the height..
                        if (k < grid.getSizeZ()-2){
                            bool alsoInHeigth = false;
                            unsigned int ktmp = k+2;
                            //on the ktmp row, I'll check if is still tiny feature
                            for (unsigned int itmp = i - nInsideConsecutive; itmp < i; itmp++){
                                alsoInHeigth = alsoInHeigth || isInside(itmp, j, ktmp);
                            }
                            if (alsoInHeigth)
                                return true;
                        }
                    }
                    nInsideConsecutive = 0;
                }
            }
        }
    }
    return false;
}

std::set<unsigned int> TinyFeatureDetection::chartExpansion(const EigenMesh &hf, unsigned int f, std::vector<bool> &seen) {
    std::stack<unsigned int> stack;
    std::set<unsigned int> chart;
    Vec3 nf = hf.getFaceNormal(f);
    Eigen::MatrixXi fadj = EigenMeshAlgorithms::getFaceAdjacences(hf);
    stack.push(f);
    do {
        f = stack.top();
        stack.pop();
        if (!seen[f]){
            seen[f] = true;
            chart.insert(f);
            for (unsigned int ia = 0; ia < 3; ia++){
                int adj = fadj(f,ia);
                if (adj >= 0 && hf.getFaceNormal(adj) == nf){
                    stack.push(adj);
                }
            }
        }

    } while (stack.size() > 0);
    return chart;
}

std::vector<Pointd> TinyFeatureDetection::getPolygonFromChart(const EigenMesh&hf, const std::set<unsigned int>& chart){
    std::vector<std::pair<unsigned int, unsigned int> > segments;
    Eigen::MatrixXi fadj = EigenMeshAlgorithms::getFaceAdjacences(hf);
    for (unsigned int f : chart){
        for (unsigned int ia = 0; ia <3; ia++){
            int adj = fadj(f,ia);
            if (adj != -1 && chart.find(adj) == chart.end()){
                std::pair<int, int> p = hf.getCommonVertices(f, (unsigned int)adj);
                assert(p.first >= 0 && p.second >= 0);
                segments.push_back(std::pair<unsigned int, unsigned int>(p.first, p.second));
            }
        }
    }

    std::vector<Pointd> polygon;
    unsigned int first, actual;
    first = segments[0].first;
    polygon.push_back(hf.getVertex(first));
    actual = segments[0].second;
    do {
        polygon.push_back(hf.getVertex(actual));
        bool found = false;
        for (unsigned int i = 0; i < segments.size() && !found; i++){
            if (actual == segments[i].first){
                actual = segments[i].second;
                found = true;
            }
        }
        assert(found);

    } while (actual != first);


    return polygon;
}

bool TinyFeatureDetection::tinyFeaturePlane(const EigenMesh &hf, const Vec3 &target, double threshold, double& mindist) {
    std::vector<bool> seen(hf.getNumberFaces(), false);
    std::vector< std::set<unsigned int> >charts;

    for (unsigned int f = 0; f < hf.getNumberFaces(); f++) {
        Vec3 nf = hf.getFaceNormal(f);
        if (nf != target && nf != -target){
            int idf = indexOfNormal(nf);
            if (idf != -1){
                if (!seen[f])
                    charts.push_back(chartExpansion(hf, f, seen));
            }
        }
    }

    std::vector<std::vector<Pointd>> polygons;
    std::vector<Vec3> polygonNormals;

    unsigned int tmp = 0;
    for (std::set<unsigned int> &chart: charts){
        //coloring
        Vec3 n = hf.getFaceNormal(*(chart.begin()));

        //polygons
        polygons.push_back(getPolygonFromChart(hf, chart));
        polygonNormals.push_back(n);
        tmp++;
    }

    for (unsigned int i = 0; i < polygons.size(); i++){
        for (unsigned int j = 0; j < polygons[i].size()-2; j++){
            Vec3 n1 = polygons[i][j+1]-polygons[i][j];
            n1.normalize();
            Vec3 n2 = polygons[i][j+2]-polygons[i][j+1];
            n2.normalize();
            if (cg3::epsilonEqual(n1, n2)){
                polygons[i].erase(polygons[i].begin()+j+1);
                j--;
            }
        }
    }

    //genrating 2D polygons
    std::vector<std::vector<Point2Dd> > polygons2D;
    for (unsigned int i = 0; i < polygons.size(); i++){
        std::vector<Point2Dd> pol2D;
        unsigned int noid = indexOfNormal(polygonNormals[i]) % 3;
        for (unsigned int j = 0; j < polygons[i].size(); j++){
            Pointd& p = polygons[i][j];
            Point2Dd p2d;
            unsigned int o = 0;
            for (unsigned int k = 0; k < 3; k++){
                if (k != noid){
                    p2d[o] = p[k];
                    o++;
                }
            }
            pol2D.push_back(p2d);
        }
        if (! cg3::isCounterClockwise(pol2D)){
            std::reverse(pol2D.begin(), pol2D.end());
        }
        polygons2D.push_back(pol2D);
    }

    //check if opposite normals
    bool isTinyFeature = false;
    for (unsigned int i = 0; i < polygonNormals.size(); i++){
        for (unsigned int j = 0; j < polygonNormals.size(); j++){
            if (indexOfNormal(polygonNormals[i]) != indexOfNormal(polygonNormals[j]) && indexOfNormal(polygonNormals[i])%3 == indexOfNormal(polygonNormals[j])%3){
                if (std::abs(polygons[i][0][indexOfNormal(polygonNormals[i])%3] - polygons[j][0][indexOfNormal(polygonNormals[j])%3]) < threshold){
                    if (polygons[i][0][indexOfNormal(polygonNormals[i])%3] > polygons[j][0][indexOfNormal(polygonNormals[j])%3] && indexOfNormal(polygonNormals[i]) < indexOfNormal(polygonNormals[j])) {
                        if (cgal::booleans2d::doIntersect(polygons2D[i], polygons2D[j])){
                            if (!isTinyFeature){
                                mindist = std::abs(polygons[i][0][indexOfNormal(polygonNormals[i])%3] - polygons[j][0][indexOfNormal(polygonNormals[j])%3]);
                            }
                            else {
                                mindist = std::min(mindist, std::abs(polygons[i][0][indexOfNormal(polygonNormals[i])%3] - polygons[j][0][indexOfNormal(polygonNormals[j])%3]));
                            }
                            isTinyFeature = true;
                        }
                    }
                }
            }
        }
    }
    return isTinyFeature;
}
