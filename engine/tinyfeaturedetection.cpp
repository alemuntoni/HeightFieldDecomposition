#include "tinyfeaturedetection.h"

#include <cg3/geometry/transformations.h>
#include <cg3/geometry/2d/utils2d.h>
#include <cg3/utilities/utils.h>
#include <cg3/libigl/face_adjacences.h>
#include <cg3/meshes/eigenmesh/algorithms/eigenmesh_algorithms.h>
#include "engine.h"
#include <common.h>

using namespace cg3;

void TinyFeatureDetection::colorSDF(EigenMesh &m, std::vector<unsigned int> &problematicFaces) {
    for (unsigned int& f : problematicFaces)
        m.setFaceColor(Color(255,0,0), f);
}

std::set<unsigned int> TinyFeatureDetection::chartExpansion(const EigenMesh &hf, unsigned int f, std::vector<bool> &seen) {
    std::stack<unsigned int> stack;
    std::set<unsigned int> chart;
    Vec3 nf = hf.getFaceNormal(f);
    Eigen::MatrixXi fadj = libigl::getFaceAdjacences(hf);
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
    Eigen::MatrixXi fadj = libigl::getFaceAdjacences(hf);
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
        if (! cg3::isPolygonCounterClockwise(pol2D)){
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
                        if (int2d(polygons2D[i], polygons2D[j])){
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
