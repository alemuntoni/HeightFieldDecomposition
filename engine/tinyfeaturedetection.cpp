#include "tinyfeaturedetection.h"

#include <cgal/cgalsdf.h>
#include <cgal/aabbtree.h>
#include "engine.h"

std::vector<unsigned int> TinyFeatureDetection::sdf(const EigenMesh &m, double threshold) {
    std::vector<unsigned int> problematicFaces;
    std::vector<double> sdf = CGALInterface::SDF::getSDFMap(m);
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
        Common::getRotationMatrix(axis, angle, r);
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

    CGALInterface::AABBTree tree(m);
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
