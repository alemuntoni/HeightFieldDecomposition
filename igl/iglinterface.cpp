#include "iglinterface.h"
#include <igl/copyleft/marching_cubes.h>
#include <igl/signed_distance.h>
#include <igl/read_triangle_mesh.h>

/**
 * @brief generateGridAndDistanceField
 *
 * The bounding box  must be sufficiently larger to build a Grid with a step gridUnit*gridUnit*gridUnit (default = 2).
 * If integer is true, a grid in only integer values will be created (gridUnit will be casted to int).
 *
 * The Grid generated will have a border of two points guaranteed to be outside the mesh.
 *
 * Every point of the grid will be stored on 3D Array grid, and the distance field will be stored on the parallel 3D array distanceField.
 */
template <typename T>
void IGLInterface::generateGridAndDistanceField(Array3D<Pointd>& grid, Array3D<T> &distanceField, const SimpleIGLMesh &m, double gridUnit, bool integer) {
    assert(gridUnit > 0);
    // Bounding Box
    Eigen::RowVector3d Vmin, Vmax;
    m.getBoundingBox(Vmin, Vmax);

    // create grid GV
    Eigen::RowVector3d border((int)gridUnit*5, (int)gridUnit*5, (int)gridUnit*5);
    Eigen::RowVector3d nGmin;
    Eigen::RowVector3d nGmax;
    if (integer) {
        Eigen::RowVector3i Gmini = (Vmin).cast<int>() - border.cast<int>();
        Eigen::RowVector3i Gmaxi = (Vmax).cast<int>() + border.cast<int>();
        nGmin = Gmini.cast<double>();
        nGmax = Gmaxi.cast<double>();
        gridUnit = (int)gridUnit;
        assert(gridUnit > 0);
    }
    else {
        nGmin = Vmin - border;
        nGmax = Vmax + border; //bounding box of the Grid
    }
    Eigen::RowVector3i res = (nGmax.cast<int>() - nGmin.cast<int>())/2; res(0)+=1; res(1)+=1; res(2)+=1;
    Eigen::MatrixXd GV(res(0)*res(1)*res(2),3);

    grid.resize(res(0), res(1), res(2));
    distanceField.resize(res(0), res(1), res(2));

    int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
    for (int i = 0; i < res(0); ++i){
        yi = nGmin(1);
        for (int j = 0; j < res(1); ++j){
            zi = nGmin(2);
            for (int k = 0; k < res(2); ++k){
                GV.row(k+res(2)*(j + res(1)*i)) = Eigen::RowVector3i(xi,yi,zi).cast<double>();
                zi+=gridUnit;
            }
            yi+=gridUnit;
        }
        xi += gridUnit;
    }

    // compute values
    Eigen::VectorXd S = m.getSignedDistance(GV);

    for (int i = 0; i < res(0); i++){
        for (int j = 0; j < res(1); j++){
            for (int k = 0; k < res(2); k++){
                grid(i,j,k) = Pointd(GV.row(k+res(2)*(j + res(1)*i)));
                distanceField(i,j,k) = S(k+res(2)*(j + res(1)*i));
            }
        }
    }
}

IGLInterface::SimpleIGLMesh IGLInterface::makeBox(const BoundingBox &bb, double minimumEdge){
    IGLInterface::SimpleIGLMesh box;
    if (minimumEdge <= 0) {
        std::vector<Pointd> extremes;
        bb.getExtremes(extremes);
        box.resizeVertices(8);
        for (unsigned int i = 0; i < 8; i++){
            box.setVertex(i, extremes[i].x(), extremes[i].y(), extremes[i].z());
        }
        box.resizeFaces(12);
        box.setFace(0, 0,1,2);
        box.setFace(1, 0,2,3);
        box.setFace(2, 2,1,5);
        box.setFace(3, 2,5,6);
        box.setFace(4, 5,1,0);
        box.setFace(5, 5,0,4);
        box.setFace(6, 6,5,4);
        box.setFace(7, 6,4,7);
        box.setFace(8, 7,4,0);
        box.setFace(9, 7,0,3);
        box.setFace(10, 7,3,2);
        box.setFace(11, 7,2,6);
    }
    else {
        unsigned int nSubdX, nSubdY, nSubdZ;
        //minimumEdge /= std::sqrt(2.f);
        nSubdX = bb.getLengthX() / minimumEdge; nSubdX++;
        nSubdY = bb.getLengthY() / minimumEdge; nSubdY++;
        nSubdZ = bb.getLengthZ() / minimumEdge; nSubdZ++;
        double edgeLengthX = bb.getLengthX() / nSubdX;
        double edgeLengthY = bb.getLengthY() / nSubdY;
        double edgeLengthZ = bb.getLengthZ() / nSubdZ;
        //creation vertices
        std::map<Pointi, Pointd> vertices;
        double x, y, z;
        unsigned int i, j , k;

        // fix z - k = 0;
        k = 0; z = bb.getMinZ();
        x = bb.getMinX();
        for (i = 0; i <= nSubdX; i++){
            y = bb.getMinY();
            for (j = 0; j <= nSubdY; j++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                y+=edgeLengthY;
            }
            x+=edgeLengthX;
        }
        // fix z - k = nSubdZ;
        k = nSubdZ; z = bb.getMaxZ();
        x = bb.getMinX();
        for (i = 0; i <= nSubdX; i++){
            y = bb.getMinY();
            for (j = 0; j <= nSubdY; j++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                y+=edgeLengthY;
            }
            x+=edgeLengthX;
        }
        // fix y - j = 0;
        j = 0; y = bb.getMinY();
        x = bb.getMinX();
        for (i = 0; i <= nSubdX; i++){
            z = bb.getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            x+=edgeLengthX;
        }
        // fix y - j = nSubdY;
        j = nSubdY; y = bb.getMaxY();
        x = bb.getMinX();
        for (i = 0; i <= nSubdX; i++){
            z = bb.getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            x+=edgeLengthX;
        }
        // fix x - i = 0;
        i = 0; x = bb.getMinX();
        y = bb.getMinY();
        for (j = 0; j <= nSubdY; j++){
            z = bb.getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            y+=edgeLengthY;
        }
        // fix x - i = nSubdX;
        i = nSubdX; x = bb.getMaxX();
        y = bb.getMinY();
        for (j = 0; j <= nSubdY; j++){
            z = bb.getMinZ();
            for (k = 0; k <= nSubdZ; k++){
                Pointi pi(i,j,k);
                Pointd pd(x,y,z);
                vertices[pi] = pd;
                z+=edgeLengthZ;
            }
            y+=edgeLengthY;
        }

        std::map<Pointi, int> indices;
        int index = 0;
        box.resizeVertices(vertices.size());
        for (std::pair<Pointi, Pointd> pair : vertices) {
            indices[pair.first] = index;
            box.setVertex(index, pair.second.x(), pair.second.y(), pair.second.z());
            index++;

        }

        //triangles
        // fix z - k = 0;
        k = 0;
        for (i = 0; i < nSubdX; i++){
            for (j = 0; j < nSubdY; j++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j+1,k);
                Pointi pi4(i,j+1,k);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix z - k = nSubdZ;
        k = nSubdZ;
        for (i = 0; i < nSubdX; i++){
            for (j = 0; j < nSubdY; j++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j+1,k);
                Pointi pi4(i,j+1,k);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
        // fix y - j = 0;
        j = 0;
        for (i = 0; i < nSubdX; i++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
        // fix y - j = nSubdY;
        j = nSubdY;
        for (i = 0; i < nSubdX; i++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i+1,j,k);
                Pointi pi3(i+1,j,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix x - i = 0;
        i = 0;
        for (j = 0; j < nSubdY; j++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i,j+1,k);
                Pointi pi3(i,j+1,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i2, i1, i3);
                box.addFace(i3, i1, i4);
            }
        }
        // fix x - i = nSubdX;
        i = nSubdX;
        for (j = 0; j < nSubdY; j++){
            for (k = 0; k < nSubdZ; k++){
                Pointi pi1(i,j,k);
                Pointi pi2(i,j+1,k);
                Pointi pi3(i,j+1,k+1);
                Pointi pi4(i,j,k+1);
                assert(indices.find(pi1) != indices.end());
                assert(indices.find(pi2) != indices.end());
                assert(indices.find(pi3) != indices.end());
                assert(indices.find(pi4) != indices.end());
                int i1 = indices[pi1], i2 = indices[pi2], i3 = indices[pi3], i4 = indices[pi4];
                box.addFace(i1, i2, i3);
                box.addFace(i1, i3, i4);
            }
        }
    }
    return box;
}

bool IGLInterface::isABox(const IGLInterface::SimpleIGLMesh& simpleIGLMesh) {
    IGLInterface::IGLMesh mesh(simpleIGLMesh);

}

template void IGLInterface::generateGridAndDistanceField<double>(Array3D<Pointd>& grid, Array3D<double> &distanceField, const SimpleIGLMesh &m, double gridUnit, bool integer);
template void IGLInterface::generateGridAndDistanceField<float>(Array3D<Pointd>& grid, Array3D<float> &distanceField, const SimpleIGLMesh &m, double gridUnit, bool integer);
template void IGLInterface::generateGridAndDistanceField<int>(Array3D<Pointd>& grid, Array3D<int> &distanceField, const SimpleIGLMesh &m, double gridUnit, bool integer);


