#ifndef UTILS_H
#define UTILS_H

#include "iglinterface.h"
#include "iglmesh.h"

namespace IGLInterface{
    inline IGLInterface::SimpleIGLMesh makeBox(const BoundingBox &bb, double minimumEdge = -1){
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

    inline IGLInterface::SimpleIGLMesh makeBox(const Pointd &min, const Pointd &max, double minimumEdge = -1){
        return makeBox(BoundingBox(min, max), minimumEdge);
    }
}

#endif // UTILS_H
