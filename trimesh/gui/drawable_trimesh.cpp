/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#include "drawable_trimesh.h"

DrawableTrimesh::DrawableTrimesh() : Trimesh<double>() {
    init();
}

DrawableTrimesh::DrawableTrimesh(const Trimesh<double> &t) : Trimesh<double>(t) {
    init();
}

DrawableTrimesh::DrawableTrimesh(const char *filename) : Trimesh<double>(filename) {
    init();
}

void DrawableTrimesh::init() {
    DrawableMesh::init();
    setTriangleColor(0.1, 0.8, 0.1);
    setVertexColor(0.1, 0.8, 0.1);
}

void DrawableTrimesh::clear() {
    Trimesh<double>::clear();
    vertexColors.clear();
    triangleColors.clear();
}

void DrawableTrimesh::draw() const {
    DrawableMesh::draw(coords.size()/3, tris.size()/3, coords.data(), tris.data(), vertexNormals.data(), vertexColors.data(), triangleNormals.data(), triangleColors.data());
}

Pointd DrawableTrimesh::sceneCenter() const {
    Pointd c = bbox.center();
    return Pointd(c.x(), c.y(), c.z());
}

double DrawableTrimesh::sceneRadius() const {
    return bbox.diag();
}

void DrawableTrimesh::setVertexColor(float r, float g, float b) {
    vertexColors.resize(numVertices()*3);
    for(int i=0; i<(int)vertexColors.size(); i+=3) {
        vertexColors[i + 0] = r;
        vertexColors[i + 1] = g;
        vertexColors[i + 2] = b;
    }
}

void DrawableTrimesh::setTriangleColor(float r, float g, float b) {
    triangleColors.resize(numTriangles()*3);
    for(int i=0; i<(int)triangleColors.size(); i+=3) {
        triangleColors[i + 0] = r;
        triangleColors[i + 1] = g;
        triangleColors[i + 2] = b;
    }
}
