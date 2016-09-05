#include "aabbtree.h"

CGALInterface::AABBTree::AABBTree() {
}

CGALInterface::AABBTree::AABBTree(const CGALInterface::AABBTree& other) : forDistanceQueries(other.forDistanceQueries), triangles(other.triangles) {
    #ifdef DCEL_DEFINED
    mapDcelVerticesToCgalPoints = other.mapDcelVerticesToCgalPoints;
    mapCgalTrianglesToDcelFaces = other.mapCgalTrianglesToDcelFaces;
    #endif
    tree.insert(triangles.begin(), triangles.end());

    if (forDistanceQueries)
        tree.accelerate_distance_queries();
}

#ifdef DCEL_DEFINED
CGALInterface::AABBTree::AABBTree(const Dcel& d, bool forDistanceQueries) : forDistanceQueries(forDistanceQueries){
    for (Dcel::ConstVertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
        const Dcel::Vertex* v = *vit;
        CGALPoint pp(v->getCoordinate().x(), v->getCoordinate().y(), v->getCoordinate().z());
        mapDcelVerticesToCgalPoints[v]= pp;
    }
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        const Dcel::HalfEdge* he = f->getOuterHalfEdge();
        const Dcel::Vertex* v1 = he->getFromVertex();
        const Dcel::Vertex* v2 = he->getToVertex();
        const Dcel::Vertex* v3 = he->getNext()->getToVertex();
        CGALTriangle t(mapDcelVerticesToCgalPoints[v1], mapDcelVerticesToCgalPoints[v2], mapDcelVerticesToCgalPoints[v3]);
        mapCgalTrianglesToDcelFaces[t] = f;
        triangles.push_back(t);
    }
    tree.insert(triangles.begin(), triangles.end());

    if (forDistanceQueries)
        tree.accelerate_distance_queries();
}
#endif

CGALInterface::AABBTree&CGALInterface::AABBTree::operator=(const CGALInterface::AABBTree& other) {
    forDistanceQueries = other.forDistanceQueries;
    triangles = other.triangles;
    #ifdef DCEL_DEFINED
    mapDcelVerticesToCgalPoints = other.mapDcelVerticesToCgalPoints;
    mapCgalTrianglesToDcelFaces = other.mapCgalTrianglesToDcelFaces;
    #endif
    tree.clear();
    tree.insert(triangles.begin(), triangles.end());

    if (forDistanceQueries)
        tree.accelerate_distance_queries();
    return *this;
}

int CGALInterface::AABBTree::getNumberIntersectedPrimitives(const Pointd& p1, const Pointd& p2) {
    CGALPoint pa(p1.x(), p1.y(), p1.z());
    CGALPoint pb(p2.x(), p2.y(), p2.z());
    CGALRay ray_query(pa,pb);
    return tree.number_of_intersected_primitives(ray_query);
}

int CGALInterface::AABBTree::getNumberIntersectedPrimitives(const BoundingBox& b) {
    CGALBoundingBox bb(b.getMinX(), b.getMinY(), b.getMinZ(), b.getMaxX(), b.getMaxY(), b.getMaxZ());
    return tree.number_of_intersected_primitives(bb);
}

double CGALInterface::AABBTree::getSquaredDistance(const Pointd& p) {
    CGALPoint query(p.x(), p.y(), p.z());
    return tree.squared_distance(query);
}

Pointd CGALInterface::AABBTree::getNearestPoint(const Pointd& p) {
    CGALPoint query(p.x(), p.y(), p.z());
    CGALPoint closest = tree.closest_point(query);
    return Pointd(closest.x(), closest.y(), closest.z());
}

#ifdef DCEL_DEFINED
void CGALInterface::AABBTree::getIntersectedDcelFaces(std::list<const Dcel::Face*>& outputList, const BoundingBox& b) {
    CGALBoundingBox bb(b.getMinX(), b.getMinY(), b.getMinZ(), b.getMaxX(), b.getMaxY(), b.getMaxZ());
    std::list< Tree::Primitive_id > trianglesIds;
    tree.all_intersected_primitives(bb, std::back_inserter(trianglesIds));
    for (std::list< Tree::Primitive_id >::const_iterator it = trianglesIds.begin(); it != trianglesIds.end(); ++it){
        const Tree::Primitive_id id = *it;
        const CGALTriangle t = *id;
        std::map<CGALTriangle, const Dcel::Face*, cmpCGALTriangle>::iterator mit = mapCgalTrianglesToDcelFaces.find(t);
        assert(mit != mapCgalTrianglesToDcelFaces.end());
        outputList.push_back(mit->second);
    }
}

void CGALInterface::AABBTree::getCompletelyContainedDcelFaces(std::list<const Dcel::Face*>& outputList, const BoundingBox& b) {
    getIntersectedDcelFaces(outputList, b);

    std::list<const Dcel::Face*>::iterator i = outputList.begin();
    while (i != outputList.end()) {
        const Dcel::Face* f = *i;
        Pointd p1 = f->getVertex1()->getCoordinate(), p2 = f->getVertex2()->getCoordinate(), p3 = f->getVertex3()->getCoordinate();

        if (!b.isIntern(p1) || !b.isIntern(p2) || !b.isIntern(p3)) {
            i =outputList.erase(i);
        }
        else ++i;
    }
}

const Dcel::Face* CGALInterface::AABBTree::getNearestDcelFace(const Pointd& p) {
    CGALPoint query(p.x(), p.y(), p.z());
    AABB_triangle_traits::Point_and_primitive_id ppid = tree.closest_point_and_primitive(query);
    const Tree::Primitive_id tp = ppid.second;
    const CGALTriangle t = *tp;
    std::map<CGALTriangle, const Dcel::Face*, cmpCGALTriangle>::iterator mit = mapCgalTrianglesToDcelFaces.find(t);
    assert(mit != mapCgalTrianglesToDcelFaces.end());
    return mit->second;

}
#endif
