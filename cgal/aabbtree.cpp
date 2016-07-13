#include "aabbtree.h"
#ifdef DCEL_DEFINED
CGALInterface::AABBTree::AABBTree() {
}

CGALInterface::AABBTree::AABBTree(const Dcel& d) {


    for (Dcel::ConstVertexIterator vit = d.vertexBegin(); vit != d.vertexEnd(); ++vit){
        const Dcel::Vertex* v = *vit;
        CGALPoint pp(v->getCoordinate().x(), v->getCoordinate().y(), v->getCoordinate().z());
        vertices_points[v]= pp;
        points_vertices[pp]=v;
    }
    for (Dcel::ConstFaceIterator fit = d.faceBegin(); fit != d.faceEnd(); ++fit){
        const Dcel::Face* f = *fit;
        const Dcel::HalfEdge* he = f->getOuterHalfEdge();
        const Dcel::Vertex* v1 = he->getFromVertex();
        const Dcel::Vertex* v2 = he->getToVertex();
        const Dcel::Vertex* v3 = he->getNext()->getToVertex();
        CGALTriangle t(vertices_points[v1], vertices_points[v2], vertices_points[v3]);
        triangles_faces[t] = f;
        faces_triangles[f] = t;
        triangles.push_back(t);
    }
    tree.insert(triangles.begin(), triangles.end());
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

void CGALInterface::AABBTree::getIntersectedPrimitives(std::list<const Dcel::Face*>& outputList, const BoundingBox& b) {
    CGALBoundingBox bb(b.getMinX(), b.getMinY(), b.getMinZ(), b.getMaxX(), b.getMaxY(), b.getMaxZ());
    std::list< Tree::Primitive_id > trianglesIds;
    tree.all_intersected_primitives(bb, std::back_inserter(trianglesIds));
    for (std::list< Tree::Primitive_id >::const_iterator it = trianglesIds.begin(); it != trianglesIds.end(); ++it){
        const Tree::Primitive_id id = *it;
        const CGALTriangle t = *id;
        std::map<CGALTriangle, const Dcel::Face*, cmpCGALTriangle>::iterator mit = triangles_faces.find(t);
        assert(mit != triangles_faces.end());
        outputList.push_back(mit->second);
    }
}
#endif
