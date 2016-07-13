#ifndef AABBTREE_H
#define AABBTREE_H

#include "cgalinterface.h"
#include "../common/bounding_box.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

namespace CGALInterface {
    #ifdef DCEL_DEFINED
    class AABBTree
    {
        public:
            AABBTree();
            AABBTree(const Dcel &d);

            int getNumberIntersectedPrimitives(const Pointd& p1, const Pointd &p2);
            int getNumberIntersectedPrimitives(const BoundingBox& b);
            void getIntersectedPrimitives(std::list<const Dcel::Face*> &outputList, const BoundingBox &b);

        protected:
            typedef CGAL::Simple_cartesian<double> K;
            typedef K::FT FT;
            typedef K::Ray_3 CGALRay;
            typedef K::Line_3 CGALLine;
            typedef K::Point_3 CGALPoint;
            typedef K::Triangle_3 CGALTriangle;
            typedef CGAL::AABB_triangle_primitive<K, std::list<CGALTriangle>::iterator> CGALTrianglePrimitive;
            typedef CGAL::AABB_traits<K, CGALTrianglePrimitive> AABB_triangle_traits;
            typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

            typedef AABB_triangle_traits::Bounding_box CGALBoundingBox;

            struct cmpCGALTriangle {
                bool operator()(const CGALTriangle& a, const CGALTriangle& b) const {
                    if (a == b) return false;
                    if (a[0] == b[0] && a[1] == b[1]) return (a[2] < b[2]);
                    if (a[0] == b[0]) return (a[1] < b[1]);
                    return (a[0] < b[0]);
                }
            };

            Tree tree;
            std::map<const Dcel::Vertex*, CGALPoint> vertices_points;
            std::map<CGALPoint, const Dcel::Vertex*> points_vertices;
            std::map<const Dcel::Face*, CGALTriangle> faces_triangles;
            std::map<CGALTriangle, const Dcel::Face*, cmpCGALTriangle> triangles_faces;
            std::list<CGALTriangle> triangles;
    };
    #endif
}

#endif // AABBTREE_H
