#ifndef AABBTREE_H
#define AABBTREE_H

#include "cgalinterface.h"
#include "../common/bounding_box.h"

#ifdef DCEL_DEFINED
#include "../dcel/dcel.h"
#endif

#ifdef TRIMESH_DEFINED
#include "../trimesh/trimesh.h"
#endif

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>

namespace CGALInterface {

    class AABBTree
    {
        public:

            AABBTree();
            AABBTree(const AABBTree& other);
            #ifdef DCEL_DEFINED
            AABBTree(const Dcel &d, bool forDistanceQueries = false);
            #endif
            #ifdef TRIMESH_DEFINED
            AABBTree(const Trimesh<double> &t, bool forDistanceQueries = false);
            #endif
            AABBTree& operator=(const AABBTree& other);

            int getNumberIntersectedPrimitives(const Pointd& p1, const Pointd &p2);
            int getNumberIntersectedPrimitives(const BoundingBox& b);
            double getSquaredDistance(const Pointd &p);
            Pointd getNearestPoint(const Pointd &p);
            bool isInside(const Pointd &p, int numberOfChecks = 7);
            #ifdef DCEL_DEFINED
            void getIntersectedDcelFaces(std::list<const Dcel::Face*> &outputList, const BoundingBox &b);
            void getCompletelyContainedDcelFaces(std::list<const Dcel::Face*> &outputList, const BoundingBox &b);
            const Dcel::Face* getNearestDcelFace(const Pointd &p);
            #endif

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
            bool forDistanceQueries;
            #ifdef DCEL_DEFINED
            std::map<const Dcel::Vertex*, CGALPoint> mapDcelVerticesToCgalPoints;
            std::map<CGALTriangle, const Dcel::Face*, cmpCGALTriangle> mapCgalTrianglesToDcelFaces;
            #endif
            #ifdef TRIMESH_DEFINED
            std::map<int, CGALPoint> mapTrimeshVerticesToCgalPoints;
            std::map<CGALTriangle, int, cmpCGALTriangle> mapCgalTrianglesToTrimeshTriangles;
            #endif
            std::list<CGALTriangle> triangles;
            BoundingBox bb;

            std::random_device rd;
            std::mt19937 e2;
    };

}

#endif // AABBTREE_H
