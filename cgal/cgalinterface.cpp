#include "cgalinterface.h"

#ifdef CGAL_DEFINED
namespace CGALInterface{
    namespace Triangulation{
        void markDomains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border) {
            if(start->info().nesting_level != -1){
                return;
            }
            std::list<CDT::Face_handle> queue;
            queue.push_back(start);
            while(! queue.empty()){
                CDT::Face_handle fh = queue.front();
                queue.pop_front();
                if(fh->info().nesting_level == -1){
                    fh->info().nesting_level = index;
                    for(int i = 0; i < 3; i++){
                        CDT::Edge e(fh,i);
                        CDT::Face_handle n = fh->neighbor(i);
                        if(n->info().nesting_level == -1){
                            if(ct.is_constrained(e)) border.push_back(e);
                            else queue.push_back(n);
                        }
                    }
                }
            }
        }

        void markDomains(CDT& cdt) {
            for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
                it->info().nesting_level = -1;
            }
            std::list<CDT::Edge> border;
            markDomains(cdt, cdt.infinite_face(), 0, border);
            while(! border.empty()){
                CDT::Edge e = border.front();
                border.pop_front();
                CDT::Face_handle n = e.first->neighbor(e.second);
                if(n->info().nesting_level == -1){
                    markDomains(cdt, n, e.first->info().nesting_level+1, border);
                }
            }
        }

        /*void triangulate(std::vector<std::array<Point2D, 3> >& triangles, const std::vector<Point2D>& polygon, const std::vector<std::vector<Point2D> >& holes) {
            triangles.clear();
            Polygon_2 polygon1;
            std::vector<Polygon_2> innerPolygons;
            for (unsigned int i = 0; i < polygon.size(); ++i){
                CGALPoint p(polygon[i].x(), polygon[i].y());
                polygon1.push_back(p);
            }
            if (holes.size() > 0){
                for (unsigned int i = 0; i < holes.size(); ++i) {
                    Polygon_2 innerPolygon;
                    for (unsigned int j = 0; j < holes[i].size(); ++j) {
                        CGALPoint p(holes[i][j].x(), holes[i][j].y());
                        innerPolygon.push_back(p);
                    }
                    innerPolygons.push_back(innerPolygon);
                }
            }

            ///TRIANGULATION

            CDT cdt;
            cdt.insert_constraint(polygon1.vertices_begin(), polygon1.vertices_end(), true);
            for (unsigned int i = 0; i < innerPolygons.size(); ++i)
                cdt.insert_constraint(innerPolygons[i].vertices_begin(), innerPolygons[i].vertices_end(), true);
            markDomains(cdt);

            for (CDT::Finite_faces_iterator fit=cdt.finite_faces_begin(); fit!=cdt.finite_faces_end();++fit) {
                if ( fit->info().in_domain() ) {
                    Triangle triangle = *fit;
                    CDT::Vertex_handle v = triangle.vertex(0);
                    const CGALPoint p1 = v->point();
                    v = triangle.vertex(1);
                    const CGALPoint p2 = v->point();
                    v = triangle.vertex(2);
                    const CGALPoint p3 = v->point();

                    std::array<Point2D, 3> t;
                    t[0] = Point2D(p1.x(), p1.y());
                    t[1] = Point2D(p2.x(), p2.y());
                    t[2] = Point2D(p3.x(), p3.y());
                    triangles.push_back(t);
                }
            }
        }*/
    }

    namespace BooleanOperations {

        void getCoordinates(const Point_2& p, double& x, double& y) {
            std::stringstream ss1;
            ss1 << std::setprecision(std::numeric_limits<double>::digits10+1);
            ss1 << p.x();
            ss1 >> x;
            std::stringstream ss2;
            ss2 << std::setprecision(std::numeric_limits<double>::digits10+1);
            ss2 << p.y();
            ss2 >> y;
        }

        /*void difference(std::vector<std::vector<Point2D> >& result, const std::vector<Point2D>& polygon1, const std::vector<Point2D>& polygon2) {
            result.clear();

            Polygon_2 P1;
            for (unsigned int i = 0; i < polygon1.size(); ++i){
                P1.push_back(Point_2(polygon1[i].x(), polygon1[i].y()));
            }
            Polygon_2 P2;
            for (unsigned int i = 0; i < polygon2.size(); ++i){
                P2.push_back(Point_2(polygon2[i].x(), polygon2[i].y()));
            }

            Polygon_with_holes_2 D;
            Pwh_list_2 diffR;
            CGAL::difference(P1, P2, std::back_inserter(diffR));
            result.reserve(diffR.size());
            for (Pwh_list_2::const_iterator it = diffR.begin(); it != diffR.end(); ++it) {

                D = *it;
                assert (! D.is_unbounded());
                assert (D.number_of_holes() == 0);

                std::vector<Point2D> polygon;

                CGAL::Polygon_2<Kernel> P = D.outer_boundary();
                polygon.reserve(P.size());
                for ( CGAL::Polygon_2<Kernel>::Vertex_const_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it){
                    Point_2 p = *it;
                    double x, y;
                    getCoordinates(p,x,y);
                    Point2D point(x, y);
                    polygon.push_back(point);
                }
                result.push_back(polygon);
            }

        }*/

        /*void intersection(std::vector<std::vector<Point2D> >& result, const std::vector<Point2D>& polygon1, const std::vector<Point2D>& polygon2) {
            result.clear();

            Polygon_2 P1;
            for (unsigned int i = 0; i < polygon1.size(); ++i){
                P1.push_back(Point_2(polygon1[i].x(), polygon1[i].y()));
            }
            Polygon_2 P2;
            for (unsigned int i = 0; i < polygon2.size(); ++i){
                P2.push_back(Point_2(polygon2[i].x(), polygon2[i].y()));
            }

            Polygon_with_holes_2 D;
            Pwh_list_2 intR;
            CGAL::intersection(P1, P2, std::back_inserter(intR));
            result.reserve(intR.size());
            for (Pwh_list_2::const_iterator it = intR.begin(); it != intR.end(); ++it) {

                D = *it;
                assert (! D.is_unbounded());
                assert (D.number_of_holes() == 0);

                std::vector<Point2D> polygon;

                CGAL::Polygon_2<Kernel> P = D.outer_boundary();
                polygon.reserve(P.size());
                for ( CGAL::Polygon_2<Kernel>::Vertex_const_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it){
                    Point_2 p = *it;
                    double x, y;
                    getCoordinates(p,x,y);
                    Point2D point(x, y);
                    polygon.push_back(point);
                }
                result.push_back(polygon);
            }
        }*/



    }

}
#endif //CGAL_DEFINED
