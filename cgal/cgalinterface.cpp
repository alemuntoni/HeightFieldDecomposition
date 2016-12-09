#include "cgalinterface.h"

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

        std::vector<std::array<Point2D, 3> > triangulate(const std::vector<Point2D>& polygon, const std::vector<std::vector<Point2D> >& holes) {
            std::vector<std::array<Point2D, 3> > triangles;
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
            return triangles;
        }

        std::vector<std::array<Pointd, 3> > triangulate(const Vec3 &normal, const std::vector<Pointd>& polygon, const std::vector<std::vector<Pointd> >& holes){
            std::vector<std::array<Pointd, 3> > triangles;
            std::map<Point2D, Pointd> pointsVerticesMap;

            //Rotation of the coordinates
            Vec3 zAxis(0,0,1);
            Vec3 axis = -(normal.cross(zAxis));
            axis.normalize();
            double dot = normal.dot(zAxis);
            double angle = acos(dot);

            double r[3][3] = {0};
            if (normal != zAxis){
                if (normal == -zAxis){
                    axis = Vec3(1,0,0);
                }
                Common::getRotationMatrix(axis, angle, r);
            }
            else {
                r[0][0] = r[1][1] = r[2][2] = 1;
            }

            //rotate points and make 2D polygon
            std::vector<Point2D> polygon2D;
            std::vector<std::vector<Point2D> > innerPolygons2D;
            for (unsigned int i = 0; i < polygon.size(); ++i){
                Pointd a = polygon[i];
                Pointd p1(a.x() * r[0][0] + a.y() * r[1][0] +a.z() * r[2][0], a.x() * r[0][1] + a.y() * r[1][1] +a.z() * r[2][1], a.x() * r[0][2] + a.y() * r[1][2] +a.z() * r[2][2]);
                Point2D p(p1.x(), p1.y());
                polygon2D.push_back(p);
                pointsVerticesMap[p] = a;
            }
            if (holes.size() > 0){
                for (unsigned int i = 0; i < holes.size(); ++i) {
                    std::vector<Point2D> innerPolygon;
                    for (unsigned j = 0; j < holes[i].size(); ++j) {
                        Pointd a = holes[i][j];
                        Pointd p1(a.x() * r[0][0] + a.y() * r[1][0] + a.z() * r[2][0],
                                  a.x() * r[0][1] + a.y() * r[1][1] + a.z() * r[2][1],
                                  a.x() * r[0][2] + a.y() * r[1][2] + a.z() * r[2][2]);
                        Point2D p(p1.x(), p1.y());
                        innerPolygon.push_back(p);
                        pointsVerticesMap[p] = a;
                    }
                    innerPolygons2D.push_back(innerPolygon);
                }
            }
            std::vector<std::array<Point2D, 3> > triangles2D;
            triangles2D = triangulate(polygon2D, innerPolygons2D);
            triangles.clear();
            triangles.reserve(triangles2D.size());
            for (unsigned int i = 0; i < triangles2D.size(); i++){
                std::array<Point2D, 3> triangle2D = triangles2D[i];
                std::array<Pointd, 3> triangle3D;
                assert(pointsVerticesMap.find(triangle2D[0]) != pointsVerticesMap.end());
                assert(pointsVerticesMap.find(triangle2D[1]) != pointsVerticesMap.end());
                assert(pointsVerticesMap.find(triangle2D[2]) != pointsVerticesMap.end());
                triangle3D[0] = pointsVerticesMap[triangle2D[0]];
                triangle3D[1] = pointsVerticesMap[triangle2D[1]];
                triangle3D[2] = pointsVerticesMap[triangle2D[2]];
                triangles.push_back(triangle3D);
            }
            return triangles;
        }
    }

    namespace BooleanOperations2D {

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

        std::vector<std::vector<Point2D> > difference(const std::vector<Point2D>& polygon1, const std::vector<Point2D>& polygon2) {
            std::vector<std::vector<Point2D> > result;

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
            return result;

        }

        std::vector<std::vector<Point2D> > intersection(const std::vector<Point2D>& polygon1, const std::vector<Point2D>& polygon2) {
            std::vector<std::vector<Point2D> > result;

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
            return result;
        }

    }

    #ifdef CGAL_EIGEN3_ENABLED
    void HoleFilling::fillHolesMeshOff(const std::string& input, const std::string& output) {
        std::ifstream sinput(input);
        Polyhedron poly;
        if ( !sinput || !(sinput >> poly) || poly.empty() ) {
          std::cerr << "Not a valid off file." << std::endl;
          assert(0);
        }
        // Incrementally fill the holes
        unsigned int nb_holes = 0;
        for(Halfedge_handle h : halfedges(poly))
        {
          if(h->is_border())
          {
            std::vector<Facet_handle>  patch_facets;
            std::vector<Vertex_handle> patch_vertices;
            bool success = CGAL::cpp11::get<0>(
              CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                        poly,
                        h,
                        std::back_inserter(patch_facets),
                        std::back_inserter(patch_vertices),
           CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, poly)).
                        geom_traits(Kernel())) );
            std::cout << " Number of facets in constructed patch: " << patch_facets.size() << std::endl;
            std::cout << " Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
            std::cout << " Fairing : " << (success ? "succeeded" : "failed") << std::endl;
            ++nb_holes;
          }
        }
        std::cout << std::endl;
        std::cout << nb_holes << " holes have been filled" << std::endl;

        std::ofstream out(output);
        out.precision(17);
        out << poly << std::endl;
    }
    #endif

    std::vector<std::vector<Pointd> > Slicer::getPolylines(const std::string& inputOffFile, const Vec3& norm, double d) {
        std::ifstream input(inputOffFile.c_str());
        Mesh mesh;
        if (!input || !(input >> mesh) || mesh.is_empty()) {
            std::cerr << "Not a valid off file." << std::endl;
            exit(1);
        }
        // Slicer constructor from the mesh
        //CGAL::Polygon_mesh_slicer<Mesh, K> slicer(mesh);
        Polylines polylines;
        AABB_tree tree(edges(mesh).first, edges(mesh).second, mesh);
        CGAL::Polygon_mesh_slicer<Mesh, K> slicer_aabb(mesh, tree);
        slicer_aabb(K::Plane_3(norm.x(), norm.y(), norm.z(), d), std::back_inserter(polylines));
        //slicer(K::Plane_3(norm.x(), norm.y(), norm.z(), d), std::back_inserter(polylines));
        //std::cout << "At z = 0.4, the slicer intersects "
        //          << polylines.size() << " polylines" << std::endl;
        std::vector< std::vector<Pointd> > result;
        for (std::vector<K::Point_3> singlePolyline : polylines){
            std::vector<Pointd> v;
            for (K::Point_3 point : singlePolyline){
                Pointd pres(point.x(), point.y(), point.z());
                v.push_back(pres);
            }
            result.push_back(v);
        }
        return result;
    }

}
