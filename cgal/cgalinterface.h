#ifndef CGALINTERFACE_H
#define CGALINTERFACE_H

#include "../common/utils.h"
#include "../common/point.h"
#include "../common/point2d.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>

#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <fstream>

/**
  CGAL Triangulation
  */
namespace CGALInterface {

    namespace Triangulation {
        struct FaceInfo2
        {
                FaceInfo2(){}
                int nesting_level;
                bool in_domain(){
                    return nesting_level%2 == 1;
                }
        };


        typedef CGAL::Exact_predicates_inexact_constructions_kernel             K;
        typedef CGAL::Triangulation_vertex_base_2<K>                            Vb;
        typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>          Fbb;
        typedef CGAL::Constrained_triangulation_face_base_2<K,Fbb>              Fb;
        typedef CGAL::Triangulation_data_structure_2<Vb,Fb>                     TDS;
        typedef CGAL::Exact_predicates_tag                                      Itag;
        typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>        CDT;
        typedef CDT::Point                                                      CGALPoint;
        typedef CGAL::Polygon_2<K>                                              Polygon_2;

        typedef CGAL::Epick                                                     E;
        typedef CGAL::Triangulation_ds_face_base_2<TDS>                         TDFB2;
        typedef CGAL::Triangulation_face_base_2<E, TDFB2>                       TFB2;
        typedef CGAL::Triangulation_face_base_with_info_2<FaceInfo2, E, TFB2>   TFBI;
        typedef CGAL::Constrained_triangulation_face_base_2<E, TFBI >           Triangle;

        static std::vector< std::vector<Point2D> > dummy_holes2D;
        static std::vector< std::vector<Pointd> > dummy_holes;

        void markDomains(CDT& ct, CDT::Face_handle start, int index, std::list<CDT::Edge>& border);
        void markDomains(CDT& cdt);
        std::vector<std::array<Point2D, 3> > triangulate(const std::vector<Point2D>& polygon, const std::vector<std::vector<Point2D> >& holes = dummy_holes2D);
        std::vector<std::array<Pointd, 3> > triangulate(const Vec3 &normal, const std::vector<Pointd>& polygon, const std::vector<std::vector<Pointd> >& holes = dummy_holes);
    }

    namespace BooleanOperations2D {

        typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
        typedef Kernel::Point_2                                   Point_2;
        typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
        typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
        typedef std::list<Polygon_with_holes_2>                   Pwh_list_2;

        void getCoordinates(const Point_2&p, double &x, double &y);

        std::vector< std::vector<Point2D> > difference(const std::vector<Point2D> &polygon1, const std::vector<Point2D> &polygon2);
        std::vector<std::vector<Point2D> > intersection(const std::vector<Point2D>& polygon1, const std::vector<Point2D>& polygon2);
    }

    #ifdef CGAL_EIGEN3_ENABLED
    namespace HoleFilling {

        typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
        typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
        typedef Polyhedron::Halfedge_handle    Halfedge_handle;
        typedef Polyhedron::Facet_handle       Facet_handle;
        typedef Polyhedron::Vertex_handle      Vertex_handle;

        void fillHolesMeshOff(const std::string& input, const std::string& output);
    }
    #endif

    class AABBTree;


}

#endif // CGALINTERFACE_H
