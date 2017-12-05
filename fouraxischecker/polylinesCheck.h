#ifndef POLYLINES_H
#define POLYLINES_H

#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>
#include <cg3/viewer/managers/eigenmesh_manager/eigenmesh_manager.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <cg3/viewer/pickable_objects/pickable_eigenmesh.h>
#include <cg3/data_structures/arrays/arrays.h>
#include <Eigen/Dense>

#include <CGAL/Surface_mesh.h>
#include <cg3/cgal/cgal_aabbtree.h>
#include <CGAL/Cartesian/Cartesian_base.h>
#include <CGAL/Surface_mesh/IO.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Filtered_kernel.h>
#include <cg3/cgal/cgal_slicer.h>
#include <cg3/utilities/utils.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>
#ifdef GUROBI_DEFINED
#include <gurobi_c++.h>
#endif

#include <algorithm>

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Simple_cartesian<double>                            KK;
typedef CGAL::Surface_mesh<K::Point_3>                            Mesh;
typedef K::Point_3                                                Point3;
typedef K::Point_2                                                Point2;
typedef K::Plane_3                                                PlaneC;
typedef Mesh::Vertex_index                                        vertex_descriptor;
typedef Mesh::Face_index                                          face_descriptor;
typedef Mesh::Vertex_range                                        verticesIter;
typedef Mesh::Property_map<vertex_descriptor, Point3>             MapPoints;
typedef std::vector<std::vector<cg3::Pointd>>                     ArrayPoint;
typedef std::vector<Point2>                                       Array2dPoint;
typedef std::vector<int>                                          VectI;
typedef std::vector<VectI>                                        MatrixI;

class PolylinesCheck
{
    public:
        PolylinesCheck();

        //Pointd  maxP;

        void    minMaxPoints            (const Mesh& mesh, int selection);

        void    setMin                  (cg3::Pointd& minP1);

        void    setMax                  (cg3::Pointd& maxP1);

        cg3::Pointd  getMin                  ();

        cg3::Pointd  getMax                  ();

        void    setPoly                 (const Mesh& mesh, cg3::Vec3& norm);

        void    convertTo2dPlane        ();

        bool    checkPolyline           ();

        void    setNormal               (cg3::Vec3& normal);

        void    setD                    ();

        int     intersect3D_RayTriangle (cg3::Pointd p0, cg3::Pointd p1, cg3::Pointd v0, cg3::Pointd v1, cg3::Pointd v2);

        void    checkIntersect          (cg3::DrawableEigenMesh* meshEigenOrigin, cg3::Pointd p0, cg3::Pointd p1, int selection);

        void    rotatePoint             (Eigen::Matrix3d rotation, cg3::Pointd p);

        void    check                   (cg3::DrawableEigenMesh *meshEigenOrigin, int color, int indexPlane);

        int     serchMinY               (std::vector<int> lista, cg3::DrawableEigenMesh *meshEigenOrigin);

        int     serchMaxY               (std::vector<int> lista, cg3::DrawableEigenMesh *meshEigenOrigin);

        void    setCheckerDimension     (int nplane, int dimension);

        void    resetChecker            ();

        cg3::Array2D<int> getChecker();

        void searchNoVisibleFace        ();

        VectI getNotVisibleFace         ();

        void minimizeProblem            ();

        void updateChecker              (bool b);

        void resetMatrixCheck();

        void serchUniqueTriangoForOrientation();
        MatrixI getUniqueTriangle() const;
        void setUniqueTriangle(const MatrixI &value);

        VectI getOrientationSelected() const;

        void addFaceExlude(unsigned int i);

private:

        cg3::PickableEigenmesh   meshPoly;
        Array2dPoint        poly2d;
        ArrayPoint          poly;
        VectI               notVisibleFace;
        VectI               orientationSelected;
        Point3              min;
        Point3              max;
        cg3::Pointd         minP;
        cg3::Pointd         maxP;
        cg3::Pointd         I;
        cg3::Array2D<int>   checker;
        MatrixI             uniqueTriangle;
        cg3::Vec3           normalplane;
        double              d;
};

#endif // POLYLINES_H
