#ifndef DRAWMANAGER_H
#define DRAWMANAGER_H

#include <cg3/viewer/interfaces/drawable_object.h>
#include <cg3/viewer/interfaces/drawable_mesh.h>
#include <cg3/viewer/mainwindow.h>
#include <cg3/viewer/glcanvas.h>

#include <cg3/viewer/drawable_objects/drawable_eigenmesh.h>
#include <cg3/meshes/eigenmesh/eigenmesh.h>
#include <Eigen/Dense>

#include <CGAL/Surface_mesh.h>
#include <cg3/cgal/cgal_aabbtree.h>
#include <cg3/cgal/cgal.h>
#include <CGAL/Cartesian/Cartesian_base.h>
#include <CGAL/Surface_mesh/IO.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Filtered_kernel.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QDebug>
#include <QFrame>

#include <fouraxischecker/polylinesCheck.h>

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif


class QLineEdit;

typedef CGAL::Exact_predicates_inexact_constructions_kernel       K;
typedef CGAL::Simple_cartesian<double>                           KK;
typedef CGAL::Surface_mesh<K::Point_3>                         Mesh;
typedef K::Point_3                                           PointC;
typedef K::Plane_3                                           PlaneC;
typedef Mesh::Vertex_index                        vertex_descriptor;
typedef Mesh::Face_index                            face_descriptor;
typedef Mesh::Vertex_range                             verticesIter;
typedef Mesh::Property_map<vertex_descriptor, K::Point_3> MapPoints;

typedef CGAL::IO output;

namespace Ui {
    class FourAxisMillingManager;
}

class FourAxisMillingManager : public QFrame
{
        Q_OBJECT

    public:
        explicit FourAxisMillingManager(QWidget *parent = 0);
        ~FourAxisMillingManager();

    void colorUniqueTriangle();

private slots:

        void on_drawAxis_clicked                ();

        void on_loadMesh_clicked                ();

        void on_eigenToCgal_clicked             ();

        void convertEigenMesh                   (cg3::DrawableEigenMesh *meshEigenOrigin);

        void on_clearAxis_clicked               ();

        void on_saveMeshCgal_clicked            ();

        void on_nPlane_editingFinished          ();

        void setButtonMeshLoaded                (bool b);

        void on_writeCoordinate_stateChanged    (int arg1);

        void on_showAxis_stateChanged           (int arg1);

        void on_xAxis_toggled                   (bool checked);

        void on_yAxis_toggled                   (bool checked);

        void on_zAxis_toggled                   (bool checked);

        void on_rotationAxis_clicked            ();

        void on_clearMesh_clicked               ();

        void on_xCoord_editingFinished          ();

        void on_yCoord_editingFinished          ();

        void on_serchPoint_clicked              ();

        void on_sphere_editingFinished          ();

        void on_cylinder_editingFinished        ();

        void on_drawPoint_clicked               ();

        void on_translate_clicked               ();

        void on_check_clicked                   ();

        void on_stepByStep_clicked              ();

        void on_meshToOrigin_clicked            ();

        void on_saveMesh_clicked                ();

        void on_resetParam_clicked              ();

        void on_xRotCord_editingFinished        ();

        void on_yRotCord_editingFinished        ();

        void on_zRotCord_editingFinished        ();

        void on_xMenoR_clicked                  ();

        void on_xPiuR_clicked                   ();

        void on_yMenoR_clicked                  ();

        void on_yPiuR_clicked                   ();

        void on_zMenoR_clicked                  ();

        void on_zPiuR_clicked                   ();

        void on_xTransCord_editingFinished      ();

        void on_yTransCord_editingFinished      ();

        void on_zTransCord_editingFinished      ();

        void on_xMeno_clicked                   ();

        void on_xPiu_clicked                    ();

        void on_yMeno_clicked                   ();

        void on_yPiu_clicked                    ();

        void on_zMeno_clicked                   ();

        void on_zPiu_clicked                    ();

        void translate                          (cg3::Pointd point);

        void rotate                             (int axis);

        void on_pushButton_clicked              ();

        void on_triangleClicked                 (QList<unsigned int> i);

        void on_pointsMeshRadioButton_toggled(bool checked);

        void on_flatMeshRadioButton_toggled(bool checked);
        void on_checkPushButton_clicked();

private:

        cg3::PickableEigenmesh*  meshEigen;
        PolylinesCheck      polyline;
        Ui::FourAxisMillingManager*    ui;
        MainWindow*         mainWindow;
        QString             filename;
        cg3::Pointd              vectorUser;
        cg3::Pointd              point;
        cg3::Pointd              meshBerycenter;
        QColor              color;
        Mesh                mesh;
        cg3::Vec2                coordPlane;
        int                 nPlaneUser=1;
        int                 selection;
        int                 increse;
        int                 stepColor;
        int                 nextColor;
        bool                updateCheckerFlag = false;
        double              stepAngle;
        double              cylinder, sphere;
        double              xRot, yRot, zRot;
        double              xTrans, yTrans, zTrans;
        const double        halfC = M_PI / 180;


};

#endif // DRAWMANAGER_H
