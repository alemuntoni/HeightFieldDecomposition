#include "fouraxischeckermanager.h"
#include "ui_fouraxischeckermanager.h"

#include <cg3/geometry/transformations.h>

#include <QtGui>
#include <string>
#include <QDebug>

#include "../fouraxischecker/fouraxischecker.h"

#define halfC (M_PI / 180)

using namespace cg3;
using namespace Eigen;

FourAxisMillingManager::FourAxisMillingManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::FourAxisMillingManager),
    mainWindow((MainWindow*)parent)

{
    ui->setupUi(this);
    //connect(mainWindow, SIGNAL(objectPicked(uint)),this, SLOT(on_triangleClicked(uint)));
    connect(mainWindow, SIGNAL(objectsPicked(QList<unsigned int>)),
            this, SLOT(on_triangleClicked(QList<unsigned int>)));
}

FourAxisMillingManager::~FourAxisMillingManager(){
    delete ui;
}

void FourAxisMillingManager::setButtonMeshLoaded(bool b){
    ui->selectAxis->setEnabled(b);
    ui->xAxis->setEnabled(b);
    ui->yAxis->setEnabled(b);
    ui->zAxis->setEnabled(b);
    ui->nPlaneLabel->setEnabled(b);
    ui->nPlane->setEnabled(b);
    //ui->writeCoordinate->setEnabled(b);
    ui->showAxis->setEnabled(b);
    ui->clearMesh->setEnabled(b);
    ui->loadMesh->setEnabled(false);
    ui->xLabel->setEnabled(b);
    ui->yLabel->setEnabled(b);
    ui->xCoord->setEnabled(b);
    ui->yCoord->setEnabled(b);
    ui->serchPoint->setEnabled(b);
    ui->selectLabel->setEnabled(b);
    ui->cylinder->setEnabled(b);
    ui->Cylinder->setEnabled(b);
    ui->sphere->setEnabled(b);
    ui->Sphere->setEnabled(b);
    ui->saveMesh->setEnabled(b);
    ui->check->setEnabled(b);
    ui->resetParam->setEnabled(b);
    ui->meshToOrigin->setEnabled(b);
    ui->visualFrame->setEnabled(b);
    ui->checkPushButton->setEnabled(b);

    ui->selectAxis->setEnabled(b);
        QList<QWidget*> list = ui->selectAxis->findChildren<QWidget*>() ;
        foreach( QWidget* w, list )
        {
           w->setEnabled(b) ;
        }
        ui->selectAxis_2->setEnabled(b);
        QList<QWidget*> list2 = ui->selectAxis_2->findChildren<QWidget*>() ;
        foreach( QWidget* w, list2 )
        {
           w->setEnabled(b) ;
        }

}

void FourAxisMillingManager::on_drawAxis_clicked()
{
    ui->clearAxis->setEnabled(true);
    mainWindow->enableDebugObjects();
    mainWindow->clearDebugCylinders();
    vectorUser.normalize();
    mainWindow->addDebugCylinder(Pointd(0,0,0),
                                 Pointd(vectorUser.x(), vectorUser.y(), vectorUser.z()),
                                 0.01,
                                 QColor("#ff0000"));
}

void FourAxisMillingManager::on_nPlane_editingFinished(){
    QLineEdit *nPlane = new QLineEdit;
    nPlane->setValidator(new QIntValidator(0, 360,nPlane));
    nPlaneUser = ui->nPlane->text().toInt();
    stepAngle = 180 / nPlaneUser;
    stepColor = 120 / nPlaneUser;
    polyline.setCheckerDimension(nPlaneUser+1,meshEigen->getNumberFaces());
}

void FourAxisMillingManager::on_loadMesh_clicked(){
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open Eigen Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    std::string nameMesh = filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1);
    if (!filename.isEmpty()) {
        meshEigen = new PickableEigenmesh();
        meshEigen->readFromObj(filename.toStdString());        
        mainWindow->pushObj(meshEigen, nameMesh);
        setButtonMeshLoaded(true);
        mainWindow->updateGlCanvas();
    }
    nextColor = 0;
    cylinder = 0.01;
    increse = 0;
    xRot = 0;
    yRot = 0;
    zRot = 0;
    QColor c;
    c.setHsv(100,0,255);

    for(unsigned int i = 0 ; i < meshEigen->getNumberFaces(); i++){
        meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
    }
}

void FourAxisMillingManager::on_eigenToCgal_clicked(){
    convertEigenMesh(meshEigen);
}

void FourAxisMillingManager::convertEigenMesh (DrawableEigenMesh *meshEigenOrigin){
    //Il tasto è stato rimosso
    int nVertex = meshEigenOrigin->getNumberVertices();
    int nFaces = meshEigenOrigin->getNumberFaces();

    std::vector<vertex_descriptor> vecIter;

    for(int i=0; i<nVertex;i++){
        Point<double> v = meshEigenOrigin->getVertex(i);
        vertex_descriptor vert = mesh.add_vertex(K::Point_3(v.x(),v.y(),v.z()));
        vecIter.push_back(vert);
    }

    for(int i=0; i<nFaces;i++){
        Point<int> f = meshEigenOrigin->getFace(i);
        vertex_descriptor u = vecIter[f.x()];
        vertex_descriptor v = vecIter[f.y()];
        vertex_descriptor z = vecIter[f.z()];
        mesh.add_face(u, v, z);
    }
}

void FourAxisMillingManager::on_clearAxis_clicked(){
    mainWindow->clearDebugCylinders();
    ui->clearAxis->setEnabled(false);
}

void FourAxisMillingManager::on_saveMeshCgal_clicked(){
    //Tasto rimosso dal manager
    QFileDialog filedialog(mainWindow, tr("Export surface to file"));
    filedialog.setFileMode(QFileDialog::AnyFile);

    filedialog.setNameFilter(tr("OFF files (*.off);;"
                                  "All files (*)"));

    filedialog.setAcceptMode(QFileDialog::AcceptSave);
    filedialog.setDefaultSuffix("off");
    if(filedialog.exec())
    {
        const QString filename = filedialog.selectedFiles().front();
        std::cerr << "Saving to file \"" << filename.toLocal8Bit().data() << "\"...";
        std::ofstream out(filename.toUtf8());
        out<<mesh;

        if(!out)
        {
          QMessageBox::warning(mainWindow, mainWindow->windowTitle(),
                             tr("Export to the OFF file <tt>%1</tt> failed!").arg(filename));
          std::cerr << qPrintable(tr("Export to the OFF file %1 failed!").arg(filename)) << std::endl;
          mainWindow->statusBar()->showMessage(tr("Export to the OFF file %1 failed!").arg(filename));
          std::cerr << " failed!\n";
        }
        else
        {
            std::cerr << " done.\n";
            std::cerr << qPrintable(tr("Successfull export to the OFF file %1.").arg(filename)) << std::endl;
            mainWindow->statusBar()->showMessage(tr("Successfull export to the OFF file %1.").arg(filename));
        }
    }
}

void FourAxisMillingManager::on_writeCoordinate_stateChanged(int arg1){
    if(arg1 == Qt::Checked){
        ui->coordinate->setEnabled(true);
        ui->drawAxis->setEnabled(true);
        ui->selectAxis->setEnabled(false);
        ui->xAxis->setEnabled(false);
        ui->yAxis->setEnabled(false);
        ui->zAxis->setEnabled(false);
    } else {
        ui->selectAxis->setEnabled(true);
        ui->xAxis->setEnabled(true);
        ui->yAxis->setEnabled(true);
        ui->zAxis->setEnabled(true);
        ui->coordinate->setEnabled(false);
        ui->drawAxis->setEnabled(false);
    }
}

void FourAxisMillingManager::on_showAxis_stateChanged(int arg1){
    if (arg1 == Qt::Checked) mainWindow->drawAxis(true);
    else mainWindow->drawAxis(false);
    mainWindow->updateGlCanvas();
}

void FourAxisMillingManager::on_xAxis_toggled(bool checked){
    if(checked){
        selection=0;
    }
}

void FourAxisMillingManager::on_yAxis_toggled(bool checked){
    if(checked){
        selection=1;
    }
}

void FourAxisMillingManager::on_zAxis_toggled(bool checked){
    if(checked){
        selection=2;
    }
}

void FourAxisMillingManager::on_rotationAxis_clicked(){
    //Funzione momentaneamente rimossa, cambio piano di attacco alla mesh
    /*polyline.minMaxPoints(mesh, selection);

    mainWindow->enableDebugObjects();
    mainWindow->clearDebugSpheres();

    //PlaneC plane(polyline.getMax(), polyline.getMin(), PointC(0,0,0));
    PointC punto(polyline.getMin().x(), polyline.getMin().y(), polyline.getMin().z()+1);
    mainWindow->addDebugSphere(Pointd(polyline.getMin().x(), polyline.getMin().y(), polyline.getMin().z()),0.01,QColor("#ff0000"),50);
    mainWindow->addDebugSphere(Pointd(polyline.getMax().x(), polyline.getMax().y(), polyline.getMax().z()),0.01,QColor("#ff0000"),50);
    mainWindow->addDebugSphere(Pointd(punto.x(), punto.y(), punto.z()), 0.01, QColor("#ff0000"),50);

    //Creo il piano passante per 3 punti
    PlaneC plane(polyline.getMax(), polyline.getMin(), punto);

    K::Vector_3 chePalle(plane.orthogonal_vector());
    Vec3 normalPlane(chePalle.x(), chePalle.y(), chePalle.z() );

    polyline.setNormal(normalPlane);
    polyline.setD();
    polyline.setPoly(mesh, normalPlane);

    for(unsigned int i = 0; i < polyline.poly.size(); i++){
        for(unsigned int j = 0; j < polyline.poly[i].size(); j++){
            mainWindow->addDebugSphere(polyline.poly[i][j], 0.01, QColor("#ff0000"),50);
        }
    }
    polyline.checkPolyline();
    for(unsigned int i = 0; i <polyline.poly2d.size(); i++) {
        mainWindow->addDebugSphere(Pointd(polyline.poly2d[i].x(),polyline.poly2d[i].y(),0) , 0.01, QColor("#ff0000"),50);
    }*/
}

void FourAxisMillingManager::on_clearMesh_clicked(){
    ui->selectAxis->setEnabled(false);
    ui->xAxis->setEnabled(false);
    ui->yAxis->setEnabled(false);
    ui->zAxis->setEnabled(false);
    ui->nPlaneLabel->setEnabled(false);
    ui->nPlane->setEnabled(false);
    ui->showAxis->setEnabled(false);
    ui->clearMesh->setEnabled(false);
    ui->loadMesh->setEnabled(true);
    ui->saveMesh->setEnabled(false);
    ui->visualFrame->setEnabled(false);
    ui->checkFrame->setEnabled(false);
    ui->selectAxis_2->setEnabled(false);
    mainWindow->deleteObj(meshEigen);
    mainWindow->clearDebugCylinders();
    mainWindow->clearDebugSpheres();
    delete meshEigen;
    polyline.getChecker().clear();
    meshEigen = nullptr;

}

void FourAxisMillingManager::on_xCoord_editingFinished(){
    QLineEdit *xCordinate = new QLineEdit;
    xCordinate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, xCordinate));
    coordPlane.setXCoord(ui->xCoord->text().toDouble());
}

void FourAxisMillingManager::on_yCoord_editingFinished(){
    QLineEdit *yCordinate = new QLineEdit;
    yCordinate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, yCordinate));
    coordPlane.setYCoord(ui->yCoord->text().toDouble());
}

void FourAxisMillingManager::on_serchPoint_clicked(){
    switch (selection) {
    case 0:
        vectorUser.setX(-100);
        vectorUser.setY(coordPlane.y());
        vectorUser.setZ(coordPlane.x());
        point.setX(100);
        point.setY(coordPlane.y());
        point.setZ(coordPlane.x());
        break;
    case 1:
        vectorUser.setX(coordPlane.x());
        vectorUser.setY(-100);
        vectorUser.setZ(coordPlane.y());
        point.setX(coordPlane.x());
        point.setY(100);
        point.setZ(coordPlane.y());
        break;
    case 2:
        vectorUser.setX(coordPlane.y());
        vectorUser.setY(coordPlane.x());
        vectorUser.setZ(-100);
        point.setX(coordPlane.y());
        point.setY(coordPlane.x());
        point.setZ(100);
        break;
    default:
        break;
    }
    mainWindow->enableDebugObjects();
    mainWindow->clearDebugSpheres();
    mainWindow->clearDebugCylinders();
    mainWindow->addDebugSphere(vectorUser, sphere, QColor("#ff0000"),50);
    mainWindow->addDebugCylinder(vectorUser, point,cylinder, QColor("#ff0000"));
}

void FourAxisMillingManager::on_sphere_editingFinished(){
    QLineEdit *spherea = new QLineEdit;
    spherea->setValidator(new QDoubleValidator(-999.0, 999.0, 2, spherea));
    sphere = ui->sphere->text().toDouble();
}

void FourAxisMillingManager::on_cylinder_editingFinished(){
    QLineEdit *cylindera = new QLineEdit;
    cylindera->setValidator(new QDoubleValidator(-999.0, 999.0, 2, cylindera));
    cylinder = ui->cylinder->text().toDouble();
}

void FourAxisMillingManager::on_drawPoint_clicked(){
    polyline.checkIntersect(meshEigen, vectorUser, point, selection);
    mainWindow->enableDebugObjects();
    mainWindow->clearDebugSpheres();
    mainWindow->clearDebugCylinders();
    mainWindow->addDebugSphere(polyline.getMin(), sphere, QColor("#ff0000"),50);
    mainWindow->addDebugSphere(polyline.getMax(), sphere, QColor("#ff0000"),50);

}

void FourAxisMillingManager::on_translate_clicked(){
    Pointd centro(-((polyline.getMax()-polyline.getMin())/2+polyline.getMin()));
    meshEigen->translate(centro);
    Matrix3d rotation;
    Vec3 y(1,0,0);
    Vec3 z(0,0,1);
    Vec3 x(1,0,0);
    if(selection == 1){
        cg3::getRotationMatrix(z, acos(0), rotation);
        meshEigen->rotate(rotation,Vector3d(0,0,0));
        cg3::getRotationMatrix(x, acos(0), rotation);
        meshEigen->rotate(rotation,Vector3d(0,0,0));
    }
    if(selection == 2){
        cg3::getRotationMatrix(y, -acos(0), rotation);
        meshEigen->rotate(rotation,Vector3d(0,0,0));
    }
    polyline.getMin().operator +=(centro);
    polyline.getMax().operator +=(centro);
    polyline.rotatePoint(rotation,Pointd(0,0,0));
    mainWindow->clearDebugSpheres();
    mainWindow->addDebugSphere(polyline.getMax(), sphere, QColor("#ff0000"),50);
    mainWindow->addDebugSphere(polyline.getMin(), sphere, QColor("#ff0000"),50);
    mainWindow->updateGeometry();
}

void FourAxisMillingManager::on_check_clicked(){
    int angleCStart = 120;
    Matrix3d rotation;
    Vec3 axis(1,0,0);
    QColor c;
    polyline.updateChecker(false);
    polyline.check(meshEigen,angleCStart,0);

    for(int i = 0; i < nPlaneUser; i++){
        cg3::getRotationMatrix(axis, (stepAngle * halfC), rotation);
        meshEigen->rotate(rotation,Vector3d(0,0,0));
        meshEigen->updateBoundingBox();
        polyline.check(meshEigen,angleCStart,0);
        mainWindow->updateGlCanvas();
    }

    rotation = getRotationMatrix(axis, 180*halfC);
    meshEigen->rotate(rotation,Vector3d(0,0,0));
    meshEigen->updateBoundingBox();
    mainWindow->updateGlCanvas();

    polyline.searchNoVisibleFace();

    if(polyline.getNotVisibleFace().size() > 0){
        c.setHsv(0,255,255);
        for(int i : polyline.getNotVisibleFace()){
            meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        QMessageBox msgBox;
        msgBox.setWindowTitle("Attenzione");
        msgBox.setText("Con questa configurazione alcune facce non sono visibili");
        msgBox.setStandardButtons(QMessageBox::Ok);
        if(msgBox.exec() == QMessageBox::Ok){
            updateCheckerFlag = true;
            ui->stepByStep->setEnabled(true);
        }
    }else {
        ui->stepByStep->setEnabled(true);
    }

}

void FourAxisMillingManager::on_stepByStep_clicked(){
    if(filename == ""){
        on_saveMesh_clicked();
    }

    Vec3 axis(1,0,0);
    QColor c;
    c.setHsv(100,0,255);
    Matrix3d rotation = getRotationMatrix(axis, (stepAngle * halfC * increse));

    for(unsigned int i = 0 ; i < meshEigen->getNumberFaces(); i++){
        c.setHsv(100,0,127);
        meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
    }

    mainWindow->updateGlCanvas();

    meshEigen->rotate(rotation,Vector3d(0,0,0));
    meshEigen->updateBoundingBox();

    polyline.resetChecker();
    polyline.setCheckerDimension(nPlaneUser,meshEigen->getNumberFaces());
    polyline.updateChecker(false);

    polyline.check(meshEigen,nextColor,0);

    rotation = getRotationMatrix(axis, -(stepAngle * halfC * increse));
    meshEigen->rotate(rotation,Vector3d(0,0,0));
    filename.truncate(filename.size()-4);
    QString format = ".obj";
    meshEigen->saveOnObj((filename + QString::number(increse)+format).toUtf8().constData());

    for(unsigned int i = 0 ; i < meshEigen->getNumberFaces(); i++){
        c.setHsv(100,0,127);
        meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
    }
    increse++;
    nextColor += stepColor;
    for(int i = 0; i < nPlaneUser; i++){
        Matrix3d rotation = getRotationMatrix(axis, (stepAngle * halfC * increse));
        meshEigen->rotate(rotation,Vector3d(0,0,0));
        meshEigen->updateBoundingBox();

        polyline.check(meshEigen,nextColor,i+1);
        //mainWindow->updateGlCanvas();

        rotation = getRotationMatrix(axis, -(stepAngle * halfC * increse));
        meshEigen->rotate(rotation,Vector3d(0,0,0));

        meshEigen->saveOnObj((filename + QString::number(increse)+format).toUtf8().constData());
        for(unsigned int i = 0 ; i < meshEigen->getNumberFaces(); i++){
            meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        increse++;
        nextColor += stepColor;
    }

        ui->stepByStep->setEnabled(false);

        polyline.minimizeProblem();
        polyline.serchUniqueTriangoForOrientation();
        colorUniqueTriangle();
}

void FourAxisMillingManager::on_meshToOrigin_clicked(){
    meshBerycenter = meshEigen->getBarycenter();
    meshEigen->translate(-meshBerycenter);
    mainWindow->updateGlCanvas();
}

void FourAxisMillingManager::on_saveMesh_clicked(){
    QFileDialog filedialog(mainWindow, tr("Export surface to file"));
    filedialog.setFileMode(QFileDialog::AnyFile);

    filedialog.setNameFilter(tr("OBJ files (*.obj);;"
                                  "All files (*)"));

    filedialog.setAcceptMode(QFileDialog::AcceptSave);
    filedialog.setDefaultSuffix("obj");
    if(filedialog.exec()){
        filename = filedialog.selectedFiles().front();
        //std::cerr << "Saving to file \"" << filename.toLocal8Bit().data() << "\"...";
        meshEigen->saveOnObj(filename.toUtf8().constData());
    }
}

void FourAxisMillingManager::on_resetParam_clicked(){
    increse = 0;
    stepColor = 0;
    nextColor = 0;
    nPlaneUser = 1;
    stepAngle = 0;
    polyline.resetChecker();
    filename ="";
}

void FourAxisMillingManager::on_xRotCord_editingFinished(){
    QLineEdit *xRotate = new QLineEdit;
    xRotate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, xRotate));
    xRot = ui->xRotCord->text().toDouble();
}

void FourAxisMillingManager::on_yRotCord_editingFinished(){
    QLineEdit *yRotate = new QLineEdit;
    yRotate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, yRotate));
    yRot = ui->yRotCord->text().toDouble();
}

void FourAxisMillingManager::on_zRotCord_editingFinished(){
    QLineEdit *zRotate = new QLineEdit;
    zRotate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, zRotate));
    zRot = ui->zRotCord->text().toDouble();
}

void FourAxisMillingManager::on_xMenoR_clicked(){
    rotate(0);
}

void FourAxisMillingManager::on_xPiuR_clicked(){
    rotate(1);
}

void FourAxisMillingManager::on_yMenoR_clicked(){
    rotate(2);
}

void FourAxisMillingManager::on_yPiuR_clicked(){
    rotate(3);
}

void FourAxisMillingManager::on_zMenoR_clicked(){
    rotate(4);
}

void FourAxisMillingManager::on_zPiuR_clicked(){
    rotate(5);
}

void FourAxisMillingManager::rotate(int axisP){
    Matrix3d rotation;
    Vec3 axis;
    switch (axisP) {
    case 0:
        axis = Vec3(1,0,0);
        rotation = getRotationMatrix(axis, -(xRot * halfC));
        break;
    case 1:
        axis = Vec3(1,0,0);
        rotation = getRotationMatrix(axis, (xRot * halfC));
        break;
    case 2:
        axis = Vec3(0,1,0);
        rotation = getRotationMatrix(axis, -(yRot * halfC));
        break;
    case 3:
        axis = Vec3(0,1,0);
        rotation = getRotationMatrix(axis, (yRot * halfC));
        break;
    case 4:
        axis = Vec3(0,0,1);
        rotation = getRotationMatrix(axis, -(zRot * halfC));
        break;
    case 5:
        axis = Vec3(0,0,1);
        rotation = getRotationMatrix(axis, (zRot * halfC));
        break;
    default:
        break;
    }
    meshEigen->rotate(rotation,Vector3d(0,0,0));
    meshEigen->updateBoundingBox();
    mainWindow->updateGlCanvas();
}

void FourAxisMillingManager::on_xTransCord_editingFinished(){
    QLineEdit *xTranslate = new QLineEdit;
    xTranslate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, xTranslate));
    xTrans = ui->xTransCord->text().toDouble();
}

void FourAxisMillingManager::on_yTransCord_editingFinished(){
    QLineEdit *yTranslate = new QLineEdit;
    yTranslate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, yTranslate));
    yTrans = ui->yTransCord->text().toDouble();
}

void FourAxisMillingManager::on_zTransCord_editingFinished()
{
    QLineEdit *zTranslate = new QLineEdit;
    zTranslate->setValidator(new QDoubleValidator(-999.0, 999.0, 2, zTranslate));
    zTrans = ui->zTransCord->text().toDouble();
}

void FourAxisMillingManager::on_xMeno_clicked(){
    Pointd translateP(xTrans, 0, 0);
    translate(-translateP);
}

void FourAxisMillingManager::on_xPiu_clicked(){
    Pointd translateP(xTrans, 0, 0);
    translate(translateP);
}

void FourAxisMillingManager::on_yMeno_clicked(){
    Pointd translateP(0, yTrans, 0);
    translate(-translateP);
}

void FourAxisMillingManager::on_yPiu_clicked(){
    Pointd translateP(0, yTrans, 0);
    translate(translateP);
}

void FourAxisMillingManager::on_zMeno_clicked(){
    Pointd translateP(0, 0, zTrans);
    translate(-translateP);
}

void FourAxisMillingManager::on_zPiu_clicked(){
    Pointd translateP(0, 0, zTrans);
    translate(translateP);
}

void FourAxisMillingManager::translate(Pointd point){
    meshEigen->translate(point);
    mainWindow->updateGlCanvas();
}

void FourAxisMillingManager::colorUniqueTriangle(){

    increse = 0;
    QString format = ".obj";
    QColor c;
    for(unsigned int i = 0; i <polyline.getUniqueTriangle().size();i++){
        for(unsigned int j = 0; j <polyline.getUniqueTriangle()[i].size();j++){
        }
    }
    for(unsigned int i = 0; i < polyline.getUniqueTriangle().size(); i++){
        //Coloro di nuovo tutta la mesh di grigio
        c.setHsv(0,0,127);
        for(unsigned int i = 0 ; i < meshEigen->getNumberFaces(); i++){
            meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }

        //verifico se per l'orientamento selezionato esistono singoli triangoli visti
        if(polyline.getUniqueTriangle()[i].size() > 0){
            c.setHsv(0,255,255);
            for(unsigned int j = 0 ; j < polyline.getUniqueTriangle()[i].size(); j++){
                int f = polyline.getUniqueTriangle()[i][j];
                meshEigen->setFaceColor(c.redF(), c.greenF(), c.blueF(), f);
            }

            meshEigen->saveOnObj((filename + QString::number(polyline.getOrientationSelected()[i])+"singole"+format).toUtf8().constData());
        }

    }
}

void FourAxisMillingManager::on_pushButton_clicked(){
    mainWindow->updateGlCanvas();
}

void FourAxisMillingManager::on_triangleClicked(QList<unsigned int> i){
    color.setHsv(0,255,255);
    meshEigen->setFaceColor(color.redF(), color.greenF(), color.blueF(), i.back());
    polyline.addFaceExlude(i.back());
}

void FourAxisMillingManager::on_pointsMeshRadioButton_toggled(bool checked){
    if (checked){
        meshEigen->setPointsShading();
        mainWindow->updateGlCanvas();
    }
}

void FourAxisMillingManager::on_flatMeshRadioButton_toggled(bool checked) {
    if (checked){
        meshEigen->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void FourAxisMillingManager::on_checkPushButton_clicked() {
    cg3::Array2D<int> visibility;
    std::vector<int> survivedPlanes;
    int nPlaneUser = ui->nPlane->text().toInt();
    FourAxisChecker::checkVisibilityAllPlanes(*meshEigen, visibility, nPlaneUser);

    FourAxisChecker::minimizeNumberPlanes(survivedPlanes, visibility);
    #ifdef MULTI_LABEL_OPTIMIZATION_INCLUDED
    std::vector<int> ass = FourAxisChecker::getAssociation(survivedPlanes, visibility, *meshEigen);
    //to know the actual orientation: survivedPlanes[ass[f]]
    int subd = 240 / survivedPlanes.size();
    for (unsigned int i = 0; i < ass.size(); i++){
        Color c;
        if (ass[i] == nPlaneUser*2)
            c = Color(0,0,0);
        else
            c.setHsv(subd*ass[i], 255, 255);
        meshEigen->setFaceColor(c, i);
    }
    #endif
    mainWindow->updateGlCanvas();
}
