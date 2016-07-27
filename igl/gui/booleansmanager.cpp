#include "booleansmanager.h"
#include "ui_booleansmanager.h"

#include <QFileDialog>
#include "common/utils.h"

BooleansManager::BooleansManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::BooleansManager),
    mainWindow((MainWindow*)parent),
    result(nullptr){
    meshes.resize(2);
    meshes[0] = meshes[1] = nullptr;
    ui->setupUi(this);
}

BooleansManager::~BooleansManager() {
    delete ui;
    for (unsigned int i = 0; i < meshes.size(); i++) {
        if (meshes[i]!=nullptr){
            mainWindow->deleteObj(meshes[i]);
            delete meshes[i];
        }
    }
}

void BooleansManager::setButtonsMeshLoaded(bool b) {
    ui->loadIGLMeshButton->setEnabled(!b);
    ui->clearIGLMeshButton->setEnabled(b);
    ui->saveIGLMeshButton->setEnabled(b);
    ui->pointsIGLMeshRadioButton->setEnabled(b);
    ui->flatIGLMeshRadioButton->setEnabled(b);
    ui->smoothIGLMeshRadioButton->setEnabled(b);
    ui->wireframeIGLMeshCheckBox->setEnabled(b);
    ui->wireframeIGLMeshCheckBox->setChecked(false);
    ui->smoothIGLMeshRadioButton->setChecked(true);
}

void BooleansManager::setButtonsMeshLoaded_2(bool b) {
    ui->loadIGLMeshButton_2->setEnabled(!b);
    ui->clearIGLMeshButton_2->setEnabled(b);
    ui->saveIGLMeshButton_2->setEnabled(b);
    ui->pointsIGLMeshRadioButton_2->setEnabled(b);
    ui->flatIGLMeshRadioButton_2->setEnabled(b);
    ui->smoothIGLMeshRadioButton_2->setEnabled(b);
    ui->wireframeIGLMeshCheckBox_2->setEnabled(b);
    ui->wireframeIGLMeshCheckBox_2->setChecked(false);
    ui->smoothIGLMeshRadioButton_2->setChecked(true);
}

void BooleansManager::setButtonsResultLoaded(bool b) {
    ui->clearIGLMeshButton_3->setEnabled(b);
    ui->saveIGLMeshButton_3->setEnabled(b);
    ui->pointsIGLMeshRadioButton_3->setEnabled(b);
    ui->flatIGLMeshRadioButton_3->setEnabled(b);
    ui->smoothIGLMeshRadioButton_3->setEnabled(b);
    ui->wireframeIGLMeshCheckBox_3->setEnabled(b);
    ui->wireframeIGLMeshCheckBox_3->setChecked(false);
    ui->smoothIGLMeshRadioButton_3->setChecked(true);
}

void BooleansManager::setMesh1(const IGLMesh& m) {
    if (meshes[0] != nullptr){
        mainWindow->deleteObj(meshes[0]);
        delete meshes[0];
    }
    meshes[0] = new DrawableIGLMesh(m);
    mainWindow->pushObj(meshes[0], "IglMesh");
    setButtonsMeshLoaded(true);
    mainWindow->updateGlCanvas();
}

void BooleansManager::setMesh2(const IGLMesh& m) {
    if (meshes[1] != nullptr){
        mainWindow->deleteObj(meshes[1]);
        delete meshes[1];
    }
    meshes[1] = new DrawableIGLMesh(m);
    mainWindow->pushObj(meshes[1], "IglMesh");
    setButtonsMeshLoaded_2(true);
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_loadIGLMeshButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open IGL Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    if (!filename.isEmpty()) {
        DrawableIGLMesh* mesh = new DrawableIGLMesh();
        mesh->readFromFile(filename.toStdString());
        mainWindow->pushObj(mesh, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
        setButtonsMeshLoaded(true);
        mainWindow->updateGlCanvas();
        if (meshes[0] != nullptr){
            mainWindow->deleteObj(meshes[0]);
            delete meshes[0];
        }
        meshes[0] = mesh;
    }
}

void BooleansManager::on_clearIGLMeshButton_clicked() {
    setButtonsMeshLoaded(false);
    mainWindow->deleteObj(meshes[0]);
    delete meshes[0];
    meshes[0] = nullptr;
}

void BooleansManager::on_saveIGLMeshButton_clicked() {
    QString selectedFilter;
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save DCEL",
                       ".",
                       "PLY(*.ply);;OBJ(*.obj)", &selectedFilter);

    std::cout << "save: " << filename.toStdString() << std::endl;

    if (selectedFilter == "PLY(*.ply)") {
        meshes[0]->saveOnPly(filename.toStdString());
    }
    else  if (selectedFilter == "OBJ(*.obj)") {
        meshes[0]->saveOnObj(filename.toStdString());
    }
}

void BooleansManager::on_setFromResultButton_clicked() {
    if (result != nullptr){
        if (meshes[0] != nullptr){
            mainWindow->deleteObj(meshes[0]);
            delete meshes[0];
        }
        meshes[0] = new DrawableIGLMesh(*result);
        mainWindow->pushObj(meshes[0], "Result");
        setButtonsMeshLoaded(true);
        mainWindow->updateGlCanvas();

    }
}

void BooleansManager::on_pointsIGLMeshRadioButton_toggled(bool checked) {
    if (checked){
        meshes[0]->setPointShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_flatIGLMeshRadioButton_toggled(bool checked) {
    if (checked){
        meshes[0]->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_smoothIGLMeshRadioButton_toggled(bool checked) {
    if (checked){
        meshes[0]->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_wireframeIGLMeshCheckBox_stateChanged(int arg1) {
    meshes[0]->setWireframe(arg1 == Qt::Checked);
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_loadIGLMeshButton_2_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open IGL Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    if (!filename.isEmpty()) {
        DrawableIGLMesh* mesh = new DrawableIGLMesh();
        mesh->readFromFile(filename.toStdString());
        mainWindow->pushObj(mesh, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
        setButtonsMeshLoaded_2(true);
        mainWindow->updateGlCanvas();
        if (meshes[1] != nullptr){
            mainWindow->deleteObj(meshes[1]);
            delete meshes[1];
        }
        meshes[1] = mesh;
    }
}

void BooleansManager::on_clearIGLMeshButton_2_clicked() {
    setButtonsMeshLoaded_2(false);
    mainWindow->deleteObj(meshes[1]);
    delete meshes[1];
    meshes[1] = nullptr;
}

void BooleansManager::on_saveIGLMeshButton_2_clicked() {
    QString selectedFilter;
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save DCEL",
                       ".",
                       "PLY(*.ply);;OBJ(*.obj)", &selectedFilter);

    std::cout << "save: " << filename.toStdString() << std::endl;

    if (selectedFilter == "PLY(*.ply)") {
        meshes[1]->saveOnPly(filename.toStdString());
    }
    else  if (selectedFilter == "OBJ(*.obj)") {
        meshes[1]->saveOnObj(filename.toStdString());
    }
}

void BooleansManager::on_setFromResultButton_2_clicked() {
    if (result != nullptr){
        if (meshes[1] != nullptr){
            mainWindow->deleteObj(meshes[1]);
            delete meshes[1];
        }
        meshes[1] = new DrawableIGLMesh(*result);
        mainWindow->pushObj(meshes[1], "Result");
        setButtonsMeshLoaded(true);
        mainWindow->updateGlCanvas();

    }
}

void BooleansManager::on_pointsIGLMeshRadioButton_2_toggled(bool checked) {
    if (checked){
        meshes[1]->setPointShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_flatIGLMeshRadioButton_2_toggled(bool checked) {
    if (checked){
        meshes[1]->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_smoothIGLMeshRadioButton_2_toggled(bool checked) {
    if (checked){
        meshes[1]->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_wireframeIGLMeshCheckBox_2_stateChanged(int arg1) {
    meshes[1]->setWireframe(arg1 == Qt::Checked);
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_intersectionButton_clicked() {
    if (meshes[0] != nullptr && meshes[1] != nullptr){
        if (result != nullptr) {
            mainWindow->deleteObj(result);
            delete result;
        }
        result = new DrawableIGLMesh();
        IGLMesh::intersection(*result, *(meshes[0]), *(meshes[1]));
        mainWindow->pushObj(result, "Intersection");
        mainWindow->updateGlCanvas();
        setButtonsResultLoaded(true);
    }
}

void BooleansManager::on_differenceButton_clicked() {
    if (meshes[0] != nullptr && meshes[1] != nullptr){
        if (result != nullptr) {
            mainWindow->deleteObj(result);
            delete result;
        }
        result = new DrawableIGLMesh();
        IGLMesh::difference(*result, *(meshes[0]), *(meshes[1]));
        mainWindow->pushObj(result, "Difference");
        mainWindow->updateGlCanvas();
        setButtonsResultLoaded(true);
    }
}

void BooleansManager::on_unionButton_clicked() {
    if (meshes[0] != nullptr && meshes[1] != nullptr){
        if (result != nullptr) {
            mainWindow->deleteObj(result);
            delete result;
        }
        result = new DrawableIGLMesh();
        IGLMesh::unionn(*result, *(meshes[0]), *(meshes[1]));
        mainWindow->pushObj(result, "Union");
        mainWindow->updateGlCanvas();
        setButtonsResultLoaded(true);
    }
}

void BooleansManager::on_clearIGLMeshButton_3_clicked() {
    mainWindow->deleteObj(result);
    delete result;
    result = nullptr;
    setButtonsResultLoaded(false);
}

void BooleansManager::on_saveIGLMeshButton_3_clicked() {
    QString selectedFilter;
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save DCEL",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)", &selectedFilter);

    std::cout << "save: " << filename.toStdString() << std::endl;

    if (selectedFilter == "PLY(*.ply)") {
        result->saveOnPly(filename.toStdString());
    }
    else  if (selectedFilter == "OBJ(*.obj)") {
        result->saveOnObj(filename.toStdString());
    }
}

void BooleansManager::on_pointsIGLMeshRadioButton_3_toggled(bool checked) {
    if (checked){
        result->setPointShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_flatIGLMeshRadioButton_3_toggled(bool checked) {
    if (checked){
        result->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_smoothIGLMeshRadioButton_3_toggled(bool checked) {
    if (checked){
        result->setSmoothShading();
        mainWindow->updateGlCanvas();
    }
}

void BooleansManager::on_wireframeIGLMeshCheckBox_3_stateChanged(int arg1) {
    result->setWireframe(arg1 == Qt::Checked);
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_plusXButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(ui->stepSpinBox->value(), 0, 0));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(ui->stepSpinBox->value(), 0, 0));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(ui->stepSpinBox->value(), 0, 0));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_minusXButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(-ui->stepSpinBox->value(), 0, 0));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(-ui->stepSpinBox->value(), 0, 0));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(-ui->stepSpinBox->value(), 0, 0));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_plusYButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(0, ui->stepSpinBox->value(), 0));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(0, ui->stepSpinBox->value(), 0));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(0, ui->stepSpinBox->value(), 0));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_minusYButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(0, -ui->stepSpinBox->value(), 0));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(0, -ui->stepSpinBox->value(), 0));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(0, -ui->stepSpinBox->value(), 0));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_plusZButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(0, 0, ui->stepSpinBox->value()));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(0, 0, ui->stepSpinBox->value()));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(0, 0, ui->stepSpinBox->value()));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_minusZButton_clicked() {
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->translate(Pointd(0, 0, -ui->stepSpinBox->value()));
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->translate(Pointd(0, 0, -ui->stepSpinBox->value()));
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->translate(Pointd(0, 0, -ui->stepSpinBox->value()));
    }
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_rotateButton_clicked() {
    Vec3 axis(ui->axisXSpinBox->value(), ui->axisYSpinBox->value(), ui->axisZSpinBox->value());
    double angle = ui->angleSpinBox->value()*M_PI/180;
    lastAxis = axis;
    lastAngle = angle;
    Eigen::Matrix3d m;
    getRotationMatrix(axis, angle, m);
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->rotate(m);
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->rotate(m);
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->rotate(m);
    }
    ui->undoRotateButton->setEnabled(true);
    mainWindow->updateGlCanvas();
}

void BooleansManager::on_undoRotateButton_clicked() {
    Eigen::Matrix3d m;
    getRotationMatrix(-lastAxis, lastAngle, m);
    if (ui->mesh1CheckBox->isChecked() && meshes[0] != nullptr){
        meshes[0]->rotate(m);
    }
    if (ui->mesh2CheckBox->isChecked() && meshes[1] != nullptr){
        meshes[1]->rotate(m);
    }
    if (ui->resultCheckBox->isChecked() && result != nullptr){
        result->rotate(m);
    }
    ui->undoRotateButton->setEnabled(false);
    mainWindow->updateGlCanvas();
}
