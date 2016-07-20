#include "iglmeshmanager.h"
#include "ui_iglmeshmanager.h"

#include <QFileDialog>

IGLMeshManager::IGLMeshManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::IGLMeshManager),
    mainWindow((MainWindow*)parent),
    mesh (nullptr) {
    ui->setupUi(this);
}

void IGLMeshManager::setButtonsMeshLoaded(bool b) {
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

void IGLMeshManager::setIGLMesh(const IGLMesh& m) {
    if (mesh != nullptr){
        mainWindow->deleteObj(mesh);
        delete mesh;
    }
    mesh = new DrawableIGLMesh(m);
    mainWindow->pushObj(mesh, "IGLMesh");
    setButtonsMeshLoaded(true);
    mainWindow->updateGlCanvas();

}

IGLMeshManager::~IGLMeshManager() {
    delete ui;
    if (mesh != nullptr){
        mainWindow->deleteObj(mesh);
        delete mesh;
    }
}

void IGLMeshManager::on_loadIGLMeshButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open IGL Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    if (!filename.isEmpty())
    {
        mesh = new DrawableIGLMesh();
        mesh->readFromFile(filename.toStdString());
        mainWindow->pushObj(mesh, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
        setButtonsMeshLoaded(true);
        mainWindow->updateGlCanvas();
    }

}

void IGLMeshManager::on_clearIGLMeshButton_clicked() {
    setButtonsMeshLoaded(false);
    mainWindow->deleteObj(mesh);
    delete mesh;
    mesh = nullptr;
}

void IGLMeshManager::on_saveIGLMeshButton_clicked() {
    QString selectedFilter;
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save DCEL",
                       ".",
                       "PLY(*.ply);;OBJ(*.obj)", &selectedFilter);

    std::cout << "save: " << filename.toStdString() << std::endl;

    if (selectedFilter == "PLY(*.ply)") {
        ;
    }
    else  if (selectedFilter == "OBJ(*.obj)") {
        ;
    }
}

void IGLMeshManager::on_pointsIGLMeshRadioButton_toggled(bool checked) {
    if (checked){
        mesh->setPointShading();
        mainWindow->updateGlCanvas();
    }
}

void IGLMeshManager::on_flatIGLMeshRadioButton_toggled(bool checked) {
    if (checked){
        mesh->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void IGLMeshManager::on_smoothIGLMeshRadioButton_toggled(bool checked){
    if (checked){
        mesh->setSmoothShading();
        mainWindow->updateGlCanvas();
    }
}

void IGLMeshManager::on_wireframeIGLMeshCheckBox_stateChanged(int arg1) {
    mesh->setWireframe(arg1 == Qt::Checked);
    mainWindow->updateGlCanvas();
}
