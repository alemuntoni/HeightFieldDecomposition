#include "trimeshmanager.h"
#include "ui_trimeshmanager.h"

#include "../load_save_trimesh.h"
#include <QFileDialog>
#include <QColorDialog>

TrimeshManager::TrimeshManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::TrimeshManager),
    mainWindow((MainWindow*)parent),
    trimesh(nullptr) {
    ui->setupUi(this);
}

TrimeshManager::~TrimeshManager() {
    delete ui;
    if (trimesh != nullptr){
        mainWindow->deleteObj(trimesh);
        delete trimesh;
    }
}

void TrimeshManager::setButtonsTrimeshLoaded(bool b) {
    ui->butLoadTrimesh->setEnabled(!b);
    ui->butSaveTrimesh->setEnabled(b);
    ui->butClearTrimesh->setEnabled(b);
    ui->rbPoints->setEnabled(b);
    ui->rbFlat->setEnabled(b);
    ui->rbSmooth->setEnabled(b);
    ui->cbWireframe->setEnabled(b);
    ui->hsWireframeWidth->setEnabled(b);
    ui->butSetWireframeColor->setEnabled(b);
    ui->rbVertexColor->setEnabled(b);
    ui->rbTriangleColor->setEnabled(b);
    ui->butSetTcolor->setEnabled(b);
    ui->butSetVColor->setEnabled(b);
}

void TrimeshManager::on_butLoadTrimesh_clicked() {
    QString filename = QFileDialog::getOpenFileName(NULL,
                       "Open Trimesh",
                       ".",
                       "OBJ(*.obj)");

    std::cout << "load: " << filename.toStdString() << std::endl;

    if (!filename.isEmpty()) {
        trimesh = new DrawableTrimesh(filename.toStdString().c_str());

        mainWindow->pushObj(trimesh, "Triemsh");
        setButtonsTrimeshLoaded(true);
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_butSaveTrimesh_clicked() {
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Save Trimesh",
                       ".",
                       "OBJ(*.obj)");

    std::cout << "save: " << filename.toStdString() << std::endl;
    //Trimesh Save missing!!
    //saveObj(filename.toStdString().c_str());

}

void TrimeshManager::on_butClearTrimesh_clicked() {
    mainWindow->deleteObj(trimesh);
    delete trimesh;
    trimesh = nullptr;
    setButtonsTrimeshLoaded(false);
}

void TrimeshManager::on_cbWireframe_stateChanged(int arg1) {
    trimesh->setWireframe(arg1 == Qt::Checked);
    mainWindow->updateGlCanvas();
}

void TrimeshManager::on_rbPoints_toggled(bool checked) {
    if (checked) {
        trimesh->setPointsShading();
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_rbFlat_toggled(bool checked) {
    if (checked) {
        trimesh->setFlatShading();
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_rbSmooth_toggled(bool checked) {
    if (checked){
        trimesh->setSmoothShading();
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_butSetVColor_clicked() {
    QColor color = QColorDialog::getColor(Qt::white, this);
    trimesh->setVertexColor(color.redF(), color.greenF(), color.blueF());
    mainWindow->updateGlCanvas();
}

void TrimeshManager::on_butSetTcolor_clicked() {
    QColor color = QColorDialog::getColor(Qt::white, this);
    trimesh->setTriangleColor(color.redF(), color.greenF(), color.blueF());
    mainWindow->updateGlCanvas();
}

void TrimeshManager::on_rbVertexColor_toggled(bool checked) {
    if (checked) {
        trimesh->setEnableVertexColor();
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_rbTriangleColor_toggled(bool checked) {
    if (checked) {
        trimesh->setEnableTriangleColor();
        mainWindow->updateGlCanvas();
    }
}

void TrimeshManager::on_hsWireframeWidth_valueChanged(int value) {
    trimesh->setWireframeWidth(value);
    mainWindow->updateGlCanvas();
}

void TrimeshManager::on_butSetWireframeColor_clicked() {
    QColor color = QColorDialog::getColor(Qt::white, this);
    trimesh->setWireframeColor(color.redF(), color.greenF(), color.blueF());
    mainWindow->updateGlCanvas();
}
