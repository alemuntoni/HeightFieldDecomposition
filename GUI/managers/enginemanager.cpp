#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>

EngineManager::EngineManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::EngineManager),
    mainWindow((MainWindow*)parent),
    g(nullptr),
    d(nullptr){
    ui->setupUi(this);
}

void EngineManager::deleteDrawableObject(DrawableObject* d) {
    if (d != nullptr){
        d->setVisible(false);
        mainWindow->deleteObj(d);
        delete d;
        d = nullptr;
    }
}

EngineManager::~EngineManager() {
    delete ui;
}

void EngineManager::serialize(std::ofstream& binaryFile) const {
    g->serialize(binaryFile);
    d->serialize(binaryFile);
}

void EngineManager::deserialize(std::ifstream& binaryFile) {
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    g = new DrawableGrid();
    d = new DrawableDcel();
    g->deserialize(binaryFile);
    d->deserialize(binaryFile);
    d->update();
    mainWindow->pushObj(d, "Scaled Mesh");
    mainWindow->pushObj(g, "Grid");
}

void EngineManager::on_generateGridPushButton_clicked() {
    DcelManager* dm = (DcelManager*)mainWindow->getManager(DCEL_MANAGER_ID);
    DrawableDcel* dd = dm->getDcel();
    if (dd != nullptr){
        deleteDrawableObject(d);
        d = new DrawableDcel(*dd);
        mainWindow->pushObj(d, "Scaled Mesh");
        int s = ui->samplesSpinBox->value();
        BoundingBox bb = d->getBoundingBox();
        double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
        maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
        double av = maxl / s;
        BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);
        d->scale(nBB);

        //double m[3][3];
        //getRotationMatrix(Vec3(0,0,1), 0.785398, m);
        //d->rotate(m);

        d->update();
        d->saveOnObjFile("tmp.obj");
        exec("./grid_generator tmp.obj");


        Eigen::RowVector3i nGmin;
        Eigen::RowVector3i nGmax;
        Eigen::VectorXd S;
        Eigen::MatrixXd GV;
        Eigen::RowVector3i res;

        std::ifstream file;
        file.open ("tmp.bin", std::ios::in | std::ios::binary);
        Serializer::deserialize(nGmin, file);
        Serializer::deserialize(nGmax, file);
        Serializer::deserialize(res, file);
        Serializer::deserialize(GV, file);
        Serializer::deserialize(S, file);
        file.close();

        if (g!=nullptr){
            g->setVisible(false);
            mainWindow->updateGlCanvas();
            mainWindow->deleteObj(g);
            delete g;
            g = nullptr;
        }
        g = new DrawableGrid(res, GV, S, nGmin, nGmax);
        g->setKernelDistance(ui->distanceSpinBox->value());
        mainWindow->pushObj(g, "Grid");
        g->setTarget(XYZ[ui->targetComboBox->currentIndex()]);
        g->calculateWeights(*d);
        mainWindow->updateGlCanvas();


        std::remove("tmp.bin");
        std::remove("tmp.obj");
        std::remove("tmp.mtu");

    }
}

void EngineManager::on_distanceSpinBox_valueChanged(double arg1) {
    if (g!=nullptr){
        g->setKernelDistance(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_targetComboBox_currentIndexChanged(int index) {
    DcelManager* dm = (DcelManager*)mainWindow->getManager(DCEL_MANAGER_ID);
    DrawableDcel* d = dm->getDcel();
    if (d!= nullptr && g!= nullptr){
        g->setTarget(XYZ[index]);
        g->calculateWeights(*d);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_kernelRadioButton_toggled(bool checked) {
    if (checked && g!=nullptr){
        g->setDrawKernel();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_weigthsRadioButton_toggled(bool checked) {
    if (checked && g!=nullptr){
        g->setDrawBorders();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_freezeKernelPushButton_clicked() {
    if (g!=nullptr && d!=nullptr){
        double value = ui->distanceSpinBox->value();
        g->freezeKernel(*d, value);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_sliceCheckBox_stateChanged(int arg1) {
    if (g!=nullptr){
        if (arg1 == Qt::Checked){
            ui->sliceComboBox->setEnabled(true);
            ui->sliceSlider->setEnabled(true);
            int s = ui->sliceComboBox->currentIndex();
            g->setSliceValue(0);
            g->setSlice(s+1);
            if (s == 0) ui->sliceSlider->setMaximum(g->getResX() -1);
            if (s == 1) ui->sliceSlider->setMaximum(g->getResY() -1);
            if (s == 2) ui->sliceSlider->setMaximum(g->getResZ() -1);
        }
        else {
            ui->sliceComboBox->setEnabled(false);
            ui->sliceSlider->setEnabled(false);
            g->setSlice(0);
        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_sliceSlider_valueChanged(int value) {
    if (g!=nullptr){
        g->setSliceValue(value);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_sliceComboBox_currentIndexChanged(int index) {
    if (g!=nullptr){
        g->setSliceValue(0);
        g->setSlice(index+1);
        ui->sliceSlider->setValue(0);
        if (index == 0) ui->sliceSlider->setMaximum(g->getResX() -1);
        if (index == 1) ui->sliceSlider->setMaximum(g->getResY() -1);
        if (index == 2) ui->sliceSlider->setMaximum(g->getResZ() -1);
    }
}

void EngineManager::on_pushButton_clicked() {
    std::ofstream myfile;
    myfile.open ("engine.bin", std::ios::out | std::ios::binary);
    serialize(myfile);
    myfile.close();
}

void EngineManager::on_pushButton_2_clicked() {
    std::ifstream myfile;
    myfile.open ("engine.bin", std::ios::in | std::ios::binary);
    deserialize(myfile);
    myfile.close();
}
