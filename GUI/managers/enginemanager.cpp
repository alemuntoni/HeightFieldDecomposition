#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>
#include <QFileDialog>
#include <QMessageBox>
#include <omp.h>
#include "cgal/aabbtree.h"
#include "engine/packing.h"
#include "igl/iglinterface.h"
#include "engine/reconstruction.h"
#include "engineworker.h"
#include <QThread>
#include "lib/dcel_segmentation/segmentation.h"

EngineManager::EngineManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::EngineManager),
    mainWindow((MainWindow*)parent),
    g(nullptr),
    d(nullptr),
    b(nullptr),
    iterations(nullptr),
    solutions(nullptr),
    baseComplex(nullptr),
    he(nullptr){
    ui->setupUi(this);
    ui->iterationsSlider->setMaximum(0);

    //ui->frame_2->setVisible(false);
    //ui->frame_5->setVisible(false);
}

void EngineManager::deleteDrawableObject(DrawableObject* d) {
    if (d != nullptr){
        //d->setVisible(false);
        mainWindow->deleteObj(d);
        delete d;
        d = nullptr;
    }
}

EngineManager::~EngineManager() {
    delete ui;
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    deleteDrawableObject(b);
    deleteDrawableObject(iterations);
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
}

void EngineManager::updateLabel(double value, QLabel* label) {
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss << value;
    label->setText(QString::fromStdString(ss.str()));
}

void EngineManager::updateBoxValues() {
    if (b != nullptr){
        ui->minXSpinBox->setValue(b->getMinX());
        ui->minYSpinBox->setValue(b->getMinY());
        ui->minZSpinBox->setValue(b->getMinZ());
        ui->maxXSpinBox->setValue(b->getMaxX());
        ui->maxYSpinBox->setValue(b->getMaxY());
        ui->maxZSpinBox->setValue(b->getMaxZ());
        ui->wSpinBox->setValue(b->getMaxX() - b->getMinX());
        ui->hSpinBox->setValue(b->getMaxY() - b->getMinY());
        ui->dSpinBox->setValue(b->getMaxZ() - b->getMinZ());
    }
}

void EngineManager::updateColors(double angleThreshold, double areaThreshold) {
    d->setColor(QColor(128,128,128));
    std::set<const Dcel::Face*> flippedFaces, savedFaces;
    Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], angleThreshold/100, areaThreshold);

    for (const Dcel::Face* cf : flippedFaces){
        Dcel::Face* f = d->getFace(cf->getId());
        f->setColor(QColor(255,0,0));
    }
    for (const Dcel::Face* cf : savedFaces){
        Dcel::Face* f = d->getFace(cf->getId());
        f->setColor(QColor(0,0,255));
    }

    d->update();
    mainWindow->updateGlCanvas();
}

Pointd EngineManager::getLimits() {
    assert(d!=nullptr);
    BoundingBox bb = d->getBoundingBox();
    double lx = bb.getLengthX();
    double ly = bb.getLengthY();
    double lz = bb.getLengthZ();
    double min;
    if (lx <= ly && lx <= lz){
        min = lx;
    }
    else if (ly <= lx && ly <= lz) {
        min = ly;
    }
    else if (lz <= lx && lz <= ly) {
        min = lz;
    } else assert(0);
    double minreal = ui->minEdgeBBSpinBox->value();
    double limitRealX = ui->xLimitSpinBox->value();
    double limitRealY = ui->xLimitSpinBox->value();
    double limitRealZ = ui->zLimitSpinBox->value();
    Pointd limits;
    limits.x() = (limitRealX * min) / minreal;
    limits.y() = (limitRealY * min) / minreal;
    limits.z() = (limitRealZ * min) / minreal;
    return limits;
}

void EngineManager::serializeBC(const std::string& filename) {
    std::ofstream myfile;
    myfile.open (filename, std::ios::out | std::ios::binary);
    d->serialize(myfile);
    solutions->serialize(myfile);
    baseComplex->serialize(myfile);
    he->serialize(myfile);
    originalMesh.serialize(myfile);
    //entirePieces->serialize(myfile);
    double factor = ui->factorSpinBox->value();
    double kernel = ui->distanceSpinBox->value();
    Serializer::serialize(factor, myfile);
    Serializer::serialize(kernel, myfile);
    myfile.close();
}

void EngineManager::deserializeBC(const std::string& filename) {
    deleteDrawableObject(d);
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
    d = new DrawableDcel();
    solutions = new BoxList();
    baseComplex = new IGLInterface::DrawableIGLMesh();
    he = new HeightfieldsList();
    std::ifstream myfile;
    myfile.open (filename, std::ios::in | std::ios::binary);
    d->deserialize(myfile);
    solutions->deserialize(myfile);
    baseComplex->deserialize(myfile);
    he->deserialize(myfile);
    if (originalMesh.deserialize(myfile)){
        if (originalMesh.getNumberVertices() > 0)
            mainWindow->pushObj(&originalMesh, "Original Mesh");
    }
    double factor, kernel;
    if (Serializer::deserialize(factor, myfile) && Serializer::deserialize(kernel, myfile)){
        ui->factorSpinBox->setValue(factor);
        ui->distanceSpinBox->setValue(kernel);
    }
    myfile.close();
    d->update();
    d->setPointsShading();
    d->setWireframe(true);
    mainWindow->pushObj(d, "Input Mesh");
    mainWindow->pushObj(solutions, "Boxes");
    mainWindow->pushObj(baseComplex, "Base Complex");
    mainWindow->pushObj(he, "Heightfields");
    mainWindow->updateGlCanvas();
    ui->showAllSolutionsCheckBox->setEnabled(true);
    //he->explode(40);
    solutions->setVisibleBox(0);
    ui->heightfieldsSlider->setMaximum(he->getNumHeightfields()-1);
    ui->allHeightfieldsCheckBox->setChecked(true);
    ui->solutionsSlider->setEnabled(true);
    ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
    ui->setFromSolutionSpinBox->setValue(0);
    ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
}

void EngineManager::serialize(std::ofstream& binaryFile) const {
    //g->serialize(binaryFile);
    d->serialize(binaryFile);
    bool bb = false;

    bb = false;
    if (solutions!=nullptr){
        bb = true;
        Serializer::serialize(bb, binaryFile);
        solutions->serialize(binaryFile);
    }
    else
        Serializer::serialize(bb, binaryFile);
    if (originalMesh.getNumberVertices() > 0){
        bb = true;
        Serializer::serialize(bb, binaryFile);
        originalMesh.serialize(binaryFile);
    }
    double factor = ui->factorSpinBox->value();
    double kernel = ui->distanceSpinBox->value();
    Serializer::serialize(factor, binaryFile);
    Serializer::serialize(kernel, binaryFile);
}

bool EngineManager::deserialize(std::ifstream& binaryFile) {
    //deleteDrawableObject(g);
    deleteDrawableObject(d);
    //g = new DrawableGrid();
    d = new DrawableDcel();
    //g->deserialize(binaryFile);
    if (! d->deserialize(binaryFile)) return false;
    bool bb = false;
    for (Dcel::FaceIterator fit = d->faceBegin(); fit != d->faceEnd(); ++fit)
        (*fit)->setColor(QColor(128,128,128));
    d->setWireframe(true);
    d->setPointsShading();
    //updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
    d->update();
    mainWindow->pushObj(d, "Scaled Mesh");
    //mainWindow->pushObj(g, "Grid");
    //e = Energy(*g);
    ui->weigthsRadioButton->setChecked(true);
    ui->sliceCheckBox->setChecked(true);
    //g->setDrawBorders();
    //g->setSlice(1);

    if (! Serializer::deserialize(bb, binaryFile)) return false;
    if (bb){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        if (! solutions->deserialize(binaryFile)) return false;
        if (solutions->getNumberBoxes() > 0){
            solutions->setVisibleBox(0);
            solutions->setCylinders(false);
            mainWindow->pushObj(solutions, "Solutions");
            ui->showAllSolutionsCheckBox->setEnabled(true);
            ui->solutionsSlider->setEnabled(true);
            ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
            ui->setFromSolutionSpinBox->setValue(0);
            ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        }
        else
            std::cerr << "ERROR: no boxes in binary file. ???\n";
    }
    if (Serializer::deserialize(bb, binaryFile)){
        originalMesh.deserialize(binaryFile);
        mainWindow->pushObj(&originalMesh, "Original Mesh");
    }
    double kernel, factor;
    if (Serializer::deserialize(factor, binaryFile) && Serializer::deserialize(kernel, binaryFile)){
        ui->factorSpinBox->setValue(factor);
        ui->distanceSpinBox->setValue(kernel);
    }

    mainWindow->updateGlCanvas();
    return true;
}

void EngineManager::on_generateGridPushButton_clicked() {
    if (d != nullptr){
        deleteDrawableObject(g);
        g = new DrawableGrid();
        BoundingBox bb= d->getBoundingBox();
        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        originalMesh.scale(bb, d->getBoundingBox());
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        Engine::generateGrid(*g, *d, ui->distanceSpinBox->value(), ui->heightfieldsCheckBox->isChecked(), XYZ[ui->targetComboBox->currentIndex()], savedFaces);
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
        g->updateMinSignedDistance();
        g->setKernelDistance(ui->distanceSpinBox->value());
        e = Energy(*g);
        mainWindow->pushObj(g, "Grid");
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_distanceSpinBox_valueChanged(double arg1) {
    if (g!=nullptr){
        g->setKernelDistance(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_targetComboBox_currentIndexChanged(int index) {
    if (d!= nullptr && g!= nullptr){
        g->setTarget(XYZ[index]);
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        //std::set<const Dcel::Face*> flippedFaces, savedFaces;
        //Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        //g->calculateBorderWeights(*d, ui->heightfieldsCheckBox->isChecked(), savedFaces);
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
        g->setTarget(XYZ[ui->targetComboBox->currentIndex()]);
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        g->calculateWeightsAndFreezeKernel(*d, value, ui->heightfieldsCheckBox->isChecked(), savedFaces);
        e = Energy(*g);
        e.calculateFullBoxValues(*g);
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
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_serializePushButton_clicked() {
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Serialize",
                       ".",
                       "BIN(*.bin)");
    if (!filename.isEmpty()) {
        std::ofstream myfile;
        myfile.open (filename.toStdString(), std::ios::out | std::ios::binary);
        serialize(myfile);
        myfile.close();
    }
}

void EngineManager::on_deserializePushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Deserialize",
                       ".",
                       "BIN(*.bin)");

    if (!filename.isEmpty()) {
        std::ifstream myfile;
        myfile.open (filename.toStdString(), std::ios::in | std::ios::binary);
        deserialize(myfile);
        myfile.close();
    }
}

void EngineManager::on_saveObjsButton_clicked() {
    if (d != nullptr && baseComplex != nullptr && he != nullptr) {
        QString foldername = QFileDialog::getExistingDirectory(nullptr, "SaveObjs");
        if (!foldername.isEmpty()){
            /*QString originalMeshString = foldername + "/OriginalMesh.obj";
            QString inputMeshString = foldername + "/InputMesh.obj";
            QString baseComplexString = foldername + "/BaseComplex.obj";
            QString heightfieldString = foldername + "/Heightfield";
            if (originalMesh.getNumberVertices() > 0)
                originalMesh.saveOnObj(originalMeshString.toStdString());
            d->saveOnObjFile(inputMeshString.toStdString());
            baseComplex->saveOnObj(baseComplexString.toStdString());
            for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
                IGLInterface::IGLMesh h = he->getHeightfield(i);
                std::stringstream ss;
                ss << heightfieldString.toStdString() << i << ".obj";
                h.saveOnObj(ss.str());
            }*/
            Engine::saveObjs(foldername, originalMesh, *d, *baseComplex, *he);
        }
    }
}

void EngineManager::on_wSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setW(arg1);
        updateBoxValues();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_hSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setH(arg1);
        updateBoxValues();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_dSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setD(arg1);
        updateBoxValues();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_plusXButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveX(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveX(ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x()+ui->stepSpinBox->value(), c.y(), c.z()));
            }
        }

        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minusXButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveX(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveX(- ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x()-ui->stepSpinBox->value(), c.y(), c.z()));
            }
        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_plusYButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveY(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveY(ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y()+ui->stepSpinBox->value(), c.z()));
            }

        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minusYButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveY(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveY(- ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y()-ui->stepSpinBox->value(), c.z()));
            }

        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_plusZButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveZ(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveZ(ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y(), c.z()+ui->stepSpinBox->value()));
            }

        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minusZButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveZ(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else {
            Pointd c;
            if (ui->c1RadioButton->isChecked()){
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
            }
            if (ui->c2RadioButton->isChecked()){
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
            }
            if (ui->c3RadioButton->isChecked()){
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
            }
            if (ui->allRadioButton->isChecked()){
                b->moveZ(- ui->stepSpinBox->value());
                c = b->getConstraint1();
                b->setConstraint1(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
                c = b->getConstraint2();
                b->setConstraint2(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
                c = b->getConstraint3();
                b->setConstraint3(Pointd(c.x(), c.y(), c.z()-ui->stepSpinBox->value()));
            }

        }
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_energyBoxPushButton_clicked() {
    if (b!=nullptr){
        double energy = e.energy(*b);
        Eigen::VectorXd gradient(6);
        Eigen::VectorXd solution(6);
        solution << b->getMinX(), b->getMinY(), b->getMinZ(), b->getMaxX(), b->getMaxY(), b->getMaxZ();
        e.gradientEnergy(gradient, solution, b->getConstraint1(), b->getConstraint2(), b->getConstraint3());
        updateLabel(gradient(0), ui->gminx);
        updateLabel(gradient(1), ui->gminy);
        updateLabel(gradient(2), ui->gminz);
        updateLabel(gradient(3), ui->gmaxx);
        updateLabel(gradient(4), ui->gmaxy);
        updateLabel(gradient(5), ui->gmaxz);
        std::cerr << "\nGradient: \n" << gradient << "\n";
        updateLabel(energy, ui->energyLabel);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minimizePushButton_clicked() {
    if (b!=nullptr){
        double it;
        if (ui->saveIterationsCheckBox->isChecked()){
            deleteDrawableObject(iterations);
            iterations = new BoxList();
            Timer t("Graidient Discent");
            it = e.gradientDiscend(*b, *iterations);
            t.stopAndPrint();
            iterations->setVisibleBox(0);
            ui->iterationsSlider->setMaximum(iterations->getNumberBoxes()-1);
            mainWindow->pushObj(iterations, "Iterations");

            double energy = e.energy(iterations->getBox(0));
            updateLabel(energy, ui->energyIterationLabel);
        }
        else {
            Timer t("Gradient Discent");
            it = e.gradientDiscend(*b);
            t.stopAndPrint();
        }
        updateLabel(it, ui->minimizedEnergyLabel);
        double energy = e.energy(*b);
        updateLabel(energy, ui->energyLabel);
        updateBoxValues();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_BFGSButton_clicked() {
    if (b!=nullptr){
        double it;
        if (ui->saveIterationsCheckBox->isChecked()){
            deleteDrawableObject(iterations);
            iterations = new BoxList();
            Timer t("BFGS");
            it = e.BFGS(*b, *iterations);
            t.stopAndPrint();
            iterations->setVisibleBox(0);
            ui->iterationsSlider->setMaximum(iterations->getNumberBoxes()-1);
            mainWindow->pushObj(iterations, "Iterations");

            double energy = e.energy(iterations->getBox(0));
            updateLabel(energy, ui->energyIterationLabel);
        }
        else {
            Timer t("BFGS");
            it = e.BFGS(*b);
            t.stopAndPrint();
        }
        updateLabel(it, ui->minimizedEnergyLabel);
        double energy = e.energy(*b);
        updateLabel(energy, ui->energyLabel);
        updateBoxValues();
        mainWindow->updateGlCanvas();
    }
}


void EngineManager::on_serializeBoxPushButton_clicked() {
    if (b!=nullptr){
        std::ofstream myfile;
        myfile.open ("box.bin", std::ios::out | std::ios::binary);
        b->serialize(myfile);
        myfile.close();
    }
}

void EngineManager::on_deserializeBoxPushButton_clicked() {
    if (b == nullptr){
        b = new Box3D();
        mainWindow->pushObj(b, "Box", false);
    }

    std::ifstream myfile;
    myfile.open ("box.bin", std::ios::in | std::ios::binary);
    b->deserialize(myfile);
    myfile.close();
    updateBoxValues();
    mainWindow->updateGlCanvas();
}

void EngineManager::on_iterationsSlider_sliderMoved(int position) {
    if (iterations != nullptr){
        iterations->setVisibleBox(position);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_energyIterationsButton_clicked() {
    if (iterations != nullptr){
        Box3D b = iterations->getBox(ui->iterationsSlider->value());
        double energy = e.energy(b);
        Eigen::VectorXd gradient(6);
        e.gradientTricubicInterpolationEnergy(gradient, b.getMin(), b.getMax());
        std::cerr << "Gradient: \n" << gradient << "\n";
        updateLabel(energy, ui->energyIterationLabel);
    }
}

void EngineManager::on_showAllSolutionsCheckBox_stateChanged(int arg1) {
    if (arg1 == Qt::Checked){
        ui->solutionsSlider->setEnabled(false);
        ui->solutionsSlider->setValue(0);
        solutions->setCylinders(true);
        solutions->setVisibleBox(-1);
    }
    else {
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setValue(0);
        solutions->setCylinders(false);
        solutions->setVisibleBox(0);
    }
    ui->solutionNumberLabel->setNum((int)solutions->getNumberBoxes());
    mainWindow->updateGlCanvas();
}

void EngineManager::on_solutionsSlider_valueChanged(int value) {
    if (ui->solutionsSlider->isEnabled()){
        solutions->setVisibleBox(value);
        ui->setFromSolutionSpinBox->setValue(value);
        ui->solutionNumberLabel->setText(QString::number(value));
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_setFromSolutionButton_clicked() {
    if (solutions != nullptr){
        unsigned int value = ui->setFromSolutionSpinBox->value();
        if (value < solutions->getNumberBoxes()){
            if (b == nullptr){
                b = new Box3D(solutions->getBox(value));
                mainWindow->pushObj(b, "Box");
            }
            else {
                b->setMin(solutions->getBox(value).getMin());
                b->setMax(solutions->getBox(value).getMax());
                b->setConstraint1(solutions->getBox(value).getConstraint1());
                b->setConstraint2(solutions->getBox(value).getConstraint2());
                b->setConstraint3(solutions->getBox(value).getConstraint3());
                b->setColor(solutions->getBox(value).getColor());
                b->setRotationMatrix(solutions->getBox(value).getRotationMatrix());
                b->setTarget(solutions->getBox(value).getTarget());
            }
            //deleteDrawableObject(b);
            //solutions->getBox(value);
            //b = new Box3D(solutions->getBox(value));
            //mainWindow->pushObj(b, "Box");
            mainWindow->updateGlCanvas();
        }

    }
}

void EngineManager::on_wireframeDcelCheckBox_stateChanged(int arg1) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        d->setWireframe(arg1 == Qt::Checked);
        mainWindow->updateGlCanvas();
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        baseComplex->setWireframe(arg1 == Qt::Checked);
        mainWindow->updateGlCanvas();
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        he->setWireframe(arg1 == Qt::Checked);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_pointsDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setPointsShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setPointsShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setPointShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_flatDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setFlatShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setFlatShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setFlatShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_smoothDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setSmoothShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setSmoothShading();
            mainWindow->updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setSmoothShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_trianglesCoveredPushButton_clicked() {
    if (d!=nullptr && b != nullptr) {
        Eigen::Matrix3d m[ORIENTATIONS];
        Eigen::Matrix3d mb = b->getRotationMatrix();
        m[0] = Eigen::Matrix3d::Identity();
        #if ORIENTATIONS > 1
        getRotationMatrix(Vec3(0,0,-1), 0.785398, m[1]);
        getRotationMatrix(Vec3(-1,0,0), 0.785398, m[2]);
        getRotationMatrix(Vec3(0,-1,0), 0.785398, m[3]);
        #endif
        Dcel dd = *d;
        std::list<const Dcel::Face*> covered;
        if (mb == m[0]){
            CGALInterface::AABBTree t(dd);
            t.getIntersectedDcelFaces(covered, *b);
        }
        #if ORIENTATIONS > 1
        else if (mb == m[1]){

            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(0,0,1), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedDcelFaces(covered, *b);
        }
        else if (mb == m[2]){
            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(1,0,0), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedDcelFaces(covered, *b);
        }
        else if (mb == m[3]){
            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(0,1,0), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedDcelFaces(covered, *b);
        }
        #endif
        else assert(0);


        std::list<const Dcel::Face*>::iterator i = covered.begin();
        while (i != covered.end()) {
            const Dcel::Face* f = *i;
            Pointd p1 = f->getVertex1()->getCoordinate(), p2 = f->getVertex2()->getCoordinate(), p3 = f->getVertex3()->getCoordinate();

            if (!b->isIntern(p1) || !b->isIntern(p2) || !b->isIntern(p3)) {
                i =covered.erase(i);
            }
            else ++i;
        }


        for (std::list<const Dcel::Face*>::iterator it = covered.begin(); it != covered.end(); ++it){
            const Dcel::Face* cf = *it;
            Dcel::Face* f = d->getFace(cf->getId());
            f->setColor(QColor(0,0,255));
        }
        d->update();

        mainWindow->updateGlCanvas();

    }
}

void EngineManager::on_stepDrawGridSpinBox_valueChanged(double arg1) {
    if (g!=nullptr){
        if (arg1 > 0){
            g->setStepDrawGrid(arg1);
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_subtractPushButton_clicked() {
    if (solutions!= nullptr && d != nullptr){
        deleteDrawableObject(baseComplex);
        IGLInterface::IGLMesh m = (Dcel)*d;
        baseComplex = new IGLInterface::DrawableIGLMesh(m);
        mainWindow->pushObj(baseComplex, "Base Complex");
        deleteDrawableObject(he);
        //deleteDrawableObject(entirePieces);
        he = new HeightfieldsList();
        //entirePieces = new HeightfieldsList();
        mainWindow->pushObj(he, "Heightfields");
        //mainWindow->pushObj(entirePieces, "Entire Pieces");
        mainWindow->updateGlCanvas();
        IGLInterface::SimpleIGLMesh bc((IGLInterface::SimpleIGLMesh)*baseComplex);

        /*WaitDialog* dialog = new WaitDialog("Prova", mainWindow);
        connect(this, SIGNAL(finished()), dialog, SLOT(pop()));

        dialog->show();
        EngineWorker* ew = new EngineWorker();
        QThread* thread = new QThread;
        ew->moveToThread(thread);
        connect(ew, SIGNAL (error(QString)), this, SLOT (errorString(QString)));
        connect(thread, SIGNAL (started()), ew, SLOT (booleanOperations(*he, bc, *solutions, *d)));
        connect(ew, SIGNAL (finished()), thread, SLOT (quit()));
        connect(ew, SIGNAL (finished()), ew, SLOT (deleteLater()));
        connect(thread, SIGNAL (finished()), thread, SLOT (deleteLater()));
        connect(thread, SIGNAL (finished()), this, SLOT(emit finished()));
        thread->start();*/

        ///cleaning solutions
        Engine::booleanOperations(*he, bc, *solutions);
        Engine::splitConnectedComponents(*he, *solutions);
        Engine::glueInternHeightfieldsToBaseComplex(*he, *solutions, bc, *d);
        ui->showAllSolutionsCheckBox->setEnabled(true);
        solutions->setVisibleBox(0);
        ui->heightfieldsSlider->setMaximum(he->getNumHeightfields()-1);
        ui->allHeightfieldsCheckBox->setChecked(true);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow->deleteObj(baseComplex);
        delete baseComplex;
        baseComplex = new IGLInterface::DrawableIGLMesh(bc);
        baseComplex->updateFaceNormals();
        for (unsigned int i = 0; i < baseComplex->getNumberFaces(); ++i){
            Vec3 n = baseComplex->getNormal(i);
            n.normalize();
            QColor c = colorOfNearestNormal(n);
            baseComplex->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow->pushObj(baseComplex, "Base Complex");
        mainWindow->updateGlCanvas();
        baseComplex->saveOnObj("BaseComplex.obj");
    }
}

void EngineManager::on_stickPushButton_clicked() {
    if (d!=nullptr && baseComplex != nullptr && he != nullptr){
        IGLInterface::SimpleIGLMesh bc = *baseComplex;
        Engine::reduceHeightfields(*he, bc, *d);
        deleteDrawableObject(baseComplex);
        baseComplex = new IGLInterface::DrawableIGLMesh(bc);
        baseComplex->updateFaceNormals();
        for (unsigned int i = 0; i < baseComplex->getNumberFaces(); ++i){
            Vec3 n = baseComplex->getNormal(i);
            n.normalize();
            QColor c = colorOfNearestNormal(n);
            baseComplex->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow->pushObj(baseComplex, "Base Complex");
    }
    mainWindow->updateGlCanvas();
}

void EngineManager::on_serializeBCPushButton_clicked() {
    if (baseComplex != nullptr && solutions != nullptr && d != nullptr && he != nullptr /*&& entirePieces != nullptr*/){
        QString filename = QFileDialog::getSaveFileName(nullptr,
                           "Serialize BC",
                           ".",
                           "BIN(*.bin)");
        if (!filename.isEmpty()) {
            serializeBC(filename.toStdString());
        }
    }

}

void EngineManager::on_deserializeBCPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Deserialize BC",
                       ".",
                       "BIN(*.bin)");

    if (!filename.isEmpty()) {
        deserializeBC(filename.toStdString());
    }
}

void EngineManager::on_createAndMinimizeAllPushButton_clicked() {
    if (g == nullptr && d!= nullptr) {
        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
        mainWindow->updateGlCanvas();
    }
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        mainWindow->pushObj(solutions, "Solutions");
        double kernelDistance = ui->distanceSpinBox->value();
        Timer t("Total Time Grids and Minimization Boxes");
        /// here d is already scaled!!
        if (ui->limitsConstraintCheckBox->isChecked()){
            Engine::optimizeAndDeleteBoxes(*solutions, *d, kernelDistance, true, getLimits(), ui->heightfieldsCheckBox->isChecked(), ui->onlyNearestTargetCheckBox->isChecked(), ui->areaToleranceSpinBox->value(), (double)ui->toleranceSlider->value()/100, ui->useFileCheckBox->isChecked());
        }
        else {
            Engine::optimizeAndDeleteBoxes(*solutions, *d, kernelDistance, false, Pointd(), ui->heightfieldsCheckBox->isChecked(), ui->onlyNearestTargetCheckBox->isChecked(), ui->areaToleranceSpinBox->value(), (double)ui->toleranceSlider->value()/100, ui->useFileCheckBox->isChecked());
        }
        t.stopAndPrint();
        ui->showAllSolutionsCheckBox->setEnabled(true);
        solutions->setVisibleBox(0);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_allHeightfieldsCheckBox_stateChanged(int arg1) {
    if (arg1 == Qt::Checked){
        if (he != nullptr){
            he->setVisibleHeightfield(-1);
            //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(-1);
            mainWindow->updateGlCanvas();
        }
    }
    else {
        if (he != nullptr){
            //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(ui->heightfieldsSlider->value());
            he->setVisibleHeightfield(ui->heightfieldsSlider->value());
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_heightfieldsSlider_valueChanged(int value) {
    if (he != nullptr){
        ui->solutionsSlider->setValue(value);
        //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(value);
        he->setVisibleHeightfield(value);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minXSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinX(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minYSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinY(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_minZSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinZ(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_maxXSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxX(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_maxYSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxY(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_maxZSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxZ(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_toleranceSlider_valueChanged(int value) {
    if (d!= nullptr){
        updateColors(value, ui->areaToleranceSpinBox->value());
    }
}

void EngineManager::on_areaToleranceSpinBox_valueChanged(double arg1){
    if (d!= nullptr){
        updateColors(ui->toleranceSlider->value(), arg1);
    }
}

void EngineManager::on_cleanAllPushButton_clicked() {
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    deleteDrawableObject(b);
    deleteDrawableObject(iterations);
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
    mainWindow->deleteObj(&originalMesh);
    originalMesh.clear();
    g = nullptr;
    d = nullptr;
    b = nullptr;
    iterations = nullptr;
    solutions = nullptr;
    baseComplex = nullptr;
    he = nullptr;
    //deleteDrawableObject(entirePieces);
}

void EngineManager::on_reorderBoxes_clicked() {
    if (d != nullptr && solutions != nullptr){
        Array2D<int> ordering = Splitting::getOrdering(*solutions, *d);
        solutions->setIds();
        solutions->sort(ordering);
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            std::cerr << solutions->getBox(i).getId() << " ";
        }
        std::cerr << "\n";
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_loadOriginalPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open IGL Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    if (!filename.isEmpty()) {
        originalMesh.readFromFile(filename.toStdString());
        if (! (mainWindow->contains(&originalMesh)))
            mainWindow->pushObj(&originalMesh, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_loadSmoothedPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open IGL Mesh",
                       ".",
                       "OBJ(*.obj);;PLY(*.ply)");
    if (!filename.isEmpty()) {
        std::string s = filename.toStdString();
        deleteDrawableObject(d);
        d = new DrawableDcel();
        if (d->loadFromFile(s)){
            std::cout << "load: " << filename.toStdString() << std::endl;
            d->updateVertexNormals();
            d->setWireframe(true);
            d->setPointsShading();
            d->update();
            mainWindow->pushObj(d, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
            mainWindow->updateGlCanvas();
        }
        else {
            delete d;
            d = nullptr;
            QMessageBox msgBox;
            msgBox.setText("Format file not supported.");
            msgBox.exec();
        }
    }
    /*if (originalMesh.getNumberVertices() > 0){
        cinolib::Trimesh m;
        Reconstruction::iglMeshToTrimesh(m, originalMesh);
        cinolib::smooth_taubin(m, cinolib::COTANGENT, std::set<int>(), 100, 0.89, -0.9);
        IGLInterface::SimpleIGLMesh tmp;
        Reconstruction::trimeshToIglMesh(tmp, m);
        deleteDrawableObject(d);
        d = new DrawableDcel(tmp);
        d->updateVertexNormals();
        d->setWireframe(true);
        d->setPointsShading();
        d->update();
        mainWindow->pushObj(d, "Smoothed");
        mainWindow->updateGlCanvas();
    }
    else {
        QString filename = QFileDialog::getOpenFileName(nullptr,
                                                        "Open IGL Mesh",
                                                        ".",
                                                        "OBJ(*.obj);;PLY(*.ply)");
        if (!filename.isEmpty()) {
            std::string s = filename.toStdString();
            deleteDrawableObject(d);
            d = new DrawableDcel();
            if (d->loadFromFile(s)){
                std::cout << "load: " << filename.toStdString() << std::endl;
                d->updateVertexNormals();
                d->setWireframe(true);
                d->setPointsShading();
                d->update();
                mainWindow->pushObj(d, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
                mainWindow->updateGlCanvas();
            }
            else {
                delete d;
                d = nullptr;
                QMessageBox msgBox;
                msgBox.setText("Format file not supported.");
                msgBox.exec();
            }
        }
    }*/
}

void EngineManager::on_packPushButton_clicked() {
    if (he != nullptr){
        QString foldername = QFileDialog::getExistingDirectory(nullptr, "SaveObjs");
        if (!foldername.isEmpty()){
            HeightfieldsList myHe = *he;
            Packing::rotateAllPieces(myHe);
            BoundingBox packSize(Pointd(), Pointd(ui->sizeXPackSpinBox->value(), ui->sizeYPackSpinBox->value(), ui->sizeZPackSpinBox->value()));
            BoundingBox limits = packSize;
            switch(ui->limitComboBox->currentIndex()){
                case 1:
                    limits.setMaxX(ui->limitValueSpinBox->value());
                    break;
                case 2:
                    limits.setMaxY(ui->limitValueSpinBox->value());
                    break;
                case 3:
                    limits.setMaxZ(ui->limitValueSpinBox->value());
                    break;
            }
            double factor;
            Packing::getMaximum(myHe, limits, factor);
            Packing::scaleAll(myHe, factor);
            std::vector< std::vector<std::pair<int, Pointd> > > tmp = Packing::pack(myHe, packSize);
            std::vector< std::vector<IGLInterface::IGLMesh> > packs = Packing::getPacks(tmp, myHe);
            IGLInterface::makeBox(packSize).saveOnObj(QString(foldername + "/box.obj").toStdString());
            for (unsigned int i = 0; i < packs.size(); i++){
                QString pstring = foldername + "/pack" + QString::number(i) + ".obj";
                IGLInterface::IGLMesh packMesh = packs[i][0];
                QString bstring = foldername + "/b" + QString::number(i);
                for (unsigned int j = 0; j < packs[i].size(); j++){
                    QString meshName = bstring + "p" + QString::number(j) + ".obj";
                    packs[i][j].saveOnObj(meshName.toStdString());
                    //Dcel d(packs[i][j]);
                    //Segmentation s(d);
                    //Dcel dd = s.getDcelFromSegmentation();
                    //dd.triangulate();
                    //dd.saveOnObjFile(meshName.toStdString());
                    if (j > 0){
                        packMesh = IGLInterface::IGLMesh::merge(packMesh, packs[i][j]);
                    }
                }
                packMesh.saveOnObj(pstring.toStdString());
            }
        }
    }
}

void EngineManager::on_reconstructionPushButton_clicked() {
    if (d != nullptr && he != nullptr){
        std::vector< std::pair<int,int> > mapping = Reconstruction::getMapping(*d, *he);
        Reconstruction::reconstruction(*d, mapping, originalMesh, *solutions);
        d->update();
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_putBoxesAfterPushButton_clicked() {
    int nTriangles = ui->coveredTrianglesSpinBox->value();
    if (nTriangles > 0 && solutions != nullptr){
        BoxList smallBoxes;
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            if (solutions->getBox(i).getTrianglesCovered() <= nTriangles){
                Box3D small = solutions->getBox(i);
                smallBoxes.addBox(small);
                solutions->removeBox(i);
                i--;
            }
        }
        for (unsigned int i = 0; i < smallBoxes.getNumberBoxes(); i++){
            solutions->addBox(smallBoxes.getBox(i));
        }
    }
}

void EngineManager::on_snappingPushButton_clicked() {
    double av = d->getAverageHalfEdgesLength();
    double epsilon = ui->snappingSpinBox->value();
    if (solutions != nullptr && d != nullptr){
        for (unsigned int i = 0; i < solutions->getNumberBoxes()-1; i++){
            Box3D b1 = solutions->getBox(i);
            for (unsigned int j = i+1; j < solutions->getNumberBoxes(); j++){
                Box3D b2 = solutions->getBox(j);
                for (unsigned int coord = 0; coord < 6; coord++) {
                    if (std::abs(b1[coord]-b2[coord]) < epsilon) {
                        b2[coord] = b1[coord];
                    }
                }
                b2.generatePiece(av*7);
                solutions->setBox(j, b2);
            }
        }
    }
    mainWindow->updateGlCanvas();
}

void EngineManager::on_colorPiecesPushButton_clicked() {
    if (he!=nullptr && d != nullptr){
        std::array<QColor, 10> colors;
        colors[0] = QColor(0,255,0);
        colors[1] = QColor(0,0,255);
        colors[2] = QColor(255,0,0);
        colors[3] = QColor(255,255,0);
        colors[4] = QColor(0,255,255);
        colors[5] = QColor(255,0,255);
        colors[6] = QColor(74,134,232);
        colors[7] = QColor(152,0,0);
        colors[8] = QColor(255,153,0);
        colors[9] = QColor(153,0,255);


        std::map< const Dcel::Vertex*, int > mapping = Reconstruction::getMappingId(*d, *he);
        std::vector< std::set<int> > adjacences(he->getNumHeightfields());
        for (const Dcel::Vertex* v : d->vertexIterator()){
            if (mapping.find(v) != mapping.end()){
                int hev = mapping[v];
                for (const Dcel::Vertex* adj : v->adjacentVertexIterator()){
                    if (mapping.find(adj) != mapping.end()){
                        int headj = mapping[adj];
                        if (hev != headj){
                            adjacences[hev].insert(headj);
                            adjacences[headj].insert(hev);
                        }
                    }
                }
            }
        }

        std::vector<bool> colored(he->getNumHeightfields(), false);
        std::vector<QColor> heColors(he->getNumHeightfields(), QColor(0,0,0));
        for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
            if (!colored[i]){
                std::set<QColor, cmpQColor> adjColors;
                for (int adj : adjacences[i]){
                    if (colored[adj])
                        adjColors.insert(heColors[adj]);
                }

                QColor color;
                bool finded = false;
                for (unsigned int k = 0; k < 10 && !finded; k++){
                    if (adjColors.find(colors[k]) == adjColors.end()){
                        finded = true;
                        color = colors[k];
                    }
                }
                assert(finded);
                heColors[i] = color;
                colored[i] = true;


                IGLInterface::IGLMesh mesh = he->getHeightfield(i);
                mesh.setFaceColor(color.redF(),color.greenF(),color.blueF());
                he->setHeightfield(mesh, i);
            }


        }
    }
    mainWindow->updateGlCanvas();
}

void EngineManager::on_deleteBoxesPushButton_clicked() {
    if (solutions != nullptr && d != nullptr){
        int n = Engine::deleteBoxes(*solutions, *d);
        std::cerr << "N deleted boxes: " << n << "\n";
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_volumePushButton_clicked() {
    if (baseComplex!=nullptr && d != nullptr){
        Eigen::RowVector3d Vmin, Vmax;
        Vmin = Eigen::RowVector3d(d->getBoundingBox().minX(), d->getBoundingBox().minY(), d->getBoundingBox().minZ());
        Vmax = Eigen::RowVector3d(d->getBoundingBox().maxX(), d->getBoundingBox().maxY(), d->getBoundingBox().maxZ());

        // create grid GV
        Eigen::RowVector3d border(2,2,2);
        Eigen::RowVector3d nGmin;
        Eigen::RowVector3d nGmax;
        Eigen::RowVector3i Gmini = (Vmin).cast<int>() - border.cast<int>();
        Eigen::RowVector3i Gmaxi = (Vmax).cast<int>() + border.cast<int>();
        nGmin = Gmini.cast<double>();
        nGmax = Gmaxi.cast<double>();
        Eigen::RowVector3i res = (nGmax.cast<int>() - nGmin.cast<int>())/2; res(0)+=1; res(1)+=1; res(2)+=1;

        double gridUnit = 2;
        res(0) /= ui->factorSpinBox->value();
        res(1) /= ui->factorSpinBox->value();
        res(2) /= ui->factorSpinBox->value();
        gridUnit*=ui->factorSpinBox->value();


        CGALInterface::AABBTree td(*d);
        CGALInterface::AABBTree the(*baseComplex);
        int counterd = 0, counterhe = 0;
        int xi = nGmin(0), yi = nGmin(1), zi = nGmin(2);
        for (int i = 0; i < res(0); ++i){
            yi = nGmin(1);
            for (int j = 0; j < res(1); ++j){
                zi = nGmin(2);
                for (int k = 0; k < res(2); ++k){
                    Pointd p(xi,yi,zi);
                    if (td.isInside(p)){
                        counterd++;
                        if (the.isInside(p))
                            counterhe++;
                    }
                    zi+=gridUnit;
                }
                yi+=gridUnit;
            }
            xi +=gridUnit;
        }

        std::cerr << "Counter d: " << counterd << "; Counter he: " << counterhe << "\n";
        double percent = ((double)counterhe / counterd) * 100;
        percent = 100 - percent;
        updateLabel(percent, ui->volumePercentLabel);
        ui->volumePercentLabel->setText(ui->volumePercentLabel->text() + "%");
    }
    /*
    if (he!=nullptr && d != nullptr){

        IGLInterface::SimpleIGLMesh heMesh;
        for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
            heMesh = IGLInterface::SimpleIGLMesh::merge(heMesh, he->getHeightfield(i));
        }
        Array3D<Pointd> gridd;
        Array3D<float> dfd, dfhe;
        double gridUnit = ui->factorSpinBox->value() * 2;
        IGLInterface::generateGridAndDistanceField(gridd, dfd, IGLInterface::SimpleIGLMesh(*d), gridUnit);
        IGLInterface::generateGridAndDistanceField(gridd, dfhe, heMesh, gridUnit);

        int counterd = 0, counterhe = 0;
        for (int i = 0; i < dfd.getSizeX(); ++i){
            for (int j = 0; j < dfd.getSizeY(); ++j){
                for (int k = 0; k < dfd.getSizeZ(); ++k){
                    if (dfd(i,j,k) < 0){
                        counterd++;
                        if (dfhe(i,j,k))
                            counterhe++;
                    }
                }
            }
        }
        std::cerr << "Counter d: " << counterd << "; Counter he: " << counterhe << "\n";
        double percent = ((double)counterhe / counterd) * 100;
        updateLabel(percent, ui->volumePercentLabel);
        ui->volumePercentLabel->setText(ui->volumePercentLabel->text() + "%");
    }
    */
}

void EngineManager::on_pushButton_clicked() {
    if (g == nullptr && d!= nullptr) {
        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
        mainWindow->updateGlCanvas();
    }
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        mainWindow->pushObj(solutions, "Solutions");
        double kernelDistance = ui->distanceSpinBox->value();
        Timer t("Total Time Grids and Minimization Boxes");
        /// here d is already scaled!!
        if (ui->limitsConstraintCheckBox->isChecked()){
            Engine::optimize(*solutions, *d, kernelDistance, true, getLimits(), ui->heightfieldsCheckBox->isChecked(), ui->onlyNearestTargetCheckBox->isChecked(), ui->areaToleranceSpinBox->value(), (double)ui->toleranceSlider->value()/100, ui->useFileCheckBox->isChecked());
        }
        else {
            Engine::optimize(*solutions, *d, kernelDistance, false, Pointd(), ui->heightfieldsCheckBox->isChecked(), ui->onlyNearestTargetCheckBox->isChecked(), ui->areaToleranceSpinBox->value(), (double)ui->toleranceSlider->value()/100, ui->useFileCheckBox->isChecked());
        }
        t.stopAndPrint();
        ui->showAllSolutionsCheckBox->setEnabled(true);
        solutions->setVisibleBox(0);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_limitsConstraintCheckBox_stateChanged(int arg1) {
    if (arg1 ==Qt::Checked){
        ui->xLimitSpinBox->setEnabled(true);
        ui->xLimitSpinBox->setEnabled(true);
        ui->zLimitSpinBox->setEnabled(true);
        ui->minEdgeBBSpinBox->setEnabled(true);
    }
    else {
        ui->xLimitSpinBox->setEnabled(false);
        ui->xLimitSpinBox->setEnabled(false);
        ui->zLimitSpinBox->setEnabled(false);
        ui->minEdgeBBSpinBox->setEnabled(false);
    }
}
