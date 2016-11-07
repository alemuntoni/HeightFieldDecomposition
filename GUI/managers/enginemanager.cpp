#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>
#include <QFileDialog>
#include <omp.h>
#include "cgal/aabbtree.h"

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
    he(nullptr),
    entirePieces(nullptr),
    recBoxes(nullptr),
    irregularGrid(nullptr),
    newPieces(nullptr),
    newBaseComplex(nullptr){
    ui->setupUi(this);
    ui->iterationsSlider->setMaximum(0);

    //ui->frame_2->setVisible(false);
    //ui->frame_5->setVisible(false);
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
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    deleteDrawableObject(b);
    deleteDrawableObject(iterations);
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
    deleteDrawableObject(entirePieces);
    deleteDrawableObject(recBoxes);
    deleteDrawableObject(newPieces);
    deleteDrawableObject(newBaseComplex);
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

void EngineManager::serialize(std::ofstream& binaryFile) const {
    g->serialize(binaryFile);
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
}

void EngineManager::deserialize(std::ifstream& binaryFile) {
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    g = new DrawableGrid();
    d = new DrawableDcel();
    g->deserialize(binaryFile);
    d->deserialize(binaryFile);
    bool bb = false;
    for (Dcel::FaceIterator fit = d->faceBegin(); fit != d->faceEnd(); ++fit)
        (*fit)->setColor(QColor(128,128,128));
    d->setWireframe(true);
    d->setPointsShading();
    updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
    d->update();
    mainWindow->pushObj(d, "Scaled Mesh");
    mainWindow->pushObj(g, "Grid");
    e = Energy(*g);
    ui->weigthsRadioButton->setChecked(true);
    ui->sliceCheckBox->setChecked(true);
    g->setDrawBorders();
    g->setSlice(1);


    Serializer::deserialize(bb, binaryFile);
    if (bb){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        solutions->deserialize(binaryFile);
        solutions->setVisibleBox(0);
        solutions->setCylinders(false);
        mainWindow->pushObj(solutions, "Solutions");
        ui->showAllSolutionsCheckBox->setEnabled(true);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
    }

    mainWindow->updateGlCanvas();
}

void EngineManager::on_generateGridPushButton_clicked() {
    DcelManager* dm = (DcelManager*)mainWindow->getManager(DCEL_MANAGER_ID);
    DrawableDcel* dd = dm->getDcel();
    if (dd != nullptr){
        deleteDrawableObject(d);
        d = new DrawableDcel(*dd);
        mainWindow->pushObj(d, "Scaled Mesh");
        deleteDrawableObject(g);
        g = new DrawableGrid();

        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        Engine::generateGrid(*g, *d, ui->distanceSpinBox->value(), ui->heightfieldsCheckBox->isChecked(), XYZ[ui->targetComboBox->currentIndex()], savedFaces);
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
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
    if (d != nullptr && baseComplex != nullptr && he != nullptr && entirePieces != nullptr) {
        QString foldername = QFileDialog::getExistingDirectory(nullptr, "SaveObjs");
        if (!foldername.isEmpty()){
            QString inputMeshString = foldername + "/InputMesh.obj";
            QString baseComplexString = foldername + "/BaseComplex.obj";
            QString heightfieldString = foldername + "/Heightfield";
            QString entirePieceString = foldername + "/EntirePiece";
            d->saveOnObjFile(inputMeshString.toStdString());
            baseComplex->saveOnObj(baseComplexString.toStdString());
            for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
                IGLInterface::IGLMesh h = he->getHeightfield(i);
                IGLInterface::IGLMesh ep = entirePieces->getHeightfield(i);
                std::stringstream ss;
                std::stringstream ss1;
                ss << heightfieldString.toStdString() << i << ".obj";
                ss1 << entirePieceString.toStdString() << i << ".obj";
                h.saveOnObj(ss.str());
                ep.saveOnObj(ss1.str());
            }
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

void EngineManager::on_createBoxesPushButton_clicked() {
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        //Engine::calculateInitialBoxes(*solutions, *d, Eigen::Matrix3d::Identity(), true, XYZ[ui->targetComboBox->currentIndex()]);
        Dcel copy = *d;
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();

        Engine::calculateInitialBoxes(*solutions, copy, m);
        ui->showAllSolutionsCheckBox->setEnabled(true);
        solutions->setVisibleBox(0);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow->pushObj(solutions, "Solutions");
        mainWindow->updateGlCanvas();
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

void EngineManager::on_minimizeAllPushButton_clicked() {
    if (solutions != nullptr){
        Engine::expandBoxes(*solutions, *g);
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
            baseComplex->setPointShading();
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

void EngineManager::on_deleteBoxesPushButton_clicked() {
    if (solutions!= nullptr && d != nullptr){
        Engine::deleteBoxesMemorySafe(*solutions, *d);
        solutions->setVisibleBox(0);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
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

void EngineManager::on_baseComplexPushButton_clicked() {
    if (d != nullptr){
        deleteDrawableObject(baseComplex);
        IGLInterface::IGLMesh m = (Dcel)*d;
        baseComplex = new IGLInterface::DrawableIGLMesh(m);
        mainWindow->pushObj(baseComplex, "Base Complex");
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_subtractPushButton_clicked() {
    if (solutions!= nullptr && baseComplex != nullptr && d != nullptr){
        deleteDrawableObject(he);
        deleteDrawableObject(entirePieces);
        he = new HeightfieldsList();
        entirePieces = new HeightfieldsList();
        mainWindow->pushObj(he, "Heightfields");
        mainWindow->pushObj(entirePieces, "Entire Pieces");
        mainWindow->updateGlCanvas();
        IGLInterface::SimpleIGLMesh bc((IGLInterface::SimpleIGLMesh)*baseComplex);
        Engine::booleanOperations(*he, bc, *solutions, *d, *entirePieces);
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
            baseComplex->setColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow->pushObj(baseComplex, "Base Complex");
        mainWindow->updateGlCanvas();
        baseComplex->saveOnObj("BaseComplex.obj");
    }
}

void EngineManager::on_stickPushButton_clicked() {
    if (d!=nullptr && baseComplex != nullptr && he != nullptr && solutions != nullptr){
        IGLInterface::SimpleIGLMesh bc = *baseComplex;
        Engine::gluePortionsToBaseComplex(*he, bc, *solutions, *d);
        deleteDrawableObject(baseComplex);
        baseComplex = new IGLInterface::DrawableIGLMesh(bc);
        baseComplex->updateFaceNormals();
        for (unsigned int i = 0; i < baseComplex->getNumberFaces(); ++i){
            Vec3 n = baseComplex->getNormal(i);
            n.normalize();
            QColor c = colorOfNearestNormal(n);
            baseComplex->setColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow->pushObj(baseComplex, "Base Complex");
    }

    mainWindow->updateGlCanvas();
}

void EngineManager::on_serializeBCPushButton_clicked() {
    if (baseComplex != nullptr && solutions != nullptr && d != nullptr && he != nullptr && entirePieces != nullptr){
        QString filename = QFileDialog::getSaveFileName(nullptr,
                           "Serialize",
                           ".",
                           "BIN(*.bin)");
        if (!filename.isEmpty()) {
            std::ofstream myfile;
            myfile.open (filename.toStdString(), std::ios::out | std::ios::binary);
            d->serialize(myfile);
            solutions->serialize(myfile);
            baseComplex->serialize(myfile);
            he->serialize(myfile);
            entirePieces->serialize(myfile);
            myfile.close();
        }
    }

}

void EngineManager::on_deserializeBCPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Deserialize",
                       ".",
                       "BIN(*.bin)");

    if (!filename.isEmpty()) {
        deleteDrawableObject(d);
        deleteDrawableObject(solutions);
        deleteDrawableObject(baseComplex);
        deleteDrawableObject(he);
        deleteDrawableObject(entirePieces);
        d = new DrawableDcel();
        solutions = new BoxList();
        baseComplex = new IGLInterface::DrawableIGLMesh();
        he = new HeightfieldsList();
        entirePieces = new HeightfieldsList();
        std::ifstream myfile;
        myfile.open (filename.toStdString(), std::ios::in | std::ios::binary);
        d->deserialize(myfile);
        solutions->deserialize(myfile);
        baseComplex->deserialize(myfile);
        he->deserialize(myfile);
        entirePieces->deserialize(myfile);
        myfile.close();
        d->update();
        d->setPointsShading();
        d->setWireframe(true);
        ///
        //Box3D bb = solutions->getBox(5);
        //solutions->removeBox(5);
        //solutions->addBox(bb,0);
        ///
        mainWindow->pushObj(d, "Input Mesh");
        mainWindow->pushObj(solutions, "Boxes");
        mainWindow->pushObj(baseComplex, "Base Complex");
        mainWindow->pushObj(he, "Heightfields");
        mainWindow->pushObj(entirePieces, "Entire Pieces");
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
        //Engine::gluePortionsToBaseComplex(*he, *baseComplex, *solutions, *d);
        IGLInterface::IGLMesh m(*d);
        m.saveOnObj("results/input_model.obj");
        baseComplex->saveOnObj("results/base_complex.obj");
        for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
            IGLInterface::IGLMesh h;
            h = he->getHeightfield(i);
            std::stringstream ss;
            ss << "results/heightfield" << i << ".obj";
            h.saveOnObj(ss.str());
        }
    }
}

void EngineManager::on_createAndMinimizeAllPushButton_clicked() {
    if (d == nullptr) {
        DcelManager* dm = (DcelManager*)mainWindow->getManager(DCEL_MANAGER_ID);
        DrawableDcel* dd = dm->getDcel();
        if (dd != nullptr){
            d = new DrawableDcel(*dd);
            mainWindow->pushObj(d, "Scaled Mesh");

            Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
            std::set<const Dcel::Face*> flippedFaces, savedFaces;
            Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
            updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
            d->update();
            mainWindow->updateGlCanvas();
        }
    }
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        //deleteDrawableObject(g);
        solutions = new BoxList();
        mainWindow->pushObj(solutions, "Solutions");
        double kernelDistance = ui->distanceSpinBox->value();
        Timer t("Total Time Grids and Minimization Boxes");
        Engine::createAndMinimizeAllBoxes(*solutions, *d, kernelDistance, ui->heightfieldsCheckBox->isChecked(), ui->onlyNearestTargetCheckBox->isChecked(), ui->areaToleranceSpinBox->value(), (double)ui->toleranceSlider->value()/100);
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
            if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(-1);
            mainWindow->updateGlCanvas();
        }
    }
    else {
        if (he != nullptr){
            if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(ui->heightfieldsSlider->value());
            he->setVisibleHeightfield(ui->heightfieldsSlider->value());
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_heightfieldsSlider_valueChanged(int value) {
    if (he != nullptr){
        ui->solutionsSlider->setValue(value);
        if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(value);
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
    deleteDrawableObject(entirePieces);
}

void EngineManager::on_createIrregularGridButton_clicked() {
    if (solutions != nullptr && d != nullptr) {
        irregularGrid = new DrawableIrregularGrid();
        Reconstruction::createIrregularGrid(*irregularGrid, *solutions, *d);
        mainWindow->pushObj(irregularGrid, "Irregular Grid", false);
        mainWindow->updateGlCanvas();

        int count = 0, othercount=0;
        for (unsigned int i = 0; i < irregularGrid->getResolutionX()-1; i++){
            for (unsigned int j = 0; j < irregularGrid->getResolutionY()-1; j++){
                for (unsigned int k = 0; k < irregularGrid->getResolutionZ()-1; k++){
                    if (irregularGrid->getNumberPossibleTargets(i,j,k) == 1 || irregularGrid->getNumberPossibleTargets(i,j,k) == 0)
                        count++;
                    else
                        othercount++;
                }
            }
        }
        std::cerr << count << "\n";
        std::cerr << othercount << "\n";
    }
}

void EngineManager::on_createPieces_clicked() {
    if (irregularGrid != nullptr) {
        std::vector<Vec3> targets;
        std::vector<IGLInterface::IGLMesh> pieces = Reconstruction::getPieces(*irregularGrid, targets);
        deleteDrawableObject(recBoxes);
        recBoxes = new HeightfieldsList();
        for (unsigned int i = 0; i < pieces.size(); i++){
            recBoxes->addHeightfield(IGLInterface::DrawableIGLMesh(pieces[i]), targets[i]);
        }
        recBoxes->setVisibleHeightfield(0);
        std::cerr << recBoxes->getNumHeightfields() << "\n";
        mainWindow->pushObj(recBoxes, "Boxes");
        mainWindow->updateGlCanvas();
        ui->recBoxesSlider->setMaximum(recBoxes->getNumHeightfields()-1);
        ui->recBoxesSlider->setValue(0);
    }
}

void EngineManager::on_recBoxesSlider_valueChanged(int value) {
    if (recBoxes != nullptr) {
        recBoxes->setVisibleHeightfield(value);
        if (newPieces != nullptr)
            newPieces->setVisibleHeightfield(value);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_intersectionsPushButton_clicked() {
    if (recBoxes != nullptr && d != nullptr) {
        deleteDrawableObject(newBaseComplex);
        IGLInterface::IGLMesh m = (Dcel)*d;
        newBaseComplex = new IGLInterface::DrawableIGLMesh(m);
        mainWindow->pushObj(newBaseComplex, "New Base Complex");

        newPieces = new HeightfieldsList();

        Reconstruction::booleanOperations(*newPieces, *newBaseComplex, *recBoxes);

        mainWindow->pushObj(newPieces, "New Heightfields");

        recBoxes->setVisibleHeightfield(0);
        newPieces->setVisibleHeightfield(0);
        ui->recBoxesSlider->setMaximum(recBoxes->getNumHeightfields()-1);
        ui->recBoxesSlider->setValue(0);

        mainWindow->updateGlCanvas();
    }
}
