#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>
#include <QFileDialog>
#include <omp.h>
#include "lib/cgal_interface/aabbtree.h"

EngineManager::EngineManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::EngineManager),
    mainWindow((MainWindow*)parent),
    g(nullptr),
    d(nullptr),
    b(nullptr),
    iterations(nullptr),
    solutions(nullptr){
    ui->setupUi(this);
    ui->iterationsSlider->setMaximum(0);
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

void EngineManager::updateLabel(double value, QLabel* label) {
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::digits10+1);
    ss << value;
    label->setText(QString::fromStdString(ss.str()));
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

    ////DEBUG
    double min, max;
    g->getMinAndMax(min, max);
    std::cerr << "Min: " << min <<"; max: " << max << "\n";
    //FINEDEBUG
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

        Engine::scaleAndRotateDcel(*d,  ui->samplesSpinBox->value(), 0);
        Engine::generateGrid(*g, *d, ui->distanceSpinBox->value(), ui->heightfieldsCheckBox->isChecked(), XYZ[ui->targetComboBox->currentIndex()]);

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
        g->calculateBorderWeights(*d);
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
        g->calculateWeightsAndFreezeKernel(*d, value, ui->heightfieldsCheckBox->isChecked());
        e = Energy(*g);
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

void EngineManager::on_wSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setW(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_hSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setH(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_dSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setD(arg1);
        mainWindow->updateGlCanvas();
    }
}

void EngineManager::on_plusXButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked())
            b->moveX(ui->stepSpinBox->value());
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
        if (ui->boxRadioButton->isChecked())
            b->moveX(- ui->stepSpinBox->value());
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
        if (ui->boxRadioButton->isChecked())
            b->moveY(ui->stepSpinBox->value());
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
        if (ui->boxRadioButton->isChecked())
            b->moveY(- ui->stepSpinBox->value());
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
        if (ui->boxRadioButton->isChecked())
            b->moveZ(ui->stepSpinBox->value());
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
        if (ui->boxRadioButton->isChecked())
            b->moveZ(- ui->stepSpinBox->value());
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
        e.gradientTricubicInterpolationEnergy(gradient, b->getMin(), b->getMax());
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
            Timer t("Gradient Discend");
            it = e.gradientDiscend(*b, *iterations);
            t.stopAndPrint();
            iterations->setVisibleBox(0);
            ui->iterationsSlider->setMaximum(iterations->getNumberBoxes()-1);
            mainWindow->pushObj(iterations, "Iterations");

            double energy = e.energy(iterations->getBox(0));
            updateLabel(energy, ui->energyIterationLabel);
        }
        else {
            Timer t("Gradient Discend");
            it = e.gradientDiscend(*b);
            t.stopAndPrint();
        }
        updateLabel(it, ui->minimizedEnergyLabel);
        double energy = e.energy(*b);
        updateLabel(energy, ui->energyLabel);
        ui->wSpinBox->setValue(b->getMax().x()-b->getMin().x());
        ui->hSpinBox->setValue(b->getMax().y()-b->getMin().y());
        ui->dSpinBox->setValue(b->getMax().z()-b->getMin().z());
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
        int rot = ui->orientationComboBox->currentIndex();
        if (rot > 0){
            switch (rot){
                case 1:
                    getRotationMatrix(Vec3(0,0,1), 0.785398, m);
                    copy.rotate(m);
                    break;
                case 2:
                    getRotationMatrix(Vec3(1,0,0), 0.785398, m);
                    copy.rotate(m);
                    break;
                case 3:
                    getRotationMatrix(Vec3(0,1,0), 0.785398, m);
                    copy.rotate(m);
                    break;
                default:
                    assert(0);
            }
        }
        /**
         * @todo is transpose right?
         */
        if (rot > 0){
            switch (rot){
                case 1:
                    getRotationMatrix(Vec3(0,0,-1), 0.785398, m);
                    break;
                case 2:
                    getRotationMatrix(Vec3(-1,0,0), 0.785398, m);
                    break;
                case 3:
                    getRotationMatrix(Vec3(0,-1,0), 0.785398, m);
                    break;
                default:
                    assert(0);
            }
        }


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
    mainWindow->updateGlCanvas();
}

void EngineManager::on_solutionsSlider_valueChanged(int value) {
    if (ui->solutionsSlider->isEnabled()){
        solutions->setVisibleBox(value);
        ui->setFromSolutionSpinBox->setValue(value);
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
            deleteDrawableObject(b);
            solutions->getBox(value);
            b = new Box3D(solutions->getBox(value));
            mainWindow->pushObj(b, "Box");
            mainWindow->updateGlCanvas();
        }

    }
}

void EngineManager::on_wireframeDcelCheckBox_stateChanged(int arg1) {
    if (d!=nullptr){
        d->setWireframe(arg1 == Qt::Checked);
        mainWindow->updateGlCanvas();
    }

}

void EngineManager::on_pointsDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr){
        if (checked){
            d->setPointsShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_flatDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr){
        if (checked){
            d->setFlatShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_smoothDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr){
        if (checked){
            d->setSmoothShading();
            mainWindow->updateGlCanvas();
        }
    }
}

void EngineManager::on_trianglesCoveredPushButton_clicked() {
    if (d!=nullptr && b != nullptr) {
        Eigen::Matrix3d m[ORIENTATIONS];
        Eigen::Matrix3d mb = b->getRotationMatrix();
        m[0] = Eigen::Matrix3d::Identity();
        getRotationMatrix(Vec3(0,0,-1), 0.785398, m[1]);
        getRotationMatrix(Vec3(-1,0,0), 0.785398, m[2]);
        getRotationMatrix(Vec3(0,-1,0), 0.785398, m[3]);
        Dcel dd = *d;
        std::list<const Dcel::Face*> covered;
        if (mb == m[0]){
            CGALInterface::AABBTree t(dd);
            t.getIntersectedPrimitives(covered, *b);
        }
        else if (mb == m[1]){

            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(0,0,1), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedPrimitives(covered, *b);
        }
        else if (mb == m[2]){
            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(1,0,0), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedPrimitives(covered, *b);
        }
        else if (mb == m[3]){
            Eigen::Matrix3d mm;
            getRotationMatrix(Vec3(0,1,0), 0.785398, mm);
            dd.rotate(mm);
            CGALInterface::AABBTree t(dd);
            t.getIntersectedPrimitives(covered, *b);
        }
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

void EngineManager::on_deserializePreprocessingPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Deserialize",
                       ".",
                       "BIN(*.bin)");

    if (!filename.isEmpty()) {
        deleteDrawableObject(d);
        d = new DrawableDcel();

        std::ifstream myfile;
        myfile.open (filename.toStdString(), std::ios::in | std::ios::binary);
        bool heightfields;

        d->deserialize(myfile);
        for (Dcel::FaceIterator fit = d->faceBegin(); fit != d->faceEnd(); ++fit)
            (*fit)->setColor(QColor(128,128,128));
        d->setWireframe(true);
        d->setPointsShading();
        d->update();
        mainWindow->pushObj(d, "Scaled Mesh");

        Serializer::deserialize(heightfields, myfile);
        //heightfields = true;
        if (!heightfields){
            Grid g[ORIENTATIONS];
            BoxList bl[ORIENTATIONS];
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                g[i].deserialize(myfile);
                bl[i].deserialize(myfile);
            }
            myfile.close();

            deleteDrawableObject(this->g);
            this->g = new DrawableGrid();
            *(this->g) = g[0];
            mainWindow->pushObj(this->g, "Grid");

            deleteDrawableObject(solutions);
            solutions = new BoxList();

            //caricamento solutions
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                solutions->insert(bl[i]);
            }

            solutions->setVisibleBox(0);
            solutions->setCylinders(false);
            mainWindow->pushObj(solutions, "Solutions");
            ui->showAllSolutionsCheckBox->setEnabled(true);
            ui->solutionsSlider->setEnabled(true);
            ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
            ui->setFromSolutionSpinBox->setValue(0);
            ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);

        }
        else {
            Grid g[ORIENTATIONS][TARGETS];
            BoxList bl[ORIENTATIONS][TARGETS];
            for (unsigned int i = 0; i < ORIENTATIONS; ++i){
                for (unsigned j = 0; j < TARGETS; ++j){
                    g[i][j].deserialize(myfile);
                    bl[i][j].deserialize(myfile);
                }
            }
            myfile.close();

            deleteDrawableObject(this->g);
            this->g = new DrawableGrid();
            *(this->g) = g[0][0];
            mainWindow->pushObj(this->g, "Grid");

            deleteDrawableObject(solutions);
            solutions = new BoxList();

            //caricamento solutions
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                for (unsigned int j = 0; j < TARGETS; ++j){
                    solutions->insert(bl[i][j]);
                }
            }

            solutions->setVisibleBox(0);
            solutions->setCylinders(false);
            mainWindow->pushObj(solutions, "Solutions");
            ui->showAllSolutionsCheckBox->setEnabled(true);
            ui->solutionsSlider->setEnabled(true);
            ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
            ui->setFromSolutionSpinBox->setValue(0);
            ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        }
        e = Energy(*g);
    }
    mainWindow->updateGlCanvas();
}

void EngineManager::on_serializePreprocessingPushButton_clicked() {
    QString filename = QFileDialog::getSaveFileName(nullptr,
                       "Serialize",
                       ".",
                       "BIN(*.bin)");
    if (!filename.isEmpty()) {
        Dcel scaled[ORIENTATIONS];
        Eigen::Matrix3d m[ORIENTATIONS];
        bool heightfields = ui->heightfieldsCheckBox->isChecked();
        scaled[0] = (Dcel)*d;
        m[0] = Eigen::Matrix3d::Identity();
        for (unsigned int i = 1; i < ORIENTATIONS; i++){
            scaled[i] = (Dcel)*d;
            m[i] = Engine::rotateDcelAlreadyScaled(scaled[i], i);
        }
        if (! heightfields){
            Grid g[ORIENTATIONS];
            BoxList bl[ORIENTATIONS];
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                Engine::generateGrid(g[i], scaled[i], ui->distanceSpinBox->value(), false);
                Engine::calculateInitialBoxes(bl[i],scaled[i], m[i], false);

            }
            std::ofstream myfile;
            myfile.open (filename.toStdString(), std::ios::out | std::ios::binary);
            scaled[0].serialize(myfile);
            Serializer::serialize(heightfields, myfile);
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                g[i].serialize(myfile);
                bl[i].serialize(myfile);
            }

            myfile.close();
        }
        else {
            Grid g[ORIENTATIONS][TARGETS];
            BoxList bl[ORIENTATIONS][TARGETS];
            for (unsigned int i = 0; i < ORIENTATIONS; ++i){
                for (unsigned j = 0; j < TARGETS; ++j){
                    Engine::generateGrid(g[i][j], scaled[i], ui->distanceSpinBox->value(), true, XYZ[j]);
                    Engine::calculateInitialBoxes(bl[i][j],scaled[i], m[i], true, XYZ[j]);
                }
            }
            std::ofstream myfile;
            myfile.open (filename.toStdString(), std::ios::out | std::ios::binary);
            scaled[0].serialize(myfile);
            Serializer::serialize(heightfields, myfile);
            for (unsigned int i = 0; i < ORIENTATIONS; i++){
                for (unsigned j = 0; j < TARGETS; ++j){
                    g[i][j].serialize(myfile);
                    bl[i][j].serialize(myfile);
                }
            }
            myfile.close();
        }
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
