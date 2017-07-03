#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>
#include <QFileDialog>
#include <QMessageBox>
#include <omp.h>
#include "cgal/aabbtree.h"
#include "engine/packing.h"
#include "engine/reconstruction.h"
#include <QThread>
#include "lib/dcel_segmentation/segmentation.h"
#include <eigenmesh/algorithms/eigenmesh_algorithms.h>
#include "engine/orientation.h"
#include <lib/graph/bipartitegraph.h>
#include <engine/greedysubdivision.h>

EngineManager::EngineManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::EngineManager),
    mainWindow((MainWindow&)*parent),
    g(nullptr),
    d(nullptr),
    b(nullptr),
    b2(nullptr),
    iterations(nullptr),
    solutions(nullptr),
    baseComplex(nullptr),
    he(nullptr),
    alreadySplitted(false){
    ui->setupUi(this);
    ui->iterationsSlider->setMaximum(0);

    //ui->frame_2->setVisible(false);
    //ui->frame_5->setVisible(false);
}

void EngineManager::deleteDrawableObject(DrawableObject* d) {
    if (d != nullptr) {
        //d->setVisible(false);
        mainWindow.deleteObj(d);
        delete d;
        d = nullptr;
    }
}

EngineManager::~EngineManager() {
    delete ui;
    deleteDrawableObject(g);
    deleteDrawableObject(d);
    deleteDrawableObject(b);
    deleteDrawableObject(b2);
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
    d->setColor(Color(128,128,128));
    std::set<const Dcel::Face*> flippedFaces, savedFaces;
    Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], angleThreshold/100, areaThreshold);

    for (const Dcel::Face* cf : flippedFaces){
        Dcel::Face* f = d->getFace(cf->getId());
        f->setColor(Color(255,0,0));
    }
    for (const Dcel::Face* cf : savedFaces){
        Dcel::Face* f = d->getFace(cf->getId());
        f->setColor(Color(0,0,255));
    }

    d->update();
    mainWindow.updateGlCanvas();
}

Pointd EngineManager::getLimits() {
    assert(d!=nullptr);
    BoundingBox bb = d->getBoundingBox();
    double lx = bb.getLengthX();
    double ly = bb.getLengthY();
    double lz = bb.getLengthZ();
    double min = lx;
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

void EngineManager::saveMSCFile(const std::string& filename, const Dcel& d, const BoxList& bl) {
    Array2D<int> mat(bl.getNumberBoxes(), d.getNumberFaces(), 0);
    CGALInterface::AABBTree tree(d);
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::list<const Dcel::Face*> list = tree.getCompletelyContainedDcelFaces(bl.getBox(i));
        for (const Dcel::Face* f : list){
            mat(i,f->getId()) = 1;
        }
    }
    std::ofstream of;
    of.open(filename);
    of << mat.getSizeX() << " " << mat.getSizeY() << "\n";
    for (unsigned int i = 0; i < mat.getSizeX(); i++){
        for (unsigned int j =0; j < mat.getSizeY(); j++){
            of << mat(i,j) << " ";
        }
        of << "x\n";
    }
    of.close();
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
    bool b = true;
    Serializer::serialize(b, myfile);
    originalSolutions.serialize(myfile);
    Serializer::serialize(splittedBoxesToOriginals, myfile);
    Serializer::serialize(priorityBoxes, myfile);
    for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++)
        Serializer::serialize(solutions->getBox(i).getId(), myfile);
    myfile.close();
}

void EngineManager::deserializeBC(const std::string& filename) {
    deleteDrawableObject(d);
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
    d = new DrawableDcel();
    solutions = new BoxList();
    baseComplex = new DrawableEigenMesh();
    he = new HeightfieldsList();
    std::ifstream myfile;
    myfile.open (filename, std::ios::in | std::ios::binary);
    d->deserialize(myfile);
    solutions->deserialize(myfile);
    baseComplex->deserialize(myfile);
    he->deserialize(myfile);
    he->checkHeightfields();
    if (originalMesh.deserialize(myfile)){
        if (originalMesh.getNumberVertices() > 0)
            mainWindow.pushObj(&originalMesh, "Original Mesh");
    }
    double factor, kernel;
    Serializer::deserialize(factor, myfile);
    Serializer::deserialize(kernel, myfile);
    ui->factorSpinBox->setValue(factor);
    ui->distanceSpinBox->setValue(kernel);
    bool b;
    if (Serializer::deserialize(b, myfile) && b == true){
        originalSolutions.deserialize(myfile);
        Serializer::deserialize(splittedBoxesToOriginals, myfile);
        Serializer::deserialize(priorityBoxes, myfile);
        alreadySplitted = true;
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            int bid;
            Serializer::deserialize(bid, myfile);
            (*solutions)[i].setId(bid);
        }
    }
    /*d->saveOnObjFile("boxes/mesh.obj");
    for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
        solutions->getBox(i).saveOnObjFile("boxes/box" + std::to_string(i) + ".obj", solutions->getBox(i).getColor());
    }*/
    //BU
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(0,0,1), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    originalMesh.rotate(m);
    baseComplex->rotate(m);

    m = Common::getRotationMatrix(Vec3(0,1,0), M_PI);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/
    //

    //Statue
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    originalMesh.rotate(m);
    baseComplex->rotate(m);

    m = Common::getRotationMatrix(Vec3(0,1,0), M_PI);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/
    //

    //Airplane
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    originalMesh.rotate(m);
    baseComplex->rotate(m);

    m = Common::getRotationMatrix(Vec3(0,1,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/
    //

    //Batman
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/
    //

    //Fertility
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/

    //Bunny
    /*Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
    he->rotate(m);
    d->rotate(m);
    d->updateFaceNormals();
    d->updateVertexNormals();
    d->updateBoundingBox();
    originalMesh.rotate(m);
    originalMesh.updateVertexAndFaceNormals();
    baseComplex->rotate(m);*/
    //
    myfile.close();
    d->update();
    d->setPointsShading();
    d->setWireframe(true);
    mainWindow.pushObj(d, "Input Mesh");
    mainWindow.pushObj(solutions, "Boxes");
    mainWindow.pushObj(baseComplex, "Base Complex");
    mainWindow.pushObj(he, "Heightfields");
    mainWindow.updateGlCanvas();
    ui->showAllSolutionsCheckBox->setEnabled(true);
    //he->explode(150);
    solutions->setVisibleBox(0);
    ui->heightfieldsSlider->setMaximum(he->getNumHeightfields()-1);
    ui->allHeightfieldsCheckBox->setChecked(true);
    ui->solutionsSlider->setEnabled(true);
    ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
    ui->setFromSolutionSpinBox->setValue(0);
    ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);

    for (unsigned int i : priorityBoxes){
        std::string s = ui->listLabel->text().toStdString();
        ui->listLabel->setText(QString::fromStdString(s + std::to_string(i) + "; "));
    }
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
        (*fit)->setColor(Color(128,128,128));
    d->setWireframe(true);
    d->setPointsShading();
    //updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
    d->update();
    mainWindow.pushObj(d, "Scaled Mesh");
    //mainWindow.pushObj(g, "Grid");
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
            mainWindow.pushObj(solutions, "Solutions");
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
        mainWindow.pushObj(&originalMesh, "Original Mesh");
    }
    double kernel, factor;
    if (Serializer::deserialize(factor, binaryFile) && Serializer::deserialize(kernel, binaryFile)){
        ui->factorSpinBox->setValue(factor);
        ui->distanceSpinBox->setValue(kernel);
    }

    mainWindow.updateGlCanvas();
    //saveMSCFile("prova.txt", *d, *solutions);
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
        mainWindow.pushObj(g, "Grid");
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_distanceSpinBox_valueChanged(double arg1) {
    if (g!=nullptr){
        g->setKernelDistance(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_targetComboBox_currentIndexChanged(int index) {
    if (d!= nullptr && g!= nullptr){
        g->setTarget(XYZ[index]);
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        //std::set<const Dcel::Face*> flippedFaces, savedFaces;
        //Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        //g->calculateBorderWeights(*d, ui->heightfieldsCheckBox->isChecked(), savedFaces);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_kernelRadioButton_toggled(bool checked) {
    if (checked && g!=nullptr){
        g->setDrawKernel();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_weigthsRadioButton_toggled(bool checked) {
    if (checked && g!=nullptr){
        g->setDrawBorders();
        mainWindow.updateGlCanvas();
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
        mainWindow.updateGlCanvas();
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_sliceSlider_valueChanged(int value) {
    if (g!=nullptr){
        g->setSliceValue(value);
        mainWindow.updateGlCanvas();
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
        mainWindow.updateGlCanvas();
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
            d->updateVertexNormals();
            Engine::saveObjs(foldername, originalMesh, *d, *baseComplex, *he);
        }
    }
}

void EngineManager::on_wSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setW(arg1);
        updateBoxValues();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_hSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setH(arg1);
        updateBoxValues();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_dSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setD(arg1);
        updateBoxValues();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_plusXButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveX(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveX(ui->stepSpinBox->value());
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

        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_minusXButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveX(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveX(- ui->stepSpinBox->value());
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_plusYButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveY(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveY(ui->stepSpinBox->value());
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_minusYButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveY(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveY(- ui->stepSpinBox->value());
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_plusZButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveZ(ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveZ(ui->stepSpinBox->value());
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_minusZButton_clicked() {
    if (b!=nullptr){
        if (ui->boxRadioButton->isChecked()){
            b->moveZ(- ui->stepSpinBox->value());
            updateBoxValues();
        }
        else if (ui->box2RadioButton->isChecked()){
            b2->moveZ(- ui->stepSpinBox->value());
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
        mainWindow.updateGlCanvas();
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
        mainWindow.updateGlCanvas();
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
            mainWindow.pushObj(iterations, "Iterations");

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
        mainWindow.updateGlCanvas();
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
            mainWindow.pushObj(iterations, "Iterations");

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
        mainWindow.updateGlCanvas();
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
        mainWindow.pushObj(b, "Box", false);
    }

    std::ifstream myfile;
    myfile.open ("box.bin", std::ios::in | std::ios::binary);
    b->deserialize(myfile);
    myfile.close();
    updateBoxValues();
    mainWindow.updateGlCanvas();
}

void EngineManager::on_iterationsSlider_sliderMoved(int position) {
    if (iterations != nullptr){
        iterations->setVisibleBox(position);
        mainWindow.updateGlCanvas();
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
    mainWindow.updateGlCanvas();
}

void EngineManager::on_solutionsSlider_valueChanged(int value) {
    if (ui->solutionsSlider->isEnabled() && value >= 0){
        solutions->setVisibleBox(value);
        ui->setFromSolutionSpinBox->setValue(value);
        ui->solutionNumberLabel->setText(QString::number((*solutions)[value].getId()));
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_setFromSolutionButton_clicked() {
    if (solutions != nullptr){
        unsigned int value = ui->setFromSolutionSpinBox->value();
        if (value < solutions->getNumberBoxes()){
            if (b == nullptr){
                b = new Box3D(solutions->getBox(value));
                mainWindow.pushObj(b, "Box");
            }
            b->setMin(solutions->getBox(value).getMin());
            b->setMax(solutions->getBox(value).getMax());
            b->setConstraint1(solutions->getBox(value).getConstraint1());
            b->setConstraint2(solutions->getBox(value).getConstraint2());
            b->setConstraint3(solutions->getBox(value).getConstraint3());
            b->setColor(solutions->getBox(value).getColor());
            b->setRotationMatrix(solutions->getBox(value).getRotationMatrix());
            b->setTarget(solutions->getBox(value).getTarget());
            //deleteDrawableObject(b);
            //solutions->getBox(value);
            //b = new Box3D(solutions->getBox(value));
            //mainWindow.pushObj(b, "Box");
            mainWindow.updateGlCanvas();
        }

    }
    else if (d != nullptr) {
        unsigned int value = ui->setFromSolutionSpinBox->value();
        if (value < d->getNumberFaces()) {
            const Dcel::Face* f = d->getFace(value);
            if (b == nullptr){
                b = new Box3D(solutions->getBox(value));
                mainWindow.pushObj(b, "Box");
            }
            Box3D box;
            box.setTarget(nearestNormal(f->getNormal()));
            Pointd p1 = f->getOuterHalfEdge()->getFromVertex()->getCoordinate();
            Pointd p2 = f->getOuterHalfEdge()->getToVertex()->getCoordinate();
            Pointd p3 = f->getOuterHalfEdge()->getNext()->getToVertex()->getCoordinate();
            Pointd bmin = p1;
            bmin = bmin.min(p2);
            bmin = bmin.min(p3);
            bmin = bmin - 1;
            Pointd bmax = p1;
            bmax = bmax.max(p2);
            bmax = bmax.max(p3);
            bmax = bmax + 1;
            box.setMin(bmin);
            box.setMax(bmax);
            box.setColor(colorOfNearestNormal(f->getNormal()));
            box.setConstraint1(p1);
            box.setConstraint2(p2);
            box.setConstraint3(p3);
            *b = box;
        }
    }
}

void EngineManager::on_wireframeDcelCheckBox_stateChanged(int arg1) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        d->setWireframe(arg1 == Qt::Checked);
        mainWindow.updateGlCanvas();
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        baseComplex->setWireframe(arg1 == Qt::Checked);
        mainWindow.updateGlCanvas();
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        he->setWireframe(arg1 == Qt::Checked);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_pointsDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setPointsShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setPointsShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setPointShading();
            mainWindow.updateGlCanvas();
        }
    }
}

void EngineManager::on_flatDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setFlatShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setFlatShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setFlatShading();
            mainWindow.updateGlCanvas();
        }
    }
}

void EngineManager::on_smoothDcelRadioButton_toggled(bool checked) {
    if (d!=nullptr && ui->inputMeshRadioButton->isChecked()){
        if (checked){
            d->setSmoothShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (baseComplex!=nullptr && ui->baseComplexRadioButton->isChecked()) {
        if (checked){
            baseComplex->setSmoothShading();
            mainWindow.updateGlCanvas();
        }
    }
    if (he != nullptr && ui->heightfieldsRadioButton->isChecked()){
        if (checked){
            he->setSmoothShading();
            mainWindow.updateGlCanvas();
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
            t.getContainedDcelFaces(covered, *b);
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
            f->setColor(Color(0,0,255));
        }
        d->update();

        mainWindow.updateGlCanvas();

    }
}

void EngineManager::on_stepDrawGridSpinBox_valueChanged(double arg1) {
    if (g!=nullptr){
        if (arg1 > 0){
            g->setStepDrawGrid(arg1);
            mainWindow.updateGlCanvas();
        }
    }
}

void EngineManager::on_subtractPushButton_clicked() {
    if (solutions!= nullptr && d != nullptr){
        deleteDrawableObject(baseComplex);
        EigenMesh m = (Dcel)*d;
        baseComplex = new DrawableEigenMesh(m);
        mainWindow.pushObj(baseComplex, "Base Complex");
        deleteDrawableObject(he);
        //deleteDrawableObject(entirePieces);
        he = new HeightfieldsList();
        //entirePieces = new HeightfieldsList();
        mainWindow.pushObj(he, "Heightfields");
        //mainWindow.pushObj(entirePieces, "Entire Pieces");
        mainWindow.updateGlCanvas();
        SimpleEigenMesh bc((SimpleEigenMesh)*baseComplex);

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
        Timer tBooleans("Total Booleans Time");
        Engine::booleanOperations(*he, bc, *solutions, false);
        Engine::splitConnectedComponents(*he, *solutions, splittedBoxesToOriginals);
        Engine::glueInternHeightfieldsToBaseComplex(*he, *solutions, bc, *d);
        CGALInterface::AABBTree tree(*d);
        Engine::updatePiecesNormals(tree, *he);
        tBooleans.stopAndPrint();
        ui->showAllSolutionsCheckBox->setEnabled(true);
        solutions->setVisibleBox(0);
        ui->heightfieldsSlider->setMaximum(he->getNumHeightfields()-1);
        ui->allHeightfieldsCheckBox->setChecked(true);
        ui->solutionsSlider->setEnabled(true);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->setFromSolutionSpinBox->setValue(0);
        ui->setFromSolutionSpinBox->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow.deleteObj(baseComplex);
        delete baseComplex;
        baseComplex = new DrawableEigenMesh(bc);
        baseComplex->updateFaceNormals();
        for (unsigned int i = 0; i < baseComplex->getNumberFaces(); ++i){
            Vec3 n = baseComplex->getFaceNormal(i);
            n.normalize();
            Color c = colorOfNearestNormal(n);
            baseComplex->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow.pushObj(baseComplex, "Base Complex");
        mainWindow.updateGlCanvas();
        baseComplex->saveOnObj("BaseComplex.obj");
    }
}

void EngineManager::on_stickPushButton_clicked() {
    if (d!=nullptr && baseComplex != nullptr && he != nullptr){
        SimpleEigenMesh bc = *baseComplex;
        Engine::reduceHeightfields(*he, bc, *d);
        deleteDrawableObject(baseComplex);
        baseComplex = new DrawableEigenMesh(bc);
        baseComplex->updateFaceNormals();
        for (unsigned int i = 0; i < baseComplex->getNumberFaces(); ++i){
            Vec3 n = baseComplex->getFaceNormal(i);
            n.normalize();
            Color c = colorOfNearestNormal(n);
            baseComplex->setFaceColor(c.redF(), c.greenF(), c.blueF(), i);
        }
        mainWindow.pushObj(baseComplex, "Base Complex");
    }
    mainWindow.updateGlCanvas();
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
        BoundingBox bb = d->getBoundingBox();
        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        if (originalMesh.getNumberVertices() > 0){
            originalMesh.scale(bb, d->getBoundingBox());
        }
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
        mainWindow.updateGlCanvas();
    }
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        mainWindow.pushObj(solutions, "Solutions");
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
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_allHeightfieldsCheckBox_stateChanged(int arg1) {
    if (arg1 == Qt::Checked){
        if (he != nullptr){
            he->setVisibleHeightfield(-1);
            //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(-1);
            mainWindow.updateGlCanvas();
        }
    }
    else {
        if (he != nullptr){
            //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(ui->heightfieldsSlider->value());
            he->setVisibleHeightfield(ui->heightfieldsSlider->value());
            mainWindow.updateGlCanvas();
        }
    }
}

void EngineManager::on_heightfieldsSlider_valueChanged(int value) {
    if (he != nullptr){
        ui->solutionsSlider->setValue(value);
        //if (entirePieces != nullptr) entirePieces->setVisibleHeightfield(value);
        he->setVisibleHeightfield(value);
        mainWindow.updateGlCanvas();

        std::cerr << "Box id: " << solutions->getBox(value).getId() << "\n";
        std::cerr << "Generated by: " << splittedBoxesToOriginals[solutions->getBox(value).getId()] << "\n";

        ui->prioritySpinBox->setValue(splittedBoxesToOriginals[solutions->getBox(value).getId()]);
    }
}

void EngineManager::on_minXSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinX(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_minYSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinY(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_minZSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMinZ(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_maxXSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxX(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_maxYSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxY(arg1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_maxZSpinBox_valueChanged(double arg1) {
    if (b!=nullptr){
        b->setMaxZ(arg1);
        mainWindow.updateGlCanvas();
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
    deleteDrawableObject(b2);
    if (originalMesh.getNumberVertices()>0){
        mainWindow.deleteObj(&originalMesh);
        originalMesh.clear();
    }
    g = nullptr;
    d = nullptr;
    b = nullptr;
    iterations = nullptr;
    solutions = nullptr;
    baseComplex = nullptr;
    he = nullptr;
    b2 = nullptr;
    markerMesh.clear();

    originalSolutions.clearBoxes();
    alreadySplitted = false;
    splittedBoxesToOriginals.clear();
    priorityBoxes.clear();
    //deleteDrawableObject(entirePieces);
}

void EngineManager::on_reorderBoxes_clicked() {
    if (d != nullptr && solutions != nullptr){


        Timer tGraph("Total Time Graph optimization");
        Array2D<int> ordering = Splitting::getOrdering(*solutions, *d, splittedBoxesToOriginals, priorityBoxes, userArcs);
        tGraph.stopAndPrint();
        //solutions->setIds();
        solutions->sort(ordering);
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            std::cerr << solutions->getBox(i).getId() << " ";
            //solutions->getBox(i).getEigenMesh().saveOnObj("ob" + std::to_string(i) + "_" + std::to_string(solutions->getBox(i).getId()) + ".obj");
        }
        for (std::pair<unsigned int, unsigned int> p : splittedBoxesToOriginals){
            std::cerr << "\nBox " << p.first << " splitted from Box " <<  p.second ;
        }
        std::cerr << "\n";
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow.updateGlCanvas();
    }
}

void Engine::booleanOperations(HeightfieldsList &he, SimpleEigenMesh &bc, BoxList &solutions, bool alternativeColors) {
    deleteDuplicatedBoxes(solutions);
    Timer timer("Boolean Operations");
    he.resize(solutions.getNumberBoxes());
    const double pass = 240.0 / solutions.getNumberBoxes();
    Color c;
    for (unsigned int i = 0; i <solutions.getNumberBoxes() ; i++){
        c.setHsv((int)(i*pass),255,255);
        SimpleEigenMesh box;
        SimpleEigenMesh intersection;
        box = solutions.getBox(i).getEigenMesh();
        #ifdef BOOL_DEBUG
        box.saveOnObj("bool_" + std::to_string(i) + "_" + std::to_string(solutions.getBox(i).getId()) + ".obj");
        #endif
        EigenMeshAlgorithms::intersection(intersection, bc, box);
        EigenMeshAlgorithms::difference(bc, bc, box);
        DrawableEigenMesh dimm(intersection);
        if (alternativeColors){
            dimm.setFaceColor(c.redF(), c.greenF(), c.blueF());
            he.addHeightfield(dimm, solutions.getBox(i).getRotatedTarget(), i, false);
        }
        else
            he.addHeightfield(dimm, solutions.getBox(i).getRotatedTarget(), i, true);
        std::cerr << i << ": " << solutions[i].getId() << "\n";
    }
    timer.stopAndPrint();
    for (int i = he.getNumHeightfields()-1; i >= 0 ; i--) {
        if (he.getNumberVerticesHeightfield(i) == 0) {
            he.removeHeightfield(i);
            solutions.removeBox(i);
        }
    }
}

void EngineManager::on_loadOriginalPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open Eigen Mesh",
                       ".",
                       "OBJ(*.obj)");
    if (!filename.isEmpty()) {
        originalMesh.readFromObj(filename.toStdString());
        if (! (mainWindow.contains(&originalMesh)))
            mainWindow.pushObj(&originalMesh, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_loadSmoothedPushButton_clicked() {
    QString filename = QFileDialog::getOpenFileName(nullptr,
                       "Open Dcel Mesh",
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
            mainWindow.pushObj(d, filename.toStdString().substr(filename.toStdString().find_last_of("/") + 1));
            mainWindow.updateGlCanvas();
        }
        else {
            delete d;
            d = nullptr;
            QMessageBox msgBox;
            msgBox.setText("File Format not supported.");
            msgBox.exec();
        }
    }
}

void EngineManager::on_taubinPushButton_clicked() {
    if (d == nullptr){
        d = new DrawableDcel(Reconstruction::taubinSmoothing(originalMesh, ui->nItersSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value()));
        d->setWireframe(true);
        d->setPointsShading();
        d->update();
        mainWindow.pushObj(d, "Smoothed Mesh");
        mainWindow.updateGlCanvas();
    }
    else {
        *d = DrawableDcel(Reconstruction::taubinSmoothing(*d, ui->nItersSpinBox->value(), ui->lambdaSpinBox->value(), ui->muSpinBox->value()));
        d->update();
        mainWindow.updateGlCanvas();
    }
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
            std::vector< std::vector<EigenMesh> > packs = Packing::getPacks(tmp, myHe);
            EigenMeshAlgorithms::makeBox(packSize).saveOnObj(QString(foldername + "/box.obj").toStdString());
            for (unsigned int i = 0; i < packs.size(); i++){
                QString pstring = foldername + "/pack" + QString::number(i) + ".obj";
                EigenMesh packMesh = packs[i][0];
                QString bstring = foldername + "/b" + QString::number(i);
                for (unsigned int j = 0; j < packs[i].size(); j++){
                    QString meshName = bstring + "p" + QString::number(j) + ".obj";
                    //packs[i][j].saveOnObj(meshName.toStdString());
                    Dcel d(packs[i][j]);
                    d.updateVertexNormals();
                    d.saveOnObjFile(meshName.toStdString());
                    //Dcel d(packs[i][j]);
                    //Segmentation s(d);
                    //Dcel dd = s.getDcelFromSegmentation();
                    //dd.triangulate();
                    //dd.saveOnObjFile(meshName.toStdString());
                    if (j > 0){
                        packMesh = EigenMesh::merge(packMesh, packs[i][j]);
                    }
                }
                //packMesh.saveOnObj(pstring.toStdString());
                Dcel d(packMesh);
                d.updateVertexNormals();
                d.saveOnObjFile(pstring.toStdString());

                /*BoundingBox bb = d.getBoundingBox();
                SimpleEigenMesh plane;
                plane.addVertex(bb.getMinX(), bb.getMinY(), bb.getMinZ() - EPSILON);
                plane.addVertex(bb.getMaxX(), bb.getMinY(), bb.getMinZ() - EPSILON);
                plane.addVertex(bb.getMaxX(), bb.getMaxY(), bb.getMinZ() - EPSILON);
                plane.addVertex(bb.getMinX(), bb.getMaxY(), bb.getMinZ() - EPSILON);
                plane.addFace(0, 1, 2);
                plane.addFace(0, 2, 3);
                plane.saveOnObj(foldername.toStdString() + "/plane.obj");*/
            }
        }
    }
}

void EngineManager::on_smartPackingPushButton_clicked() {
    if (he != nullptr){
        QString foldername = QFileDialog::getExistingDirectory(nullptr, "SaveObjs");
        if (!foldername.isEmpty()){
            HeightfieldsList myHe = *he;
            Packing::rotateAllPieces(myHe);
            BoundingBox packSize(Pointd(), Pointd(ui->sizeXPackSpinBox->value(), ui->sizeYPackSpinBox->value(), ui->sizeZPackSpinBox->value()));
            BoundingBox limits = packSize;
            double l = packSize.getMaxY();
            std::vector< std::vector<EigenMesh> > packs;
            do {
                limits.setMaxY(l);
                double factor;
                Packing::getMaximum(myHe, limits, factor);
                Packing::scaleAll(myHe, factor);
                std::vector< std::vector<std::pair<int, Pointd> > > tmp = Packing::pack(myHe, packSize);
                packs = Packing::getPacks(tmp, myHe);
                l -= 0.1;
            } while (packs.size() > 1);
            EigenMeshAlgorithms::makeBox(packSize).saveOnObj(QString(foldername + "/box.obj").toStdString());

            for (unsigned int i = 0; i < packs.size(); i++){
                QString pstring = foldername + "/pack" + QString::number(i) + ".obj";
                EigenMesh packMesh = packs[i][0];
                packs[i][0].saveOnObj(foldername.toStdString() + "/p0b0.obj");
                for (unsigned int j = 1; j < packs[i].size(); j++){
                    packs[i][j].saveOnObj(foldername.toStdString() + "/p0b" + std::to_string(j) + ".obj");
                    packMesh = EigenMesh::merge(packMesh, packs[i][j]);
                }
                //packMesh.saveOnObj(pstring.toStdString());
                Dcel d(packMesh);
                //Eigen::MatrixXd m = Common::getRotationMatrix(Vec3(-1,0,0), M_PI/2);
                //d.rotate(m);
                d.updateFaceNormals();
                d.updateVertexNormals();
                d.saveOnObjFile(pstring.toStdString());
            }
        }
    }
}


void EngineManager::on_reconstructionPushButton_clicked() {
    if (d != nullptr && he != nullptr){
        std::vector< std::pair<int,int> > mapping = Reconstruction::getMapping(*d, *he);
        Reconstruction::reconstruction(*d, mapping, originalMesh, *solutions);
        d->update();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_putBoxesAfterPushButton_clicked() {
    int nbox = ui->coveredTrianglesSpinBox->value();
    if (nbox >= 0 && solutions != nullptr){
        Box3D small = solutions->getBox(nbox);
        solutions->removeBox(nbox);
        solutions->addBox(small);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_snappingPushButton_clicked() {
    if (solutions != nullptr && d != nullptr){
        double av = d->getAverageHalfEdgesLength();
        CGALInterface::AABBTree tree(*d);

        //snapping
        double epsilon = ui->snappingSpinBox->value();
        for (unsigned int i = 0; i < solutions->getNumberBoxes()-1; i++){
            Box3D b1 = solutions->getBox(i);
            for (unsigned int j = i+1; j < solutions->getNumberBoxes(); j++){
                Box3D b2 = solutions->getBox(j);
                for (unsigned int coord = 0; coord < 3; coord++) {
                    if (std::abs(b1(coord)-b2(coord)) < epsilon) {
                        b2(coord) = b1(coord);
                    }
                    if (std::abs(b1(coord)-b2(coord+3)) < epsilon){
                        b2(coord+3) = b1(coord);
                    }
                    if (std::abs(b1(coord+3)-b2(coord)) < epsilon){
                        b2(coord) = b1(coord+3);
                    }
                    if (std::abs(b1(coord+3)-b2(coord+3)) < epsilon){
                        b2(coord+3) = b1(coord+3);
                    }
                }
                solutions->setBox(j, b2);
            }
        }

        //new: forced snapping
        solutions->calculateTrianglesCovered(tree);
        std::vector<unsigned int> trianglesCovered(d->getNumberFaces(), 0);
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            const std::set<unsigned int>& s = (*solutions)[i].getTrianglesCovered();
            for (unsigned int j : s){
                trianglesCovered[j]++;
            }
        }
        for (unsigned int i = 0; i < solutions->getNumberBoxes()-1; i++){
            Box3D b1 = solutions->getBox(i);
            for (unsigned int j = i+1; j < solutions->getNumberBoxes(); j++){
                Box3D b2 = solutions->getBox(j);
                if (Splitting::boxesIntersect(b1,b2)){
                    if (Splitting::isDangerousIntersection(b1, b2, tree, false) ||
                            Splitting::isDangerousIntersection(b2, b1, tree, false)){
                        if (Engine::smartSnapping(b1, b2, trianglesCovered, tree)){
                            std::cerr << "Smart snapping " << j << " in " << i << "\n";
                            solutions->setBox(j, b2);
                        }
                        else if (Engine::smartSnapping(b2, b1, trianglesCovered, tree)){
                            std::cerr << "Smart snapping " << i << " in " << j << "\n";
                            solutions->setBox(i, b1);
                        }
                    }
                }
            }
        }
        //

        solutions->generatePieces(av*LENGTH_MULTIPLIER);
        solutions->calculateTrianglesCovered(tree);
        solutions->sortByTrianglesCovered();

        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            for (unsigned int j = 0; j < solutions->getNumberBoxes() && j != i; j++){
                if ((*solutions)[i].getTarget() == (*solutions)[j].getTarget() &&  Splitting::boxesIntersect((*solutions)[i], (*solutions)[j])){
                    std::cerr << "Boxes " << i << " and " << j << " may be merged. \n";
                    Box3D& a = (*solutions)[i];
                    Box3D& b = (*solutions)[j];
                    int t = indexOfNormal((*solutions)[i].getTarget());
                    assert(t >= 0);
                    double baseA = a.getBaseLevel(), baseB = b.getBaseLevel();
                    if (baseA <= baseB){
                        if (baseA != baseB){
                            if (t < 3){
                                //x, y, z
                                Box3D tmpa = a;
                                tmpa.setBaseLevel(baseB);
                                std::list<unsigned int> newTrianglesA;
                                tree.getCompletelyContainedDcelFaces(newTrianglesA, tmpa);
                                std::set<unsigned int> nonCoveredTrianglesA(newTrianglesA.begin(), newTrianglesA.end());
                                nonCoveredTrianglesA = Common::setDifference(a.getTrianglesCovered(), nonCoveredTrianglesA);
                                bool shrink = true;
                                for (unsigned int t : nonCoveredTrianglesA){
                                    if (trianglesCovered[t] == 1)
                                        shrink = false;
                                }
                                if (shrink){
                                    for (unsigned int t : nonCoveredTrianglesA){
                                        trianglesCovered[t]--;
                                    }
                                    a.setBaseLevel(baseB);
                                    a.setTrianglesCovered(std::set<unsigned int>(newTrianglesA.begin(), newTrianglesA.end()));
                                    std::cerr << "Box " << i << " shrinked to level of Box " << j << "\n";

                                    SimpleEigenMesh u = EigenMeshAlgorithms::union_(a.getEigenMesh(), b.getEigenMesh());
                                    a.setEigenMesh(u);
                                    a.setTrianglesCovered(Common::setUnion(a.getTrianglesCovered(), b.getTrianglesCovered()));
                                    a.setMin(u.getBoundingBox().min());
                                    a.setMax(u.getBoundingBox().max());
                                    (*solutions)[i].setSplitted(true);
                                    solutions->removeBox(j);
                                    j--;
                                }
                            }
                            else {
                                //-x, -y, -z
                                Box3D tmpb = b;
                                tmpb.setBaseLevel(baseA);
                                std::list<unsigned int> newTrianglesB;
                                tree.getCompletelyContainedDcelFaces(newTrianglesB, tmpb);
                                std::set<unsigned int> nonCoveredTrianglesB(newTrianglesB.begin(), newTrianglesB.end());
                                nonCoveredTrianglesB = Common::setDifference(b.getTrianglesCovered(), nonCoveredTrianglesB);
                                bool shrink = true;
                                for (unsigned int t : nonCoveredTrianglesB){
                                    if (trianglesCovered[t] == 1)
                                        shrink = false;
                                }
                                if (shrink){
                                    for (unsigned int t : nonCoveredTrianglesB){
                                        trianglesCovered[t]--;
                                    }
                                    b.setBaseLevel(baseA);
                                    b.setTrianglesCovered(std::set<unsigned int>(newTrianglesB.begin(), newTrianglesB.end()));
                                    std::cerr << "Box " << j << " shrinked to level of Box " << i << "\n";

                                    SimpleEigenMesh u = EigenMeshAlgorithms::union_(a.getEigenMesh(), b.getEigenMesh());
                                    a.setEigenMesh(u);
                                    a.setTrianglesCovered(Common::setUnion(a.getTrianglesCovered(), b.getTrianglesCovered()));
                                    a.setMin(u.getBoundingBox().min());
                                    a.setMax(u.getBoundingBox().max());
                                    (*solutions)[i].setSplitted(true);
                                    solutions->removeBox(j);
                                    j--;
                                }
                            }
                        }
                    }
                }
            }
        }

        //merging
        /*for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
            for (unsigned int j = i+1; j < solutions->getNumberBoxes(); j++){
                Box3D a = solutions->getBox(i);
                Box3D b = solutions->getBox(j);
                if (a.getTarget() == b.getTarget()){
                    bool merge = false;
                    for (unsigned int f = 0; f < Common::AXIS.size(); f++){
                        if (Common::AXIS[f] == b.getTarget()){
                            if (f < 3){
                                if(Common::epsilonEqual(a.getMin()[f], b.getMin()[f], epsilon)){
                                    if (a.intersect(b)){
                                        merge = true;
                                        if (a.getMin()[f] < b.getMin()[f]){
                                            b.min()[f] = a.min()[f];
                                            b.generateEigenMesh(av*7);
                                        }
                                        else if (a.getMin()[f] > b.getMin()[f]){
                                            a.min()[f] = b.min()[f];
                                            a.generateEigenMesh(av*7);
                                        }
                                    }
                                }
                            }
                            else {
                                if(Common::epsilonEqual(a.getMax()[f%3], b.getMax()[f%3], 2)){
                                    if (a.intersect(b)){
                                        merge = true;
                                        if (b.getMax()[f%3] < a.getMin()[f%3]){
                                            b.max()[f%3] = a.max()[f%3];
                                            b.generateEigenMesh(d->getAverageHalfEdgesLength()*7);
                                        }
                                        else if (b.getMax()[f%3] > a.getMax()[f%3]){
                                            a.max()[f%3] = b.max()[f%3];
                                            a.generateEigenMesh(d->getAverageHalfEdgesLength()*7);
                                            solutions->setBox(i, a);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    if (merge) {
                        std::cerr << "Merged boxes " << i << "and" << j << "\n";
                        SimpleEigenMesh m1 = a.getEigenMesh();
                        SimpleEigenMesh m2 = b.getEigenMesh();
                        SimpleEigenMesh un = EigenMeshAlgorithms::union_(m1, m2);
                        a.setEigenMesh(un);
                        BoundingBox bb = un.getBoundingBox();
                        a.setMin(bb.min());
                        a.setMax(bb.max());
                        a.setSplitted(true);
                        a.addTrianglesCovered(b.getTrianglesCovered());
                        solutions->setBox(i, a);
                        solutions->removeBox(j);
                        j--;
                    }
                }
            }
        }*/
        //setting ids
        solutions->sortByTrianglesCovered();
        solutions->setIds();
        if (! alreadySplitted){
            originalSolutions = *solutions;
            alreadySplitted = true;
        }

        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow.updateGlCanvas();
    }


}

void EngineManager::on_colorPiecesPushButton_clicked() {
    if (he!=nullptr && d != nullptr){
        d->updateVertexNormals();
        CGALInterface::AABBTree tree(*d);
        std::array<Color, 10> colors;
        colors[0] = Color(182, 215, 168); //
        colors[1] = Color(159, 197, 232); //
        colors[2] = Color(234, 153, 153); //
        colors[3] = Color(255, 229, 153); //
        colors[4] = Color(162, 196, 201); //
        colors[5] = Color(213, 166, 189); //
        colors[6] = Color(164, 194, 244); //
        colors[7] = Color(221, 126, 107);//
        colors[8] = Color(249, 203, 156);//
        colors[9] = Color(180, 167, 214);//


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
        std::vector<Color> heColors(he->getNumHeightfields(), Color(0,0,0));
        for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
            if (!colored[i]){
                std::set<Color> adjColors;
                for (int adj : adjacences[i]){
                    if (colored[adj])
                        adjColors.insert(heColors[adj]);
                }

                Color color;
                bool finded = false;
                unsigned int k = i %10;
                do {
                    if (adjColors.find(colors[k]) == adjColors.end()){
                        finded = true;
                        color = colors[k];
                    }
                    k = (k+1)%10;
                } while(k != i %10 && !finded);

                //
                /*for (unsigned int k = 0; k < 10 && !finded; k++){
                    if (adjColors.find(colors[k]) == adjColors.end()){
                        finded = true;
                        color = colors[k];
                    }
                }*/
                //
                assert(finded);
                heColors[i] = color;
                colored[i] = true;


                EigenMesh mesh = he->getHeightfield(i);
                Dcel d(mesh);
                Engine::updatePieceNormals(tree, d);
                mesh = EigenMesh(d);
                mesh.setFaceColor(color.redF(),color.greenF(),color.blueF());
                he->setHeightfield(mesh, i);
            }
        }
    }
    mainWindow.updateGlCanvas();
}

void EngineManager::on_deleteBoxesPushButton_clicked() {
    if (solutions != nullptr && d != nullptr){
        int n = Engine::deleteBoxes(*solutions, *d);
        //int n = Engine::deleteBoxesGSC(*solutions, *d);
        std::cerr << "N survived boxes: " << n << "\n";
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_pushButton_clicked() {
    if (g == nullptr && d!= nullptr) {
        BoundingBox bb = d->getBoundingBox();
        Engine::scaleAndRotateDcel(*d, 0, ui->factorSpinBox->value());
        if (originalMesh.getNumberVertices() > 0){
            originalMesh.scale(bb, d->getBoundingBox());
        }
        std::set<const Dcel::Face*> flippedFaces, savedFaces;
        Engine::getFlippedFaces(flippedFaces, savedFaces, *d, XYZ[ui->targetComboBox->currentIndex()], (double)ui->toleranceSlider->value()/100, ui->areaToleranceSpinBox->value());
        updateColors(ui->toleranceSlider->value(), ui->areaToleranceSpinBox->value());
        d->update();
        mainWindow.updateGlCanvas();
    }
    if (d!=nullptr){
        deleteDrawableObject(solutions);
        solutions = new BoxList();
        mainWindow.pushObj(solutions, "Solutions");
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
        mainWindow.updateGlCanvas();
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


void EngineManager::on_explodePushButton_clicked() {
    if (d != nullptr && he != nullptr){
        double dist = ui->explodeSpinBox->value();
        Pointd bc = d->getBarycenter();
        he->explode(bc, dist);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_createBoxPushButton_clicked() {
    if (b == nullptr){
        b = new Box3D(Pointd(0,0,0), Pointd(3,3,3));
        b->setConstraint1(Pointd(1, 1.5, 1.5));
        b->setConstraint2(Pointd(1.5, 1.5, 1));
        b->setConstraint3(Pointd(1, 1.5, 1));
        b->setColor(Color(0,0,0));
        mainWindow.pushObj(b, "Box");
    }
    //deleteDrawableObject(b);
    //solutions->getBox(value);
    //b = new Box3D(solutions->getBox(value));
    //mainWindow.pushObj(b, "Box");
    mainWindow.updateGlCanvas();
}

void EngineManager::on_coveredTrianglesPushButton_clicked() {
    if (solutions != nullptr && d != nullptr){
        for (Dcel::Face* f : d->faceIterator())
            f->setColor(Color(128,128,128));
        std::vector< std::set<const Dcel::Face*> > trianglesCovered (solutions->getNumberBoxes());
        CGALInterface::AABBTree tree(*d);
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++) {
            std::list<const Dcel::Face*> ltmp = tree.getCompletelyContainedDcelFaces(solutions->getBox(i));
            std::set<const Dcel::Face*> tcbox(ltmp.begin(), ltmp.end());
            trianglesCovered[i] = tcbox;
        }
        int h = 0;
        int step = 240 / solutions->getNumberBoxes();
        Color c;
        for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++) {
            c.setHsv(h, 255, 255);
            std::set<const Dcel::Face*> lonelyTriangles = trianglesCovered[i];
            for (unsigned int j = 0; j < solutions->getNumberBoxes(); j++){
                if (j != i){
                    lonelyTriangles = Common::setDifference(lonelyTriangles, trianglesCovered[j]);
                }
            }
            //assert(lonelyTriangles.size() > 0);
            for (const Dcel::Face* f : lonelyTriangles){
                d->getFace(f->getId())->setColor(c);
            }
            h+=step;
        }
        d->update();
        d->saveOnObjFile("boxes/MeshTriangles.obj");

        mainWindow.updateGlCanvas();
    }

}

void EngineManager::on_drawBoxMeshCheckBox_stateChanged(int arg1) {
    if (solutions!=nullptr){
        solutions->visualizeEigenMeshBox(arg1==Qt::Checked);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_markerMeshPushButton_clicked() {
    if (he != nullptr && d != nullptr){
        markerMesh = Engine::getMarkerMesh(*he, *d);
        markerMesh.setPointsShading();
        markerMesh.setWireframe(true);
        markerMesh.setWireframeWidth(4);
        mainWindow.pushObj(&markerMesh, "MarkerMesh");
    }
}

void EngineManager::on_splitConnectedComponentsPushButton_clicked() {
    if (d != nullptr && solutions != nullptr){
        Engine::boxPostProcessing(*solutions, *d);
    }
}

void EngineManager::on_globalOptimalOrientationPushButton_clicked() {
    if (d != nullptr){
        Eigen::Matrix3d matr = Orientation::optimalOrientation(*d);
        std::cerr << "Rotation Matrix: " << matr << "\n";
        d->rotate(matr);
        Pointd c = d->getBoundingBox().center();
        d->translate(-c);
        d->update();
        if (originalMesh.getNumberVertices() > 0){
            originalMesh.rotate(matr);
            originalMesh.translate(-c);
        }
        mainWindow.fitScene();
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_experimentButton_clicked() {
    std::vector<unsigned int> triangles(d->getNumberFaces(), 0);
    for (unsigned int i = 0; i < solutions->getNumberBoxes(); i++){
        for (unsigned int t : (*solutions)[i].getTrianglesCovered()){
            triangles[t]++;
        }
    }
}

void EngineManager::on_createBox2PushButton_clicked() {
    if (b2 == nullptr){
        b2 = new Box3D(Pointd(0,0,0), Pointd(3,3,3));
        b2->setConstraint1(Pointd(1, 1.5, 1.5));
        b2->setConstraint2(Pointd(1.5, 1.5, 1));
        b2->setConstraint3(Pointd(1, 1.5, 1));
        b2->setColor(Color(0,0,0));
        mainWindow.pushObj(b2, "Box2");
    }
    //deleteDrawableObject(b);
    //solutions->getBox(value);
    //b = new Box3D(solutions->getBox(value));
    //mainWindow.pushObj(b, "Box");
    mainWindow.updateGlCanvas();
}

void EngineManager::on_restoreBoxesPushButton_clicked() {
    deleteDrawableObject(solutions);
    deleteDrawableObject(baseComplex);
    deleteDrawableObject(he);
    baseComplex = nullptr;
    he = nullptr;
    solutions = new BoxList(originalSolutions);
    mainWindow.pushObj(solutions, "Boxes");
    ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
    ui->solutionsSlider->setValue(0);
    splittedBoxesToOriginals.clear();
    mainWindow.updateGlCanvas();
    mainWindow.fitScene();
}

void EngineManager::on_pushPriorityBoxButton_clicked() {
    std::string s = ui->listLabel->text().toStdString();
    ui->listLabel->setText(QString::fromStdString(std::to_string(ui->prioritySpinBox->value()) + "; " + s));
    priorityBoxes.push_front(ui->prioritySpinBox->value());
}

void EngineManager::on_clearPriorityPushButton_clicked() {
    priorityBoxes.clear();
    ui->listLabel->setText("");
}

void EngineManager::on_pushUserArcButton_clicked() {
    std::pair<unsigned int, unsigned int> p(ui->boxBSpinBox->value(), ui->boxASpinBox->value());
    userArcs.push_back(p);
    ui->userArcsLabel->setText(ui->userArcsLabel->text() + QString::fromStdString("(" + std::to_string(p.first) + "; " + std::to_string(p.second) + "); "));
}

void EngineManager::on_clearUserArcsPushButton_clicked() {
    userArcs.clear();
    ui->userArcsLabel->setText("");
}

void EngineManager::on_deleteBlockPushButton_clicked() {
    if (solutions != nullptr && he != nullptr){
        unsigned int index = solutions->getIndexOf(ui->deleteBlockSpinBox->value());
        solutions->removeBox(index);
        he->removeHeightfield(index);
        ui->solutionsSlider->setMaximum(solutions->getNumberBoxes()-1);
        ui->heightfieldsSlider->setMaximum(he->getNumHeightfields()-1);
        mainWindow.updateGlCanvas();
    }
}

void EngineManager::on_sdfPushButton_clicked() {
    if (d != nullptr && he != nullptr){
        double threshold = d->getBoundingBox().diag() /10000;
        std::cerr << "Threshold: " << threshold << "\n";
        for (unsigned int i = 0; i < he->getNumHeightfields(); i++){
            EigenMesh m = he->getHeightfield(i);
            m.removeDegenerateTriangles(1e-1);
            m.saveOnObj("prova.obj");
            if (EigenMeshAlgorithms::isEdgeManifold(m)){
                Engine::tinyFeatures(m, threshold);
            }
            else
                std::cerr << "Block n. " << i << " is not Edge Manifold\n";
            he->setHeightfield(m, i);
        }
    }
    mainWindow.updateGlCanvas();
}

void EngineManager::on_deleteBoxPushButton_clicked() {
    if (b != nullptr){
        deleteDrawableObject((b));
        b = nullptr;
    }
}

void EngineManager::on_deleteBox2PushButton_clicked() {
    if (b2 != nullptr){
        deleteDrawableObject((b2));
        b2 = nullptr;
    }
}
