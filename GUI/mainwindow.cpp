/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QScrollArea>
#include <QCheckBox>

/**
 * @brief Crea una nuova mainWindow composta da canvas, toolBox avente 0 frame e scrollArea.
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow),
                                          nMeshes(0),
                                          first(true),
                                          debugObjects(nullptr){
    ui->setupUi(this);
    ui->toolBox->removeItem(0);

    checkBoxMapper = new QSignalMapper(this);
    connect(checkBoxMapper, SIGNAL(mapped(int)), this, SLOT(checkBoxClicked(int)));
    connect(ui->glCanvas, SIGNAL(objectPicked(int)),
            this, SLOT(slotObjectClicked(int)));
    QVBoxLayout * layout = new QVBoxLayout(this);
    ui->scrollArea->setLayout(layout);
    ui->scrollArea->layout()->setSpacing(0);
    ui->scrollArea->layout()->setMargin(0);

    showMaximized();
}

MainWindow::~MainWindow() {
    delete ui;
}

/**
 * @brief Centra la scena tenendo conto di tutti gli oggetti visibili all'interno di essa
 */
void MainWindow::updateGlCanvas() {
    //ui->glCanvas->fitScene();
    ui->glCanvas->updateGL();
}

/**
 * @brief Aggiunge un DrawableObject alla scena e la relativa checkBox nella scrollBar.
 *
 * Aggiorna in automatico la scena visualizzata.
 *
 * @param obj: nuovo oggetto da visualizzare nella canvas
 * @param checkBoxName: nome assegnato alla checkbox relativa al nuovo oggetto
 */
void MainWindow::pushObj(DrawableObject *obj, std::string checkBoxName, bool b) {
    ui->glCanvas->pushObj(obj);
    if (b) ui->glCanvas->fitScene();
    ui->glCanvas->updateGL();

    QCheckBox * cb = new QCheckBox();
    cb->setText(checkBoxName.c_str());
    cb->setEnabled(true);
    cb->setChecked(true);

    checkBoxes[nMeshes] = cb;
    mapObjects.insert( boost::bimap<int, DrawableObject*>::value_type(nMeshes, obj ) );
    connect(cb, SIGNAL(stateChanged(int)), checkBoxMapper, SLOT(map()));
    checkBoxMapper->setMapping(cb, nMeshes);
    nMeshes++;

    ui->scrollArea->layout()->addWidget(cb);
    ui->scrollArea->layout()->setAlignment(cb, Qt::AlignTop);
}

/**
 * @brief Elimina il DrawableObject dalla scena (non esegue nessuna free!) e la relativa checkBox nella scrollBar.
 *
 * Aggiorna in automatico la scena visualizzata.
 *
 * @param obj: oggetto che verrà rimosso dalla canvas
 */
void MainWindow::deleteObj(DrawableObject* obj, bool b) {
    int i = mapObjects.right.at(obj);

    QCheckBox * cb = checkBoxes[i];
    checkBoxMapper->removeMappings(cb);
    cb->setVisible(false);
    ui->scrollArea->layout()->removeWidget(cb);

    checkBoxes.erase(i);
    mapObjects.left.erase(i);

    delete cb;

    ui->glCanvas->deleteObj(obj);
    if (b) ui->glCanvas->fitScene();
    ui->glCanvas->updateGL();
}

/**
 * @brief Restituisce il BoundingBox di tutti gli oggetti \i presenti presenti nella canvas
 * @return il bounding box contenente gli oggetti visibili
 */
BoundingBox MainWindow::getFullBoundingBox() {
    return ui->glCanvas->getFullBoundingBox();
}

/**
 * @brief Restituisce il numero di oggetti visibili presenti nella canvas.
 * @return intero rappresentante il numero di oggetti visibili
 */
int MainWindow::getNumberVisibleObjects() {
    return ui->glCanvas->getNumberVisibleObjects();
}

/**
 * @brief salva uno snapshot della scena presente nella canvas.
 *
 * Aggiorna in automatico la scena visualizzata.
 */
void MainWindow::saveSnapshot() {
    ui->glCanvas->updateGL();
    ui->glCanvas->saveSnapshot();
}

/**
 * @brief Visualizza gli assi xyz in base al booleano passato in input.
 *
 * Aggiorna in autmoatico la scena visualizzata.
 * @param[in] b: se true visualizza gli assi, se false non li visualizza.
 */
void MainWindow::drawAxis(bool b) {
    ui->glCanvas->setAxisIsDrawn(b);
    ui->glCanvas->updateGL();
}

/**
 * @brief Aggiunge un manager (frame) alla toolbox della mainWindow.
 * @param[in] f: il QFrame del manager da aggiungere
 * @param[in] name: nome associato al manager inserito
 * @param[in] parent: di default è la toolbox alla quale aggiungiamo il manager
 * @return un intero rappresentante l'indice del manager inserito
 */
int MainWindow::addManager(QFrame * f, std::string name, QToolBox * parent) {
    if (parent == nullptr) parent = ui->toolBox;
    ui->toolBox->insertItem(managers.size(), f, QString(name.c_str()));

    f->show();
    managers.push_back(f);
    //adjustSize();
    return managers.size()-1;
}

/**
 * @brief Restituisce il puntatore al manager identificato dall'indice passato come parametro.
 * @param[in] i: indice del manager da restituire
 * @return il puntatore al manager se esiste, nullptr altrimenti
 */
QFrame *MainWindow::getManager(unsigned int i) {
    if (i < managers.size()) return managers[i];
    else return nullptr;
}

/**
 * @brief Rinomina il manager identificato dall'indice passato come parametro
 * @param[in] i: indice del manager da rinominare
 * @param[in] s: il nuovo nome del manager
 */
void MainWindow::renameManager(unsigned int i, std::string s) {
    if (i < managers.size())
        ui->toolBox->setItemText(i, QString(s.c_str()));
}

/**
 * @brief Modifica il manager visualizzato all'i-esimo.
 * @param[in] i: indice del manager da visualizzare
 */
void MainWindow::setCurrentIndexToolBox(unsigned int i){
    if (i < managers.size())
        ui->toolBox->setCurrentIndex(i);
}

void MainWindow::enableDebugObjects() {
    if (debugObjects == nullptr){
        debugObjects = new DrawableDebugObjects();
        pushObj(debugObjects, "Debug Objects");
    }
}

void MainWindow::disableDebugObjects() {
    if (debugObjects != nullptr){
        deleteObj(debugObjects);
        delete debugObjects;
        debugObjects = nullptr;
    }
    ui->glCanvas->updateGL();
}

void MainWindow::addDebugSphere(const Pointd& center, double radius, const QColor& color, int precision) {
    if (debugObjects!= nullptr){
        debugObjects->addDebugSphere(center, radius, color, precision);
        //ui->glCanvas->updateGL();
    }
}

void MainWindow::clearDebugSpheres() {
    if (debugObjects!= nullptr){
        debugObjects->clearDebugSpheres();
        ui->glCanvas->updateGL();
    }
}

void MainWindow::addDebugCylinder(const Pointd& a, const Pointd& b, double radius, const QColor color) {
    if (debugObjects!=nullptr){
        debugObjects->addDebugCylinder(a,b,radius, color);
        ui->glCanvas->updateGL();
    }
}

void MainWindow::clearDebugCylinders() {
    if (debugObjects!=nullptr){
        debugObjects->clearDebugCylinders();
        ui->glCanvas->updateGL();
    }
}

/**
 * @brief Evento i-esima checkBox cliccata, modifica la visibilità dell'oggetto ad essa collegato
 * @param[in] i: indice della checkBox cliccata
 */
void MainWindow::checkBoxClicked(int i) {
    QCheckBox * cb = checkBoxes[i];
    DrawableObject * obj = mapObjects.left.at(i);
    if (cb->isChecked()) obj->setVisible(true);
    else obj->setVisible(false);
    ui->glCanvas->updateGL();
}

void MainWindow::slotObjectClicked(int i) {
    emit objectClicked(i);
}

/**
 * WIDGETS SLOTS
 */

void MainWindow::setFullScreen(bool b) {
    ui->glCanvas->setFullScreen(b);
}

void MainWindow::setBackgroundColor(const QColor & color) {
    ui->glCanvas->setClearColor(color);
}
