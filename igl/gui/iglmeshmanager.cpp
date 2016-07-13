#include "iglmeshmanager.h"
#include "ui_iglmeshmanager.h"

IGLMeshManager::IGLMeshManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::IGLMeshManager),
    mainWindow((MainWindow*)parent) {
    ui->setupUi(this);
}

IGLMeshManager::~IGLMeshManager()
{
    delete ui;
}
