#include "enginemanager.h"
#include "ui_enginemanager.h"
#include "common.h"
#include <cstdio>

EngineManager::EngineManager(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::EngineManager),
    mainWindow((MainWindow*)parent)
{
    ui->setupUi(this);
}

EngineManager::~EngineManager()
{
    delete ui;
}

void EngineManager::on_generateGridPushButton_clicked() {
    DcelManager* dm = (DcelManager*)mainWindow->getManager(DCEL_MANAGER_ID);
    DrawableDcel* d = dm->getDcel();
    int s = 50;
    if (d != nullptr){
        BoundingBox bb = d->getBoundingBox();
        double maxl = std::max(bb.getMaxX() - bb.getMinX(), bb.getMaxY() - bb.getMinY());
        maxl = std::max(maxl, bb.getMaxZ() - bb.getMinZ());
        double av = maxl / s;
        BoundingBox nBB(-(bb.getMax()-bb.getMin())/av, (bb.getMax()-bb.getMin())/av);

        d->scale(nBB);
        d->update();
        d->saveOnObjFile("tmp.obj");
        exec("./grid_generator tmp.obj");


        Eigen::RowVector3i nGmin;
        Eigen::RowVector3i nGmax;
        Eigen::RowVector3i res;
        Eigen::VectorXd S;
        Eigen::MatrixXd GV;

        std::ifstream file;
        file.open ("tmp.bin", std::ios::in | std::ios::binary);
        Serializer::deserialize(nGmin, file);
        Serializer::deserialize(nGmax, file);
        Serializer::deserialize(res, file);
        Serializer::deserialize(GV, file);
        Serializer::deserialize(S, file);
        file.close();


        mainWindow->updateGlCanvas();

        mainWindow->enableDebugObjects();
        for (int i = 0; i < res(0); i++){
            for (int j = 0; j < res(1); j++){
                for (int k= 0; k < res(2); k++){
                    Pointd p(GV.row(i+res(0)*(j + res(1)*k))(0), GV.row(i+res(0)*(j + res(1)*k))(1), GV.row(i+res(0)*(j + res(1)*k))(2));
                    if (S(i+res(0)*(j + res(1)*k)) < -3) mainWindow->addDebugSphere(p, 0.5, QColor(255,0,0));
                    //else mainWindow->addDebugSphere(p, 0.5, QColor(0,0,255));
                }
            }
        }
        mainWindow->updateGlCanvas();

        std::remove("tmp.bin");
        std::remove("tmp.obj");
        std::remove("tmp.mtu");

    }
}
