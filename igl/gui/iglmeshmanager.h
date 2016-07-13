#ifndef IGLMESHMANAGER_H
#define IGLMESHMANAGER_H

#include <QFrame>
#include "viewer/mainwindow.h"
#include "../iglmesh.h"

namespace Ui {
    class IGLMeshManager;
}

class IGLMeshManager : public QFrame
{
        Q_OBJECT

    public:
        explicit IGLMeshManager(QWidget *parent = 0);
        ~IGLMeshManager();

    private:
        Ui::IGLMeshManager *ui;
        MainWindow* mainWindow;

};

#endif // IGLMESHMANAGER_H
