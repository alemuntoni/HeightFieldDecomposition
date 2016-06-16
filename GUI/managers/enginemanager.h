#ifndef ENGINEMANAGER_H
#define ENGINEMANAGER_H

#include <QFrame>
#include "dcelmanager.h"

namespace Ui {
    class EngineManager;
}

class EngineManager : public QFrame
{
        Q_OBJECT

    public:
        explicit EngineManager(QWidget *parent = 0);
        ~EngineManager();

    private slots:
        void on_generateGridPushButton_clicked();

    private:
        Ui::EngineManager *ui;
        MainWindow* mainWindow; //puntatore alla mainWindow
};

#endif // ENGINEMANAGER_H
