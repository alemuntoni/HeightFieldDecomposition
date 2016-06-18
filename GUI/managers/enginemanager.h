#ifndef ENGINEMANAGER_H
#define ENGINEMANAGER_H

#include <QFrame>
#include "dcelmanager.h"
#include "lib/grid/drawablegrid.h"
#include "common.h"

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

        void on_distanceSpinBox_valueChanged(double arg1);

        void on_targetComboBox_currentIndexChanged(int index);

        void on_kernelRadioButton_toggled(bool checked);

        void on_weigthsRadioButton_toggled(bool checked);

        void on_freezeKernelPushButton_clicked();

        void on_sliceCheckBox_stateChanged(int arg1);

        void on_sliceSlider_valueChanged(int value);

        void on_sliceComboBox_currentIndexChanged(int index);

    private:
        Ui::EngineManager *ui;
        MainWindow* mainWindow; //puntatore alla mainWindow
        DrawableGrid* g;
};

#endif // ENGINEMANAGER_H
