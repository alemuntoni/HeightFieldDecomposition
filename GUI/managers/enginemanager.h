#ifndef ENGINEMANAGER_H
#define ENGINEMANAGER_H

#include <QFrame>
#include <QLabel>
#include "dcel/gui/dcelmanager.h"
#include "lib/grid/drawablegrid.h"
#include "common.h"
#include "engine/box.h"
#include "engine/energy.h"
#include "engine/engine.h"
#include "common/timer.h"

namespace Ui {
    class EngineManager;
}

class EngineManager : public QFrame, public SerializableObject {
        Q_OBJECT

    public:
        explicit EngineManager(QWidget *parent = 0);

        void deleteDrawableObject(DrawableObject* d);
        ~EngineManager();

        void updateLabel(double value, QLabel* label);

        // SerializableObject interface
        void serialize(std::ofstream& binaryFile) const;
        void deserialize(std::ifstream& binaryFile);

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

        void on_serializePushButton_clicked();

        void on_deserializePushButton_clicked();

        void on_wSpinBox_valueChanged(double arg1);

        void on_hSpinBox_valueChanged(double arg1);

        void on_dSpinBox_valueChanged(double arg1);

        void on_plusXButton_clicked();

        void on_minusXButton_clicked();

        void on_plusYButton_clicked();

        void on_minusYButton_clicked();

        void on_plusZButton_clicked();

        void on_minusZButton_clicked();

        void on_energyBoxPushButton_clicked();

        void on_minimizePushButton_clicked();

        void on_serializeBoxPushButton_clicked();

        void on_deserializeBoxPushButton_clicked();

        void on_iterationsSlider_sliderMoved(int position);

        void on_energyIterationsButton_clicked();

        void on_createBoxesPushButton_clicked();

        void on_showAllSolutionsCheckBox_stateChanged(int arg1);

        void on_solutionsSlider_valueChanged(int value);

        void on_minimizeAllPushButton_clicked();

        void on_setFromSolutionButton_clicked();

        void on_wireframeDcelCheckBox_stateChanged(int arg1);

        void on_pointsDcelRadioButton_toggled(bool checked);

        void on_flatDcelRadioButton_toggled(bool checked);

        void on_smoothDcelRadioButton_toggled(bool checked);

        void on_trianglesCoveredPushButton_clicked();

        void on_deleteBoxesPushButton_clicked();

        void on_deserializePreprocessingPushButton_clicked();

        void on_serializePreprocessingPushButton_clicked();

        void on_stepDrawGridSpinBox_valueChanged(double arg1);

    private:
        Ui::EngineManager *ui;
        MainWindow* mainWindow; //puntatore alla mainWindow
        DrawableGrid* g;
        DrawableDcel* d;
        Box3D* b;
        BoxList* iterations;
        BoxList* solutions;
        Energy e;
};

#endif // ENGINEMANAGER_H
