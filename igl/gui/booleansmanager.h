#ifndef BOOLEANSMANAGER_H
#define BOOLEANSMANAGER_H

#include <QFrame>
#include "../../viewer/mainwindow.h"
#include "drawableiglmesh.h"

namespace Ui {
    class BooleansManager;
}

class BooleansManager : public QFrame
{
        Q_OBJECT

    public:
        explicit BooleansManager(QWidget *parent = 0);
        ~BooleansManager();
        void setButtonsMeshLoaded(bool b);
        void setButtonsMeshLoaded_2(bool b);
        void setButtonsResultLoaded(bool b);
        void setMesh1(const IGLInterface::IGLMesh &m);
        void setMesh2(const IGLInterface::IGLMesh &m);

    private slots:
        void on_loadIGLMeshButton_clicked();

        void on_clearIGLMeshButton_clicked();

        void on_saveIGLMeshButton_clicked();

        void on_setFromResultButton_clicked();

        void on_pointsIGLMeshRadioButton_toggled(bool checked);

        void on_flatIGLMeshRadioButton_toggled(bool checked);

        void on_smoothIGLMeshRadioButton_toggled(bool checked);

        void on_wireframeIGLMeshCheckBox_stateChanged(int arg1);

        void on_loadIGLMeshButton_2_clicked();

        void on_clearIGLMeshButton_2_clicked();

        void on_saveIGLMeshButton_2_clicked();

        void on_setFromResultButton_2_clicked();

        void on_pointsIGLMeshRadioButton_2_toggled(bool checked);

        void on_flatIGLMeshRadioButton_2_toggled(bool checked);

        void on_smoothIGLMeshRadioButton_2_toggled(bool checked);

        void on_wireframeIGLMeshCheckBox_2_stateChanged(int arg1);

        void on_intersectionButton_clicked();

        void on_differenceButton_clicked();

        void on_unionButton_clicked();

        void on_clearIGLMeshButton_3_clicked();

        void on_saveIGLMeshButton_3_clicked();

        void on_pointsIGLMeshRadioButton_3_toggled(bool checked);

        void on_flatIGLMeshRadioButton_3_toggled(bool checked);

        void on_smoothIGLMeshRadioButton_3_toggled(bool checked);

        void on_wireframeIGLMeshCheckBox_3_stateChanged(int arg1);

        void on_plusXButton_clicked();

        void on_minusXButton_clicked();

        void on_plusYButton_clicked();

        void on_minusYButton_clicked();

        void on_plusZButton_clicked();

        void on_minusZButton_clicked();

        void on_rotateButton_clicked();

        void on_undoRotateButton_clicked();

    private:
        Ui::BooleansManager *ui;
        MainWindow* mainWindow;
        std::vector<IGLInterface::DrawableIGLMesh*> meshes;
        IGLInterface::DrawableIGLMesh* result;
        Vec3 lastAxis;
        double lastAngle;
};

#endif // BOOLEANSMANAGER_H
