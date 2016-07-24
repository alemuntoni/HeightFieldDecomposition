#ifndef IGLMESHMANAGER_H
#define IGLMESHMANAGER_H

#include <QFrame>
#include "viewer/mainwindow.h"
#include "drawableiglmesh.h"

namespace Ui {
    class IGLMeshManager;
}

class IGLMeshManager : public QFrame
{
        Q_OBJECT

    public:
        explicit IGLMeshManager(QWidget *parent = 0);
        void setButtonsMeshLoaded(bool b);
        void setIGLMesh(const IGLMesh &m);
        ~IGLMeshManager();

    private slots:
        void on_loadIGLMeshButton_clicked();

        void on_clearIGLMeshButton_clicked();

        void on_saveIGLMeshButton_clicked();

        void on_pointsIGLMeshRadioButton_toggled(bool checked);

        void on_flatIGLMeshRadioButton_toggled(bool checked);

        void on_smoothIGLMeshRadioButton_toggled(bool checked);

        void on_wireframeIGLMeshCheckBox_stateChanged(int arg1);

    private:
        Ui::IGLMeshManager *ui;
        MainWindow* mainWindow;
        DrawableIGLMesh* mesh;


};

#endif // IGLMESHMANAGER_H
