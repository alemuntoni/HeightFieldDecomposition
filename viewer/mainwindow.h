/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDockWidget>
#include <QToolBox>
#include <QFrame>
#include <QSignalMapper>
#include <QCheckBox>
#include <boost/bimap.hpp>
#include <QApplication>

#include "../common/drawable_object.h"
#include "../common/bounding_box.h"
#include "objects/drawabledebugobjects.h"

namespace Ui {
    class MainWindow;
}

/**
 * @brief La classe MainWindow è una classe che gestisce la canvas di QGLViewer e tutti i manager che vengono aggiunti ad essa.
 *
 * Gestisce in oltre una scrollArea avente le checkbox che gestiscono la visualizzazione di tutti gli oggetti presenti nella canvas.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

    public:

        explicit MainWindow(QWidget *parent = 0);
        ~MainWindow();

        void updateGlCanvas();
        void pushObj(DrawableObject * obj, std::string checkBoxName, bool b = true);
        void deleteObj(DrawableObject * obj, bool b = true);
        BoundingBox getFullBoundingBox();
        int getNumberVisibleObjects();
        void saveSnapshot();
        void drawAxis(bool);

        void setFullScreen(bool);
        void setBackgroundColor(const QColor &);

        int addManager(QFrame *f, std::string name, QToolBox *parent = nullptr);
        QFrame *getManager(unsigned int i);
        void renameManager(unsigned int i, std::string s);
        void setCurrentIndexToolBox(unsigned int i);

        void enableDebugObjects();
        void disableDebugObjects();
        void addDebugSphere(const Pointd& center, double radius, const QColor &color, int precision = 4);
        void clearDebugSpheres();
        void addDebugCylinder(const Pointd& a, const Pointd& b, double radius, const QColor color);
        void clearDebugCylinders();

    signals:
        /**
         * @brief objectClicked
         * Segnale da "catchare", ha come parametro l'oggetto cliccato nella GLCanvas
         */
        void objectClicked(int);

    private slots:
        void checkBoxClicked(int i);
        void slotObjectClicked(int i);

    private:

        // GUI
        //
        Ui::MainWindow  * ui;
        std::vector<QFrame *> managers;

        // Mesh Stack
        //
        QSignalMapper* checkBoxMapper;
        std::map<int, QCheckBox * > checkBoxes;
        boost::bimap<int, DrawableObject*> mapObjects;
        int nMeshes;
        bool first;
        DrawableDebugObjects* debugObjects;
};

#endif // MAINWINDOW_H
