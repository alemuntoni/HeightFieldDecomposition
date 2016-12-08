/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef GLCANVAS_H
#define GLCANVAS_H

#include <QGLViewer/qglviewer.h>
#include <QGLViewer/manipulatedCameraFrame.h>
#include <QGLWidget>
#include <QKeyEvent>
#include <vector>

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

#include "../common/bounding_box.h"
#include "interfaces/drawable_object.h"
#include "interfaces/pickable_object.h"
#include <qmessagebox.h>

//using namespace std;

class GLcanvas : public QGLViewer {

        Q_OBJECT

    public:

        GLcanvas(QWidget * parent = nullptr);
        ~GLcanvas();

        void init();
        void draw();
        void drawWithNames();
        void clear();
        void fitScene();
        void setClearColor(const QColor & color);
        BoundingBox getFullBoundingBox();
        int getNumberVisibleObjects();
        void postSelection(const QPoint& point);

        int  pushObj(const DrawableObject * obj, bool visible = true);
        void deleteObj(const DrawableObject* obj);
        void setVisibility(const DrawableObject * obj, bool visible = true);
        bool isVisible(const DrawableObject* obj);

    signals:
        void objectPicked(unsigned int);

    private:

        QColor clearColor;
        std::vector<const DrawableObject *> drawlist;
        std::vector<bool> objVisibility;

        qglviewer::Vec orig, dir, selectedPoint;
};

#endif // GLCANVAS_H
