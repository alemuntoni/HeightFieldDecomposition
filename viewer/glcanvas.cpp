/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include <iostream>

#include "glcanvas.h"

GLcanvas::GLcanvas(QWidget * parent) : clearColor(Qt::white) {
    setParent(parent);
}

GLcanvas::~GLcanvas() {
}

void GLcanvas::init() {
    setFPSIsDisplayed(true);
    camera()->frame()->setSpinningSensitivity(100.0);
}

void GLcanvas::clear() {
    drawlist.clear();
}

/*********************************Andreas***************************************/
void GLcanvas::postSelection(const QPoint& point)
{
  // Find the selectedPoint coordinates, using camera()->pointUnderPixel().
  bool found;
  selectedPoint = camera()->pointUnderPixel(point, found);
  selectedPoint -= 0.01f*dir; // Small offset to make point clearly visible.
  unsigned int idObject = selectedName();
  // Note that "found" is different from (selectedObjectId()>=0) because of the size of the select region.

  if ((int) idObject == -1)
    QMessageBox::information(this, "No selection",
                 "No object selected under pixel " + QString::number(point.x()) + "," + QString::number(point.y()));
  else
      for(int i=0; i<(int)drawlist.size(); ++i){
          const PickableObject* po = dynamic_cast<const PickableObject*>(drawlist[i]);
          if (po) { // se è un PickableObject, allora faccio la emit sull'object!
              emit objectPicked(idObject);
              updateGL();
          }
      }
}

void GLcanvas::drawWithNames(){
    setBackgroundColor(clearColor);

    for(int i=0; i<(int)drawlist.size(); ++i){
        if ((drawlist[i])->isVisible()){
            const PickableObject* obj = dynamic_cast<const PickableObject*>(drawlist[i]);
            if (obj) // se il drawable object è anche un pickable object, allora chiamo la draw with names
                obj->drawWithNames();
        }
    }
}
/*******************************************************************************/

void GLcanvas::draw() {
    setBackgroundColor(clearColor);

    for(int i=0; i<(int)drawlist.size(); ++i) {
        drawlist[i]->draw();
    }
}

int GLcanvas::pushObj(const DrawableObject* obj) {
    drawlist.push_back(obj);
    updateGL();

    return drawlist.size();
}

void GLcanvas::deleteObj(const DrawableObject* obj) {
    std::vector<const DrawableObject *>::iterator it = std::find(drawlist.begin(), drawlist.end(), obj);
    if (it != drawlist.end()) {
        drawlist.erase(it);
    }
}

void GLcanvas::fitScene() {
    Pointd center(0,0,0);
    double radius = 0.0;
    int   count  = 0;

    if (getNumberVisibleObjects() == 0) radius = 1.0;
    else {
        for(int i=0; i<(int)drawlist.size(); ++i) {
            const DrawableObject * obj = drawlist[i];
            if (obj->isVisible() && obj->sceneRadius() > 0) {
                center += obj->sceneCenter();
                radius = std::max(radius, obj->sceneRadius());
                ++count;
            }
        }

        center /= (double)count;
    }

    setSceneCenter(qglviewer::Vec(center.x(), center.y(), center.z()));
    setSceneRadius(radius);

    showEntireScene();       
}

void GLcanvas::setClearColor(const QColor &color) {
    clearColor = color;
    updateGL();
}

BoundingBox GLcanvas::getFullBoundingBox() {
    BoundingBox bb;
    for(int i=0; i<(int)drawlist.size(); ++i) {
        const DrawableObject * obj = drawlist[i];
        if (obj->isVisible()) {
            Pointd center = obj->sceneCenter();
            bb.setMin(bb.getMin().min(Pointd(center.x() - obj->sceneRadius(), center.y() - obj->sceneRadius(), center.z() - obj->sceneRadius())));
            bb.setMax(bb.getMax().max(Pointd(center.x() + obj->sceneRadius(), center.y() + obj->sceneRadius(), center.z() + obj->sceneRadius())));
        }
    }
    return bb;
}

int GLcanvas::getNumberVisibleObjects() {
    int count = 0;
    for(int i=0; i<(int)drawlist.size(); ++i) {
        const DrawableObject * obj = drawlist[i];
        if (obj->isVisible() && obj->sceneRadius() > 0) count++;
    }
    return count;
}
