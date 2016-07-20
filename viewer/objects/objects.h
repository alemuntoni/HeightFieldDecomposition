#ifndef OBJECTS_H
#define OBJECTS_H

#ifdef __APPLE__
#include <gl.h>
#include <glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <cmath>
#include <QColor>
#include "../../common/point.h"

static inline void drawSphere(const Pointd  & center,
                   float         radius,
                   const QColor& color,
                   int precision = 4) {
    glEnable(GL_LIGHTING);
    glShadeModel(GL_SMOOTH);
    glColor3f(color.redF(), color.greenF(), color.blueF());
    glPushMatrix();
    glTranslated(center.x(), center.y(), center.z());
    GLUquadric *sphere = gluNewQuadric();
    gluQuadricNormals(sphere, GLU_SMOOTH);
    gluQuadricOrientation(sphere, GLU_OUTSIDE);
    gluSphere(sphere, radius, precision, precision);
    glPopMatrix();

}

static inline void drawCylinder(const Pointd  & a,
                     const Pointd  & b,
                     float         top_radius,
                     float         bottom_radius,
                     const QColor& color) {
    Pointd dir     = b - a; dir.normalize();
    Pointd z       = Pointd(0,0,1);
    Vec3 normal  = dir.cross(z);
    double angle = acos(dir.dot(z)) * 180 / M_PI;

    glColor3f(color.redF(), color.greenF(), color.blueF());
    glPushMatrix();
    glTranslated(a.x(), a.y(), a.z());
    glRotatef(-angle, normal.x(), normal.y(), normal.z());
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glPolygonMode(GL_FRONT, GL_FILL);
    GLUquadric *cylinder = gluNewQuadric();
    gluQuadricNormals(cylinder, GLU_SMOOTH);
    gluQuadricOrientation(cylinder, GLU_OUTSIDE);
    gluCylinder(cylinder, top_radius, bottom_radius, (a-b).getLength(), 500, 100);
    glPopMatrix();
    glDisable(GL_CULL_FACE);
}

static inline void drawLine(const Pointd &a, const Pointd &b, const QColor& c, int width = 3) {
    glBegin(GL_LINES);
    glColor3f(c.redF(), c.greenF(), c.blueF());
    glLineWidth(width);
    glVertex3f(a.x(), a.y(), a.z());
    glVertex3f(b.x(), b.y(), b.z());
    glEnd();
}

static inline void drawBox(const std::vector<Pointd> &p, const QColor& c, int width = 3) {
    drawLine(p[0], p[1], c, width);
    drawLine(p[1], p[2], c, width);
    drawLine(p[2], p[3], c, width);
    drawLine(p[0], p[3], c, width);

    drawLine(p[4], p[5], c, width);
    drawLine(p[5], p[6], c, width);
    drawLine(p[6], p[7], c, width);
    drawLine(p[4], p[7], c, width);

    drawLine(p[0], p[4], c, width);
    drawLine(p[1], p[5], c, width);
    drawLine(p[2], p[6], c, width);
    drawLine(p[3], p[7], c, width);
}

#endif // OBJECTS_H
