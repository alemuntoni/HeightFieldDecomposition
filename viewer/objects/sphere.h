/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author    Gianmarco Cherchi (gianmarcher@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef SPHERE_H
#define SPHERE_H

#ifdef __APPLE__
#include <gl.h>
#include <glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <QColor>
#include "../../common/point.h"

inline void sphere(const Pointd  & center,
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

#endif // SPHERE_H
