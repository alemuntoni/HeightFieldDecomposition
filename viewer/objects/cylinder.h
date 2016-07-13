/*
 * @author    Marco Livesu (marco.livesu@gmail.com)
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @author    Gianmarco Cherchi (gianmarcher@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#ifndef CYLINDER_H
#define CYLINDER_H

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

inline void cylinder(const Pointd  & a,
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

#endif // CYLINDER_H
