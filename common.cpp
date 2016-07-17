/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "common.h"

int WINDOW_MANAGER_ID;
int DCEL_MANAGER_ID;
int ENGINE_MANAGER_ID;

QColor colorOfNormal(const Vec3 &normal) {
    for (unsigned int i = 0; i < XYZ.size(); i++)
        if (epsilonEqual(XYZ[i], normal)) return colors[i];
    return QColor();
}
