/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "common.h"

int DCEL_MANAGER_ID;
int ENGINE_MANAGER_ID;

cg3::viewer::MainWindow* mw;

using namespace cg3;

Color colorOfNormal(const Vec3d& normal) {
    for (unsigned int i = 0; i < XYZ.size(); i++)
        if (epsilonEqual(XYZ[i], normal)) return colors[i];
    return Color(128, 128, 128);
}

Vec3d nearestNormal(const Vec3d& normal) {
    double dot = -1;
    int nid = 0;
    for (unsigned int i = 0; i < 6; i++){
        if (normal.dot(XYZ[i]) > dot){
            nid = i;
            dot = normal.dot(XYZ[i]);
        }
    }
    return XYZ[nid];
}

Color colorOfNearestNormal(const Vec3d& normal) {
    double dot = -1;
    int nid = 0;
    for (unsigned int i = 0; i < XYZ.size(); i++){
        if (normal.dot(XYZ[i]) > dot){
            nid = i;
            dot = normal.dot(XYZ[i]);
        }
    }
    return colors[nid];
}

int indexOfNormal(const Vec3d& v) {
	std::vector<Vec3d>::const_iterator it = std::find(XYZ.begin(), XYZ.end(), v);
    if (it != XYZ.end())
        return it - XYZ.begin();
    else
        return -1;
}
