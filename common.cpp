/*
 * @author    Alessandro Muntoni (muntoni.alessandro@gmail.com)
 * @copyright Alessandro Muntoni 2016.
 */

#include "common.h"

int WINDOW_MANAGER_ID;
int DCEL_MANAGER_ID;
int ENGINE_MANAGER_ID;

MainWindow* mw;

Color colorOfNormal(const Vec3& normal) {
    for (unsigned int i = 0; i < XYZ.size(); i++)
        if (Common::epsilonEqual(XYZ[i], normal)) return colors[i];
    return Color(128, 128, 128);
}

Vec3 nearestNormal(const Vec3& normal) {
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

Color colorOfNearestNormal(const Vec3& normal) {
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

int indexOfNormal(const Vec3& v) {
    std::vector<Vec3>::const_iterator it = std::find(XYZ.begin(), XYZ.end(), v);
    if (it != XYZ.end())
        return it - XYZ.begin();
    else
        return -1;
}
