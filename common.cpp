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

std::string exec(const char* cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}
