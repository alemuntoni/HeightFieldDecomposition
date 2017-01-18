#ifndef LOAD_SAVE_FILE_H
#define LOAD_SAVE_FILE_H

#include <string>

namespace Common {
    void saveTriangleMeshOnObj(const std::string &filename, size_t nVertices, size_t nFaces, const double vertices[], const int faces[]);

}

#endif // LOAD_SAVE_FILE_H
