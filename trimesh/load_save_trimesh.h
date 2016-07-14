/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#ifndef LOAD_SAVE_TRIMESH_H
#define LOAD_SAVE_TRIMESH_H

#include <vector>

// OBJ FILES
//
void loadObj(const char          * filename,
              std::vector<double> & xyz,
              std::vector<int>    & tri);

#endif // LOAD_SAVE_TRIMESH_H
