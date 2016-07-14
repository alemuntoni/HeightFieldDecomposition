/**
 @author    Marco Livesu (marco.livesu@gmail.com)
 @copyright Marco Livesu 2014.
*/

#include "load_save_trimesh.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>


//using namespace std;


void loadObj(const char     * filename,
              std::vector<double> & xyz,
              std::vector<int>    & tri)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "ERROR : " << __FILE__ << ", line " << __LINE__ << " : load_OBJ() : couldn't open input file " << filename << std::endl;
        exit(-1);
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        std::string token;
        iss >> token;
        if (token.size() > 1) continue; // vn,fn  .... I don't care

        if (token[0] == 'v')
        {
            double x, y, z;
            iss >> x >> y >> z;
            xyz.push_back(x);
            xyz.push_back(y);
            xyz.push_back(z);
            //cout << "v " << x << " " << y << " " << z << endl;
        }
        else if (token[0] == 'f')
        {
            int v0, v1, v2;
            iss >> v0 >> v1 >> v2;
            tri.push_back(v0-1);
            tri.push_back(v1-1);
            tri.push_back(v2-1);
            //cout << "f " << v0 << " " << v1 << " " << v2 << endl;
        }
    }
    file.close();
}
