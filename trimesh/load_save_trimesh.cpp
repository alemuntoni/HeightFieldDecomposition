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

void saveObj(const char *filename, const std::vector<double> &xyz, const std::vector<int> &tri)
{
    std::ofstream fp;
    fp.open (filename);
    fp.precision(6);
    fp.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

    if(!fp)
    {
        std::cerr << "ERROR : " << __FILE__ << ", line " << __LINE__ << " : save_OBJ() : couldn't open output file " << filename << std::endl;
        exit(-1);
    }

    //cout << "v " << xyz[3] << " " << xyz[4] << " " << xyz[5] << endl;

    for(int i=0; i<(int)xyz.size(); i+=3)
    {
        fp << "v " << xyz[i] << " " << xyz[i+1] << " " << xyz[i+2] << std::endl;
    }

    for(int i=0; i<(int)tri.size(); i+=3)
    {
        fp << "f " << tri[i]+1 << " " << tri[i+1]+1 << " " << tri[i+2]+1 << std::endl;
    }

    fp.close();
}
