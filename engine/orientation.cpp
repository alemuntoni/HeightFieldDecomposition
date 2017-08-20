#include "orientation.h"

using namespace std;
using namespace cinolib;
using namespace  cg3;

void Orientation::define_rotation(const cinolib::vec3d& Z_axis, cinolib::vec3d& rot_axis, double& angle) {
    vec3d Z = vec3d(0,0,1);
    rot_axis = Z_axis.cross(Z); rot_axis.normalize();
    angle    = acos(Z_axis.dot(Z));
    assert(!std::isnan(angle));
}


Eigen::Matrix3d Orientation::optimalOrientation(const Dcel& inputMesh, unsigned int n_dirs) {
    Trimesh m;
    MeshConversions::dcelToTrimesh(m, inputMesh);

    std::vector<vec3d> dir_pool;
    sphere_coverage(n_dirs, dir_pool);

    std::set<std::pair<double,vec3d>> priorizited_orientations;
    for(vec3d Z_axis : dir_pool)
    {
        vec3d  axis;
        double angle;
        Z_axis.normalize();
        define_rotation(Z_axis, axis, angle);

        double L1_extent = 0.0;
        for(int tid=0; tid<m.num_triangles(); ++tid)
        {
            vec3d n = m.triangle_normal(tid);
            rotate(n, axis, angle);
            L1_extent += std::fabs(n.x()) + std::fabs(n.y()) + std::fabs(n.z());
        }

        priorizited_orientations.insert(std::make_pair(L1_extent,Z_axis));
    }

    double best_L1 = priorizited_orientations.begin()->first;
    vec3d  best_Z  = priorizited_orientations.begin()->second;

    std::cout << "Best dir: " << best_Z << "\tenergy : " << best_L1 << std::endl;

    // dump samples for visual inspection
    /*std::vector<double> coords;
    std::vector<uint>   dummy;
    for(vec3d s : dir_pool)
    {
        coords.push_back(s.x());
        coords.push_back(s.y());
        coords.push_back(s.z());
    }
    Trimesh samples(coords,dummy);
    samples.save("samples.off");*/

    vec3d  axis;
    double angle;
    best_Z.normalize();
    define_rotation(best_Z, axis, angle);

    return Common::getRotationMatrix(Vec3(axis), angle);
}
