#ifndef MESHCONVERSIONS_H
#define MESHCONVERSIONS_H

#include <cinolib/meshes/trimesh/trimesh.h>

#ifdef  CG3_EIGENMESH_DEFINED
#include <cg3/meshes/eigenmesh/eigenmesh.h>
#endif //EIGENMESH_DEFINED

#ifdef  CG3_DCEL_DEFINED
#include <cg3/meshes/dcel/dcel.h>
#endif //DCEL_DEFINED

namespace cg3 {

namespace MeshConversions {
    #ifdef  CG3_EIGENMESH_DEFINED
    void eigenMeshToTrimesh(cinolib::Trimesh& m, const SimpleEigenMesh& simpleEigenMesh);
    #endif //EIGENMESH_DEFINED

    #ifdef  CG3_DCEL_DEFINED
    void dcelToTrimesh(cinolib::Trimesh& m, const Dcel &d);
    #endif //DCEL_DEFINED
}

}

#endif // MESHCONVERSIONS_H
