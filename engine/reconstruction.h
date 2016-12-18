#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include "heightfieldslist.h"

namespace Reconstruction {
    std::vector<Vec3> getMapping(const Dcel &smoothedSurface, const HeightfieldsList &he);
    void saveMappingOnFile(const std::vector<Vec3> &mapping, const std::string &filename);

    void reconstruction(Dcel &smoothedSurface, const std::vector<Vec3> &mapping, const IGLInterface::IGLMesh &originalSurface);

}

#endif // RECONSTRUCTION_H
