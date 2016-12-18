#include "reconstruction.h"

#include "cgal/aabbtree.h"
#include "common.h"

std::vector<Vec3> Reconstruction::getMapping(const Dcel& smoothedSurface, const HeightfieldsList& he) {
    std::vector<Vec3> mapping(smoothedSurface.getNumberVertices(), Vec3());
    CGALInterface::AABBTree tree(smoothedSurface);
    for (unsigned int i = 0; i < he.getNumHeightfields(); i++){
        const IGLInterface::IGLMesh m = he.getHeightfield(i);
        for (unsigned int j = 0; j < m.getNumberVertices(); j++){
            const Dcel::Vertex* v = tree.getNearestDcelVertex(m.getVertex(j));
            assert(v != nullptr);
            if (v->getCoordinate().dist(m.getVertex(j)) == 0){
                assert(mapping[v->getId()] == Vec3());
                mapping[v->getId()] = he.getTarget(i);
            }
        }
    }
    return mapping;
}

void Reconstruction::saveMappingOnFile(const std::vector<Vec3>& mapping, const std::string& filename) {
    std::ofstream myfile;
    myfile.open (filename);
    myfile << mapping.size() << "\n";
    for (Vec3 target : mapping){
        for (int i  = 0; i < 3; i++) {
            if (target == XYZ[i] || target == XYZ[i+3]) // it happens just one time for every target
                myfile << i << "\n";
        }
    }
    myfile.close();
}

void Reconstruction::reconstruction(Dcel& smoothedSurface, const std::vector<Vec3>& mapping, const IGLInterface::IGLMesh& originalSurface) {
    for (Dcel::Vertex* v : smoothedSurface.vertexIterator()){
        Vec3 target = mapping[v->getId()];
        if (target == XYZ[0] || target == XYZ[3]){
            Pointd p = v->getCoordinate();
            p.x() = originalSurface.getVertex(v->getId()).x();
            v->setCoordinate(p);
        }
        else if (target == XYZ[1] || target == XYZ[4]){
            Pointd p = v->getCoordinate();
            p.y() = originalSurface.getVertex(v->getId()).y();
            v->setCoordinate(p);
        }
        else if (target == XYZ[2] || target == XYZ[5]){
            Pointd p = v->getCoordinate();
            p.z() = originalSurface.getVertex(v->getId()).z();
            v->setCoordinate(p);
        }
        else assert(0);
    }
}
