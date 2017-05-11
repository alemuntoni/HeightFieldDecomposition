#include "greedysubdivision.h"

BipartiteGraph<const Dcel::Face*, Box3D> GreedySubdivision::getBipartiteGraph(const BoxList& bl, const Dcel& d) {
    BipartiteGraph<const Dcel::Face*, Box3D> bigraph;

    for (const Dcel::Face* f : d.faceIterator()) {
        bigraph.addUNode(f);
    }
    for (Box3D b : bl) {
        bigraph.addVNode(b);
    }
    CGALInterface::AABBTree tree(d);
    for (Box3D b : bl) {
        std::list<const Dcel::Face*> list = tree.getCompletelyContainedDcelFaces(b);
        for (const Dcel::Face* f : list){
            bigraph.addArc(f, b);
        }
    }

    return bigraph;
}

std::set<Box3D> GreedySubdivision::getMandatoryBoxesToCompute(const BipartiteGraph<const Dcel::Face*, Box3D>& bigraph) {
    std::vector<const Dcel::Face*> vf;
    vf.reserve(bigraph.sizeU());
    for (const Dcel::Face* f : bigraph.uNodeIterator()){
        vf.push_back(f);
    }
    struct cmp {
            const BipartiteGraph<const Dcel::Face*, Box3D> &bip;
            cmp(const BipartiteGraph<const Dcel::Face*, Box3D> &bip) : bip(bip) {}
            bool operator ()(const Dcel::Face* f1, const Dcel::Face* f2) const{
                return bip.sizeAdjacencesUNode(f1) < bip.sizeAdjacencesUNode(f2);
            }
    };
    std::sort(vf.begin(), vf.end(), cmp(bigraph));
    // vf contains triangles sorted by # of boxes wich contain that triangle
    std::set<Box3D> firstBoxes;
    // look just on triangles contained by only one box
    for (unsigned int i = 0; i < vf.size() && bigraph.sizeAdjacencesUNode(vf[i]) == 1; i++){
        firstBoxes.insert((*(bigraph.adjacentUNodeBegin(vf[i]))));
    }
    return firstBoxes;
}


