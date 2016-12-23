#include "splitting.h"

#include "common.h"
#include "common/timer.h"
#include "lib/graph/directedgraph.h"
#include "igl/utils.h"
#include <map>

bool Splitting::boxesIntersect(const Box3D& b1, const Box3D& b2) {
    if (b1.getMaxX() <= b2.getMinX()) return false; // a is left of b
    if (b1.getMinX() >= b2.getMaxX()) return false; // a is right of b
    if (b1.getMaxY() <= b2.getMinY()) return false; // a is above b
    if (b1.getMinY() >= b2.getMaxY()) return false; // a is below b
    if (b1.getMaxZ() <= b2.getMinZ()) return false; // a is behind b
    if (b1.getMinZ() >= b2.getMaxZ()) return false; // a is in front b
    return true; //boxes overlap
}

bool Splitting::isDangerousIntersection(const Box3D& b1, const Box3D& b2, const CGALInterface::AABBTree &tree, bool checkMeshes) {
    Vec3 target2 = b2.getTarget();
    BoundingBox bb = b1;
    for (unsigned int t = 0; t < 6; t++){
        if (t < 3){
            if (target2 == XYZ[t]){ //+x
                if (b1.max()[t] < b2.max()[t]){
                    bool b = false;
                    for (unsigned int i = 0; i < 3; i++){
                        if (i != t){
                            if (b1.min()[i] > b2.min()[i])
                                b = true;
                            if (b1.max()[i] < b2.max()[i])
                                b = true;
                        }
                    }
                    if (b){
                        //check if bb is empty
                        bb.min()[t]= b1.max()[t]-EPSILON;
                        for (unsigned int i = 0; i < 3; i++){
                            if (i != t){
                                bb.min()[i] = std::max(b1.min()[i], b2.min()[i]);
                                bb.max()[i] = std::min(b1.max()[i], b2.max()[i]);
                                assert(bb.min()[i] < bb.max()[i]);
                            }
                        }
                        bool isInside = false;
                        std::vector<Pointd> extremes;
                        bb.getExtremes(extremes);
                        for (unsigned int i = 0; i < extremes.size() && !isInside; i++){
                            if (tree.isInside(extremes[i]))
                                isInside = true;
                        }
                        if(isInside || tree.getNumberIntersectedPrimitives(bb) > 0){
                        //if (tree.getNumberIntersectedPrimitives(bb) > 0){
                            if (!checkMeshes){
                                return true;
                            }
                            else {
                                IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                                if (intersection.getNumberVertices() != 0){
                                    ////
                                    //intersection.saveOnObj("int.obj");
                                    ////
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        else {
            if (target2 == XYZ[t]){ //-x
                unsigned int ot = t-3;
                if(b1.min()[ot] > b2.min()[ot]){
                    bool b = false;
                    for (unsigned int i = 0; i < 3; i++){
                        if (i != ot){
                            if (b1.min()[i] > b2.min()[i])
                                b = true;
                            if (b1.max()[i] < b2.max()[i])
                                b = true;
                        }
                    }
                    if (b){
                        //check if bb is empty
                        bb.max()[ot] = (b1.min()[ot]+EPSILON);
                        for (unsigned int i = 0; i < 3; i++){
                            if (i != ot){
                                bb.min()[i] = std::max(b1.min()[i], b2.min()[i]);
                                bb.max()[i] = std::min(b1.max()[i], b2.max()[i]);
                                assert(bb.min()[i] < bb.max()[i]);
                            }
                        }
                        ///
                        IGLInterface::makeBox(bb).saveOnObj("bb.obj");
                        ///
                        bool isInside = false;
                        std::vector<Pointd> extremes;
                        bb.getExtremes(extremes);
                        for (unsigned int i = 0; i < extremes.size() && !isInside; i++){
                            if (tree.isInside(extremes[i]))
                                isInside = true;
                        }
                        if (isInside  || tree.getNumberIntersectedPrimitives(bb) > 0){
                        //if (tree.getNumberIntersectedPrimitives(bb) > 0){
                            if (!checkMeshes){
                                return true;
                            }
                            else {
                                IGLInterface::SimpleIGLMesh intersection = IGLInterface::SimpleIGLMesh::intersection(b1.getIGLMesh(), b2.getIGLMesh());
                                if (intersection.getNumberVertices() != 0){
                                    ////
                                    //intersection.saveOnObj("int.obj");
                                    ////
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

double Splitting::getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3) {
    Box3D b4;
    b3.setColor(b2.getColor());
    b3.setTarget(b2.getTarget());
    Vec3 target = b2.getTarget();
    for (unsigned int t = 0; t < 6; t++){
        if (target == XYZ[t]){
            if (t < 3){
                assert(b1.max()[t] < b2.max()[t]);
                b3.min()[t] = b1.max()[t];
                b3.max()[t] = b2.max()[t];
                b4.min()[t] = b2.min()[t];
                b4.max()[t] = b1.max()[t];
                for (unsigned u = 0; u < 3; u++){
                    if (u != t){
                        b3.min()[u] = b4.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                        b3.max()[u] = b4.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                        assert(b3.min()[u] < b3.max()[u]);
                    }
                }
            }
            else if (t >= 3){
                unsigned int i = t-3;
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                assert(b1.min()[i] > b2.min()[i]);
                b3.min()[i] = b2.min()[i];
                b3.max()[i] = b1.min()[i];
                b4.min()[i] = b1.min()[i];
                b4.max()[i] = b2.max()[i];
                for (unsigned int u = 0; u < 3; u++){
                    if (u != i){
                        b3.min()[u] = b4.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                        b3.max()[u] = b4.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                        assert(b3.min()[u] < b3.max()[u]);
                    }
                }

            }
        }
    }
    return b4.getVolume();
}

void Splitting::splitBox(const Box3D& b1, Box3D& b2, Box3D & b3, double subd) {
    getSplits(b1, b2, b3);
    //
    Box3D b3tmp = b3;
    for (unsigned int i = 0; i < 6; i++){
        if (b3tmp[i] == b2[i]){
            if (i < 3)
                b3tmp[i]-=EPSILON;
            else
                b3tmp[i]+=EPSILON;
        }
    }
    b3tmp.generatePiece(subd);
    //
    b3.generatePiece(subd);
    IGLInterface::SimpleIGLMesh oldBox = b2.getIGLMesh();
    oldBox = IGLInterface::SimpleIGLMesh::difference(oldBox, b1.getIGLMesh());
    IGLInterface::SimpleIGLMesh tmp = oldBox;
    tmp = IGLInterface::SimpleIGLMesh::intersection(tmp, b3.getIGLMesh());
    if (tmp.getNumberVertices() > 0){
        oldBox = IGLInterface::SimpleIGLMesh::difference(oldBox, b3tmp.getIGLMesh());
        b2.setIGLMesh(oldBox);
        BoundingBox newBBb2 = oldBox.getBoundingBox();
        b2.setMin(newBBb2.min());
        b2.setMax(newBBb2.max());
        b3.setIGLMesh(tmp);
        BoundingBox newBBb3 = tmp.getBoundingBox();
        b3.setMin(newBBb3.min());
        b3.setMax(newBBb3.max());
    }
    else
        b3.min() = b3.max() = Pointd();
}

double Splitting::minimumSplit(const Box3D &b1, const Box3D &b2){
    Box3D b3;
    double volumeb4 = getSplits(b1, b2, b3);
    double volumeb2 = b2.getVolume();
    double volumeb3 = b3.getVolume();
    double remainingSplit = volumeb2 - volumeb3 - volumeb4;
    assert (remainingSplit >= 0);
    return std::min(remainingSplit, volumeb3);
}

Array2D<int> Splitting::getOrdering(BoxList& bl, const Dcel& d) {
    CGALInterface::AABBTree tree(d);
    bl.calculateTrianglesCovered(tree);
    bl.sortByTrianglesCovered();
    bl.setIds();
    std::cerr << "Triangles Covered: \n";
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::cerr << bl.getBox(i).getTrianglesCovered() << "; ";
    }
    std::cerr << "\n";
    std::vector<std::vector<unsigned int> > loops;
    DirectedGraph g(bl.getNumberBoxes());
    //
    //d.saveOnObjFile("tmp.obj");
    //
    for (unsigned int i = 0; i < bl.getNumberBoxes()-1; i++){
        Box3D b1 = bl.getBox(i);
        for (unsigned int j = i+1; j < bl.getNumberBoxes(); j++){
            Box3D b2 = bl.getBox(j);
            if (boxesIntersect(b1,b2)){
                //
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                //
                if (isDangerousIntersection(b1, b2, tree)){
                    g.addEdge(i,j);
                    std::cerr << i << " -> " << j << "\n";
                }
                if (isDangerousIntersection(b2, b1, tree)){
                    g.addEdge(j,i);
                    std::cerr << j << " -> " << i << "\n";
                }
            }
        }
    }
    ///Detect and delete cycles on graph (modifying bl)

    do {
        g.getLoops(loops);
        std::cerr << "Number loops: " << loops.size() << "\n";
        if (loops.size() > 0){ // I need to modify bl

            std::map<std::pair<unsigned int, unsigned int>, int> arcs;
            for (std::vector<unsigned int> loop : loops){
                unsigned int node = 0;
                for (node = 0; node < loop.size()-1; node++){
                    std::cerr << loop[node] << " -> ";
                    std::pair<unsigned int, unsigned int> arc(loop[node], loop[node+1]);
                    std::map<std::pair<unsigned int, unsigned int>, int>::iterator it = arcs.find(arc);
                    if (it == arcs.end())
                        arcs[arc] = 1;
                    else
                        arcs[arc]++;
                }
                std::cerr << loop[node] << " \n";
            }
            std::multimap<int,std::pair<unsigned int, unsigned int> > rev = Common::flipMap(arcs);

            for (std::multimap<int,std::pair<unsigned int, unsigned int> >::reverse_iterator it = rev.rbegin(); it != rev.rend(); ++it){
                std::cerr << "K : " << (*it).first << "; V: (" << (*it).second.first << " -> " << (*it).second.second << ")\n";
            }

            std::multimap<int,std::pair<unsigned int, unsigned int> >::reverse_iterator it = rev.rbegin();

            unsigned int size = rev.count((*it).first); // number of arcs that belongs with the maximum number of loops
            std::pair<unsigned int, unsigned int> arcToRemove;
            if (size == 1){ // if I remove this arc, I will remove the maximum number of loops
                arcToRemove = (*it).second;
                std::cerr << "Eliminate Arc: " << arcToRemove.first << " -> " << arcToRemove.second << "\n";
            }
            else { // I need to choose wich arc I want to eliminate -> maximum split
                std::cerr << "Eliminate Arcs: \n";
                std::vector<std::pair<unsigned int, unsigned int> > candidateArcs;
                for (auto i=rev.equal_range((*it).first).first; i!=rev.equal_range((*it).first).second; ++i){
                    std::pair<unsigned int, unsigned int> arc = (*i).second;
                    std::cerr << "\t" << arc.first << " -> " << arc.second << "\n";
                    candidateArcs.push_back(arc);
                }
                double volmax = minimumSplit(bl.getBox(candidateArcs[0].first), bl.getBox(candidateArcs[0].second));
                int maxarc = 0;

                for (unsigned int i = 1; i < candidateArcs.size(); i++) {
                    std::pair<unsigned int, unsigned int> arc = candidateArcs[i];
                    double tmp = minimumSplit(bl.getBox(arc.first), bl.getBox(arc.second));
                    if (tmp > volmax){
                        volmax = tmp;
                        maxarc = i;
                    }
                }
                arcToRemove = candidateArcs[maxarc];
            }
            // now I can remove "arcToRemove"
            Box3D b1 = bl.getBox(arcToRemove.first), b2 = bl.getBox(arcToRemove.second), b3;
            splitBox(b1, b2, b3, d.getAverageHalfEdgesLength()*7);
            if (!(b3.min() == Pointd() && b3.max() == Pointd())){
                b3.setId(bl.getNumberBoxes());
                b3.setTrianglesCovered(tree.getNumberIntersectedPrimitives(b3));
                b2.setTrianglesCovered(b2.getTrianglesCovered()-b3.getTrianglesCovered());
                bl.setBox(b2.getId(), b2);
                bl.addBox(b3);
                ///
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                //b3.getIGLMesh().saveOnObj("b3.obj");
                ///
                g.removeEdge(arcToRemove.first, arcToRemove.second);
                g.removeEdgeIfExists(arcToRemove.second, arcToRemove.first);
                std::vector<unsigned int> incomingb2 = g.getIncomingNodes(b2.getId());
                std::vector<unsigned int> outgoingb2 = g.getOutgoingNodes(b2.getId());
                g.deleteAllIncomingNodes(b2.getId());
                g.deleteAllOutgoingNodes(b2.getId());
                unsigned int tmp = g.addNode();
                assert(tmp == (unsigned int)b3.getId());
                for (unsigned int incoming : incomingb2){
                    Box3D other = bl.getBox(incoming);
                    ///
                    //other.getIGLMesh().saveOnObj("checkother.obj");
                    //b2.getIGLMesh().saveOnObj("checkb2.obj");
                    //b3.getIGLMesh().saveOnObj("checkb3.obj");
                    ///
                    if (boxesIntersect(other,b2)){
                        if (isDangerousIntersection(other, b2, tree, true)){
                            g.addEdge(incoming,b2.getId());
                            std::cerr << incoming << " -> " << b2.getId() << "\n";
                        }
                    }
                    if (boxesIntersect(other,b3)){
                        if (isDangerousIntersection(other, b3, tree, true)){
                            g.addEdge(incoming,b3.getId());
                            std::cerr << incoming << " -> " << b3.getId() << "\n";
                        }
                    }
                }
                for (unsigned int outgoing : outgoingb2){
                    Box3D other = bl.getBox(outgoing);
                    ///
                    //other.getIGLMesh().saveOnObj("checkother.obj");
                    //b2.getIGLMesh().saveOnObj("checkb2.obj");
                    //b3.getIGLMesh().saveOnObj("checkb3.obj");
                    ///
                    if (boxesIntersect(b2, other)){
                        if (isDangerousIntersection(b2, other, tree, true)){
                            g.addEdge(b2.getId(), outgoing);
                            std::cerr << b2.getId() << " -> " << outgoing << "\n";
                        }
                    }
                    if (boxesIntersect(b3, other)){
                        if (isDangerousIntersection(b3, other, tree, true)){
                            g.addEdge(b3.getId(), outgoing);
                            std::cerr << b3.getId() << " -> " << outgoing << "\n";
                        }
                    }
                }
            }
            else {
                g.removeEdge(arcToRemove.first, arcToRemove.second);
                g.removeEdgeIfExists(arcToRemove.second, arcToRemove.first);
            }
        }
    }while (loops.size() > 0);

    //resorting and map old with new graph
    //this is necessary because small indices prevails if
    //no arcs are on the graph
    bl.sortByTrianglesCovered();
    std::map<unsigned int, unsigned int> mapping;
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        mapping[bl.getBox(i).getId()] = i;
    }
    DirectedGraph newGraph(bl.getNumberBoxes());
    for (unsigned int node = 0; node < g.size(); ++node){
        std::vector<unsigned int> outgoing = g.getOutgoingNodes(node);
        for (unsigned int o : outgoing){
            newGraph.addEdge(mapping[node], mapping[o]);
        }
    }

    //works only if graph has no cycles
    newGraph.getLoops(loops);
    assert(loops.size() == 0);
    Array2D<int> ordering(bl.getNumberBoxes(), bl.getNumberBoxes(), -1); // true -> "<", false -> ">=", undefined -> "-1"
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::set<unsigned int> visited;
        newGraph.visit(visited, i);
        for (unsigned int node : visited){ // i must be > than all nodes in visited
            ordering(node,i) = true;
            ordering(i,node) = false;
        }
    }
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        for (unsigned int j = 0; j < i; j++){
            if (ordering(i,j) == -1) {
                ordering(i,j) = false;
                ordering(j,i) = true;
                for (unsigned int k = 0; k < bl.getNumberBoxes(); k++){
                    //j now is the row
                    if (ordering(j,k) == false){
                        assert(ordering(i,k) == 0 || ordering(i,k) == -1);
                        ordering(i,k) = false;
                        ordering(k,i) = true;
                    }
                }
            }
        }
    }

    return ordering;

}
