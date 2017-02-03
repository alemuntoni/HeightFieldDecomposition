#include "splitting.h"

#include "common.h"
#include "common/timer.h"
#include "lib/graph/directedgraph.h"
#include "igl/iglinterface.h"
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

/**
 * @brief Splitting::getSplits
 * @param b1
 * @param b2
 * @param b3
 * @return the volume of the piece of b1 cutted off by b2
 */
double Splitting::getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3) {
    Box3D b4; // the piece of b2 cutted off by b1
    b3.setColor(b2.getColor());
    b3.setTarget(b2.getTarget());
    Vec3 target = b2.getTarget();
    for (unsigned int t = 0; t < 6; t++){
        if (target == XYZ[t]){
            if (t < 3){
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                //assert(b1.max()[t] <= b2.max()[t]);
                if (!(b1.max()[t] <= b2.max()[t])) {
                    b3.min() = Pointd();
                    b3.max() = Pointd();
                    return -1;
                }
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
                //assert(b1.min()[i] >= b2.min()[i]);
                if (!(b1.min()[i] >= b2.min()[i])) {
                    b3.min() = Pointd();
                    b3.max() = Pointd();
                    return -1;
                }
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
    //b4 is the piece cutted by b1 on b2, therefore oldb2 = newB2 + b3 + b4
    double volumeb4 = getSplits(b1, b2, b3);
    double volumeb2 = b2.getVolume();
    double volumeb3 = b3.getVolume();
    double newVolumeb2 = volumeb2 - volumeb3 - volumeb4;
    assert (newVolumeb2 >= 0);
    return std::min(newVolumeb2, volumeb3);
    //double tmp = std::min(b3.getLengthX(), b3.getLengthY());
    //return std::min(tmp, b3.getLengthZ());
}

std::set<unsigned int> Splitting::getTrianglesCovered(const Box3D &b, const CGALInterface::AABBTree& aabb) {
    std::set<unsigned int> trianglesCovered;
    std::list<const Dcel::Face*> list;
    aabb.getCompletelyContainedDcelFaces(list, b);
    for (const Dcel::Face* f : list){
        trianglesCovered.insert(f->getId());
    }
    return trianglesCovered;
}

Array2D<int> Splitting::getOrdering(BoxList& bl, const Dcel& d) {
    CGALInterface::AABBTree tree(d);
    bl.generatePieces(d.getAverageHalfEdgesLength()*LENGTH_MULTIPLIER);
    bl.calculateTrianglesCovered(tree);
    bl.sortByTrianglesCovered();
    bl.setIds();
    std::vector< std::set<unsigned int> > trianglesCovered(bl.getNumberBoxes());
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        trianglesCovered[i] = getTrianglesCovered(bl.getBox(i), tree);
    }
    std::set<unsigned int> boxesToEliminate;
    std::vector<std::vector<unsigned int> > loops;
    DirectedGraph g(bl.getNumberBoxes());
    for (unsigned int i = 0; i < bl.getNumberBoxes()-1; i++){
        Box3D b1 = bl.getBox(i);
        for (unsigned int j = i+1; j < bl.getNumberBoxes(); j++){
            Box3D b2 = bl.getBox(j);
            if (boxesIntersect(b1,b2)){
                if (isDangerousIntersection(b1, b2, tree)){
                    g.addEdge(i,j);
                    //std::cerr << i << " -> " << j << "\n";
                }
                if (isDangerousIntersection(b2, b1, tree)){
                    g.addEdge(j,i);
                    //std::cerr << j << " -> " << i << "\n";
                }

            }
        }
    }
    ///Detect and delete cycles on graph (modifying bl)

    int numberOfSplits = 0;
    int deletedBoxes = 0;
    //
    /*d.saveOnObjFile("mesh.obj");
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::stringstream ss;
        ss << "b" << i << ".obj";
        bl.getBox(i).getIGLMesh().saveOnObj(ss.str());
    }*/
    //
    do {
        g.getLoops(loops);
        std::cerr << "Number loops: " << loops.size() << "\n";
        if (loops.size() > 0){ // I need to modify bl

            std::map<std::pair<unsigned int, unsigned int>, int> arcs;
            for (std::vector<unsigned int> loop : loops){
                unsigned int node = 0;
                for (node = 0; node < loop.size()-1; node++){
                    std::pair<unsigned int, unsigned int> arc(loop[node], loop[node+1]);
                    std::map<std::pair<unsigned int, unsigned int>, int>::iterator it = arcs.find(arc);
                    if (it == arcs.end())
                        arcs[arc] = 1;
                    else
                        arcs[arc]++;
                }
            }
            std::multimap<int,std::pair<unsigned int, unsigned int> > rev = Common::flipMap(arcs);

            std::multimap<int,std::pair<unsigned int, unsigned int> >::reverse_iterator it = rev.rbegin();

            unsigned int size = rev.count((*it).first); // number of arcs that belongs with the maximum number of loops
            std::pair<unsigned int, unsigned int> arcToRemove;
            if (size == 1){ // if I remove this arc, I will remove the maximum number of loops
                arcToRemove = (*it).second;
            }
            else { // I need to choose wich arc I want to eliminate -> maximum split
                std::vector<std::pair<unsigned int, unsigned int> > candidateArcs;
                for (auto i=rev.equal_range((*it).first).first; i!=rev.equal_range((*it).first).second; ++i){
                    std::pair<unsigned int, unsigned int> arc = (*i).second;
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

            ///
            ///
            /// now I can choose which box split, b1 or b2

            Box3D btmp1, btmp2;
            getSplits(b2,b1,btmp2);
            if (tree.getNumberIntersectedPrimitives(btmp2) == 0){
                std::swap(b1, b2);
            }
            else {
                bool exit = false;
                std::set<unsigned int> trianglesCoveredTmp = getTrianglesCovered(btmp2, tree);
                for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
                    if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b1.getId()){
                        std::set<unsigned int>& trianglesCoveredBi = trianglesCovered[i];
                        if (std::includes(trianglesCoveredBi.begin(), trianglesCoveredBi.end(), trianglesCoveredTmp.begin(), trianglesCoveredTmp.end())){
                            exit = true;
                        }
                    }
                }
                if (exit)
                    std::swap(b1, b2);
                else {
                    getSplits(b1,b2,btmp1);
                    if (tree.getNumberIntersectedPrimitives(btmp1) != 0){
                        bool exit = false;
                        std::set<unsigned int> trianglesCoveredTmp = getTrianglesCovered(btmp1, tree);
                        for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
                            if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b2.getId()){
                                std::set<unsigned int>& trianglesCoveredBi = trianglesCovered[i];
                                if (std::includes(trianglesCoveredBi.begin(), trianglesCoveredBi.end(), trianglesCoveredTmp.begin(), trianglesCoveredTmp.end())){
                                    exit = true;
                                }
                            }
                        }
                        if (!exit){
                            double minb1b2 = minimumSplit(b1, b2);
                            double minb2b1 = minimumSplit(b2, b1);
                            if (minb2b1 > minb1b2)
                                std::swap(b1, b2);
                        }
                    }
                }
            }

            ///
            ///
            ///

            splitBox(b1, b2, b3, d.getAverageHalfEdgesLength()*LENGTH_MULTIPLIER);
            if (!(b3.min() == Pointd() && b3.max() == Pointd())){ //se b3 esiste
                g.removeEdgeIfExists(arcToRemove.first, arcToRemove.second);
                g.removeEdgeIfExists(arcToRemove.second, arcToRemove.first);
                b3.setId(bl.getNumberBoxes());
                b3.setTrianglesCovered(tree.getNumberIntersectedPrimitives(b3));
                std::set<unsigned int> trianglesCoveredB3 = getTrianglesCovered(b3, tree);

                /////gestione b2:
                b2.setTrianglesCovered(b2.getTrianglesCovered()-b3.getTrianglesCovered());
                bl.setBox(b2.getId(), b2);
                ///
                //b1.getIGLMesh().saveOnObj("b1.obj");
                //b2.getIGLMesh().saveOnObj("b2.obj");
                //b3.getIGLMesh().saveOnObj("b3.obj");
                ///
                std::vector<unsigned int> incomingb2 = g.getIncomingNodes(b2.getId());
                std::vector<unsigned int> outgoingb2 = g.getOutgoingNodes(b2.getId());
                g.deleteAllIncomingNodes(b2.getId());
                g.deleteAllOutgoingNodes(b2.getId());

                //update dei triangoli coperti da b2: tutti quelli che sono coperti da b1 e b3 non sono più coperti da b2

                std::set<unsigned int> trianglesCoveredB1 = trianglesCovered[b1.getId()];
                std::set<unsigned int> trianglesCoveredB2 = trianglesCovered[b2.getId()];
                std::set<unsigned int> tmp;
                std::set_difference(trianglesCoveredB2.begin(), trianglesCoveredB2.end(), trianglesCoveredB1.begin(), trianglesCoveredB1.end(), std::inserter(tmp, tmp.begin()));
                trianglesCoveredB2.clear();
                std::set_difference(tmp.begin(), tmp.end(), trianglesCoveredB3.begin(), trianglesCoveredB3.end(), std::inserter(trianglesCoveredB2,trianglesCoveredB2.begin()));
                trianglesCovered[b2.getId()] = trianglesCoveredB2;

                //qualcuno copre già tutti i triangoli coperti da b2? se si, b2 viene aggiunta alle box da eliminare, e nessun arco punterà più ad essa
                bool exit = false;

                if (trianglesCoveredB2.size() == 0){
                    exit = true;
                    deletedBoxes++;
                }
                else {
                    for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
                        if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b2.getId()){
                            std::set<unsigned int>& trianglesCoveredBi = trianglesCovered[i];
                            if (std::includes(trianglesCoveredBi.begin(), trianglesCoveredBi.end(), trianglesCoveredB2.begin(), trianglesCoveredB2.end())){
                                exit = true;
                                boxesToEliminate.insert(b2.getId());
                                deletedBoxes++;
                            }
                        }
                    }
                }

                if (!exit){

                    //ricontrollo tutti i conflitti di b2 (archi entranti e uscenti)
                    for (unsigned int incoming : incomingb2){
                        Box3D other = bl.getBox(incoming);
                        ///
                        //other.getIGLMesh().saveOnObj("checkother.obj");
                        //b2.getIGLMesh().saveOnObj("checkb2.obj");
                        ///
                        if (boxesIntersect(other,b2)){
                            if (isDangerousIntersection(other, b2, tree, true)){
                                g.addEdge(incoming,b2.getId());
                            }
                        }
                    }
                    for (unsigned int outgoing : outgoingb2){
                        Box3D other = bl.getBox(outgoing);
                        ///
                        //other.getIGLMesh().saveOnObj("checkother.obj");
                        //b2.getIGLMesh().saveOnObj("checkb2.obj");
                        ///
                        if (boxesIntersect(b2, other)){
                            if (isDangerousIntersection(b2, other, tree, true)){
                                g.addEdge(b2.getId(), outgoing);
                            }
                        }
                    }
                    trianglesCovered[b2.getId()] = trianglesCoveredB2;
                    b2.setTrianglesCovered(trianglesCoveredB2.size());
                }

                //////gestione b3:


                //qualcuno copre già tutti i triangoli coperti da b3? se si, b3 non viene aggiunta alla box list
                exit = false;
                if (trianglesCoveredB3.size() == 0){
                    exit = true;
                    deletedBoxes++;
                }
                else {
                    for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
                        if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b2.getId()){
                            std::set<unsigned int>& trianglesCoveredBi = trianglesCovered[i];
                            if (std::includes(trianglesCoveredBi.begin(), trianglesCoveredBi.end(), trianglesCoveredB3.begin(), trianglesCoveredB3.end())){
                                exit = true;
                                deletedBoxes++;
                            }
                        }
                    }
                }

                if (!exit){
                    bl.addBox(b3);
                    //costruisco tutti i conflitti di b3 (archi entranti e uscenti)
                    g.addNode();
                    for (unsigned int incoming : incomingb2){
                        Box3D other = bl.getBox(incoming);
                        ///
                        //other.getIGLMesh().saveOnObj("checkother.obj");
                        //b3.getIGLMesh().saveOnObj("checkb3.obj");
                        ///
                        if (boxesIntersect(other,b3)){
                            if (isDangerousIntersection(other, b3, tree, true)){
                                g.addEdge(incoming,b3.getId());
                            }
                        }
                    }
                    for (unsigned int outgoing : outgoingb2){
                        Box3D other = bl.getBox(outgoing);
                        ///
                        //other.getIGLMesh().saveOnObj("checkother.obj");
                        //b3.getIGLMesh().saveOnObj("checkb3.obj");
                        ///
                        if (boxesIntersect(b3, other)){
                            if (isDangerousIntersection(b3, other, tree, true)){
                                g.addEdge(b3.getId(), outgoing);
                            }
                        }
                    }
                    trianglesCovered.push_back(getTrianglesCovered(b3, tree));
                    b3.setTrianglesCovered(trianglesCovered[trianglesCovered.size()-1].size());
                }


                numberOfSplits++;


            }
            else { // altrimenti b3 non esiste, e b2 è rimasta uguale a prima
                g.removeEdgeIfExists(arcToRemove.first, arcToRemove.second);
                g.removeEdgeIfExists(arcToRemove.second, arcToRemove.first);
            }
        }
    }while (loops.size() > 0);
    std::cerr << "Number of Splits: " << numberOfSplits << "\n";
    std::cerr << "Number of Deleted Boxes: " << deletedBoxes << "\n";

    for (std::set<unsigned int>::reverse_iterator rit = boxesToEliminate.rbegin(); rit != boxesToEliminate.rend(); ++rit){
        bl.removeBox(*rit);
    }

    //resorting and map old with new graph
    //this is necessary because small indices prevails if
    //no arcs are on the graph
    bl.sortByTrianglesCovered();
    //bl.sortByHeight();
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

    //get the ordering from the graph
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
