#include "splitting.h"

#include "common.h"
#include "common/timer.h"
#include <map>

#include <eigenmesh/algorithms/eigenmesh_algorithms.h>


bool Splitting::boxesIntersect(const Box3D& b1, const Box3D& b2) {
    if (b1.getMaxX() <= b2.getMinX()) return false; // a is left of b
    if (b1.getMinX() >= b2.getMaxX()) return false; // a is right of b
    if (b1.getMaxY() <= b2.getMinY()) return false; // a is above b
    if (b1.getMinY() >= b2.getMaxY()) return false; // a is below b
    if (b1.getMaxZ() <= b2.getMinZ()) return false; // a is behind b
    if (b1.getMinZ() >= b2.getMaxZ()) return false; // a is in front b
    return true; //boxes overlap
}

bool Splitting::boxesIntersectNS(const Box3D& b1, const Box3D& b2) {
    if (b1.getMaxX() < b2.getMinX()) return false; // a is left of b
    if (b1.getMinX() > b2.getMaxX()) return false; // a is right of b
    if (b1.getMaxY() < b2.getMinY()) return false; // a is above b
    if (b1.getMinY() > b2.getMaxY()) return false; // a is below b
    if (b1.getMaxZ() < b2.getMinZ()) return false; // a is behind b
    if (b1.getMinZ() > b2.getMaxZ()) return false; // a is in front b
    return true; //boxes overlap
}

bool Splitting::meshCollide(const SimpleEigenMesh &b1, const SimpleEigenMesh &b2){
        CGALInterface::AABBTree tree(b1, true);
        BoundingBox bb;
        bool first = true;
        for (unsigned int i = 0; i < b2.getNumberVertices() && first; i++){
            Pointd p = b2.getVertex(i);
            if (tree.isInside(p)){
                if (tree.getSquaredDistance(p) > 0)
                    first = false;
            }
        }
        /*for (unsigned int i = 0; i < b2.getNumberFaces(); i++){
            Pointi face = b2.getFace(i);
            for (unsigned f =0; f < 3; f++){
                Pointd p1 = b2.getVertex(face[f]);
                Pointd p2 = b2.getVertex(face[(f+1)%3]);
                if (tree.getNumberIntersectedPrimitives(p1, p2) > 0){
                    if (first){
                        bb.min() = p1.min(p2);
                        bb.max() = p1.max(p2);
                        first = false;
                    }
                    else {
                        bb.min() = bb.min().min(p1);
                        bb.min() = bb.min().min(p2);
                        bb.max() = bb.max().max(p1);
                        bb.max() = bb.max().max(p2);
                    }
                }
            }
        }*/
        if (first)
            return false;
        else
            return true;
        /*else {
            if (!Common::epsilonEqual(bb.getMinX(), bb.getMaxX()) && !Common::epsilonEqual(bb.getMinY(), bb.getMaxY()) && !Common::epsilonEqual(bb.getMinZ(), bb.getMaxZ())){
                ////
                //intersection.saveOnObj("int.obj");
                ////
                return true;
            }
            else
                return false;
        }*/
}

bool Splitting::isDangerousIntersection(const Box3D& b1, const Box3D& b2, const CGALInterface::AABBTree &tree, bool checkMeshes) {
    Vec3 target2 = b2.getTarget();
    BoundingBox bb = b1;

    std::vector<Vec3>::const_iterator cit = std::find(XYZ.begin(), XYZ.begin()+6, target2);
    assert(cit != XYZ.begin()+6);
    unsigned int t = cit - XYZ.begin();

    if (t < 3){
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
            /*if (b){
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
                    if (!checkMeshes || (!(b1.isSplitted()) && !(b2.isSplitted()))){
                        return true;
                    }
                    else {
                        bool b = false;
                        SimpleEigenMesh intersection = EigenMeshAlgorithms::intersection(b1.getEigenMesh(), b2.getEigenMesh());
                        BoundingBox bb = intersection.getBoundingBox();
                        if (intersection.getNumberVertices() > 0 && !Common::epsilonEqual(bb.getMinX(), bb.getMaxX()) && !Common::epsilonEqual(bb.getMinY(), bb.getMaxY()) && !Common::epsilonEqual(bb.getMinZ(), bb.getMaxZ())){
                            b = true;
                        }
                        return b;
                    }
                }
            }*/
            if (b){
                if (!checkMeshes || (!(b1.isSplitted()) && !(b2.isSplitted()))){
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
                        return true;
                    }
                }
                else {
                    bool b = false;
                    SimpleEigenMesh intersection = EigenMeshAlgorithms::intersection(b1.getEigenMesh(), b2.getEigenMesh());
                    BoundingBox bb = intersection.getBoundingBox();
                    if (intersection.getNumberVertices() > 0 && !Common::epsilonEqual(bb.getMinX(), bb.getMaxX()) && !Common::epsilonEqual(bb.getMinY(), bb.getMaxY()) && !Common::epsilonEqual(bb.getMinZ(), bb.getMaxZ())){
                        b = true;
                    }
                    return b;
                }
            }
        }
    }
    else {
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
                if (!checkMeshes || (!(b1.isSplitted()) && !(b2.isSplitted()))){
                    //check if bb is empty
                    bb.max()[ot] = (b1.min()[ot]+EPSILON);
                    for (unsigned int i = 0; i < 3; i++){
                        if (i != ot){
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
                    if (isInside  || tree.getNumberIntersectedPrimitives(bb) > 0){
                        return true;
                    }
                }
                else {
                    bool b = false;
                    SimpleEigenMesh intersection = EigenMeshAlgorithms::intersection(b1.getEigenMesh(), b2.getEigenMesh());
                    BoundingBox bb = intersection.getBoundingBox();
                    if (intersection.getNumberVertices() > 0 && !Common::epsilonEqual(bb.getMinX(), bb.getMaxX()) && !Common::epsilonEqual(bb.getMinY(), bb.getMaxY()) && !Common::epsilonEqual(bb.getMinZ(), bb.getMaxZ())){
                        b = true;
                    }
                    return b;
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
bool Splitting::getSplits(const Box3D& b1, const Box3D& b2, Box3D & b3) {
    b3.setColor(b2.getColor());
    b3.setTarget(b2.getTarget());
    Vec3 target = b2.getTarget();

    std::vector<Vec3>::const_iterator cit = std::find(XYZ.begin(), XYZ.begin()+6, target);
    assert(cit != XYZ.begin()+6);
    unsigned int t = cit - XYZ.begin();

    if (t < 3){ //+x or +y or +z
        if (!(b1.max()[t] <= b2.max()[t])) {
            b3.min() = Pointd();
            b3.max() = Pointd();
            return false;
        }
        b3.min()[t] = b1.max()[t];
        b3.max()[t] = b2.max()[t];
        for (unsigned u = 0; u < 3; u++){
            if (u != t){
                b3.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                b3.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                assert(b3.min()[u] <= b3.max()[u]);
            }
        }
    }
    else if (t >= 3){ //-x or -y or -z
        unsigned int i = t-3;
        if (!(b1.min()[i] >= b2.min()[i])) {
            b3.min() = Pointd();
            b3.max() = Pointd();
            return false;
        }
        b3.min()[i] = b2.min()[i];
        b3.max()[i] = b1.min()[i];
        for (unsigned int u = 0; u < 3; u++){
            if (u != i){
                b3.min()[u] = std::max(b1.min()[u], b2.min()[u]);
                b3.max()[u] = std::min(b1.max()[u], b2.max()[u]);
                assert(b3.min()[u] <= b3.max()[u]);
            }
        }
    }
    return true;
}

void Splitting::splitBox(const Box3D& b1, Box3D& b2, Box3D & b3, double subd) {
    getSplits(b1, b2, b3);

    if (b3.getVolume() > 0){
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
        b3tmp.generateEigenMesh(subd);
        //
        b3.generateEigenMesh(subd);
        SimpleEigenMesh oldBox = b2.getEigenMesh();
        oldBox = EigenMeshAlgorithms::difference(oldBox, b1.getEigenMesh());
        SimpleEigenMesh tmp = oldBox;
        tmp = EigenMeshAlgorithms::intersection(tmp, b3.getEigenMesh());
        if (tmp.getNumberVertices() > 0){
            oldBox = EigenMeshAlgorithms::difference(oldBox, b3tmp.getEigenMesh());
            b2.setEigenMesh(oldBox);
            BoundingBox newBBb2 = oldBox.getBoundingBox();
            b2.setMin(newBBb2.min());
            b2.setMax(newBBb2.max());
            b3.setEigenMesh(tmp);
            BoundingBox newBBb3 = tmp.getBoundingBox();
            b3.setMin(newBBb3.min());
            b3.setMax(newBBb3.max());
        }
        else{
            b2.setEigenMesh(oldBox);
            BoundingBox newBBb2 = oldBox.getBoundingBox();
            b2.setMin(newBBb2.min());
            b2.setMax(newBBb2.max());
            b3.min() = b3.max() = Pointd();
        }
        b2.setSplitted(true);
        b3.setSplitted(true);
    }
    else {
        SimpleEigenMesh oldBox = b2.getEigenMesh();
        oldBox = EigenMeshAlgorithms::difference(oldBox, b1.getEigenMesh());
        b2.setEigenMesh(oldBox);
        BoundingBox newBBb2 = oldBox.getBoundingBox();
        b2.setMin(newBBb2.min());
        b2.setMax(newBBb2.max());
        b3.min() = b3.max() = Pointd();
    }
}


int Splitting::getMinTrianglesCoveredIfBoxesSplitted(const Box3D &b1, const Box3D &b2, const CGALInterface::AABBTree& tree){

    Box3D b3;
    getSplits(b1, b2, b3);
    std::set<unsigned int> b3t = getTrianglesCovered(b3, tree);
    b3t = Common::setIntersection(b3t, b2.getTrianglesCovered());
    std::set<unsigned int> b2t = Common::setDifference(Common::setDifference(b2.getTrianglesCovered(), b1.getTrianglesCovered()), b3t);

    if (b3t.size() == 0){
        return b2t.size();
    }
    int min = std::min(b3t.size(), b2t.size());
    return min;
}

std::set<unsigned int> Splitting::getTrianglesCovered(const Box3D &b, const CGALInterface::AABBTree& aabb, bool completely) {
    std::set<unsigned int> trianglesCovered;
    std::list<const Dcel::Face*> list;
    if (completely)
        aabb.getCompletelyContainedDcelFaces(list, b);
    else
        aabb.getContainedDcelFaces(list, b);
    for (const Dcel::Face* f : list){
        trianglesCovered.insert(f->getId());
    }
    return trianglesCovered;
}

DirectedGraph Splitting::getGraph(const BoxList& bl, const CGALInterface::AABBTree &tree){
    int lastId = bl[0].getId();
    for (unsigned int i = 1; i < bl.getNumberBoxes(); i++){
        if (bl[i].getId() > lastId)
            lastId = bl[i].getId();
    }
    DirectedGraph g(lastId+1);
    std::cerr << "Graph: "<< bl.getNumberBoxes() <<"\n";
    for (unsigned int i = 0; i < bl.getNumberBoxes()-1; i++){
        Box3D b1 = bl.getBox(i);
        for (unsigned int j = i+1; j < bl.getNumberBoxes(); j++){
            Box3D b2 = bl.getBox(j);
            if (boxesIntersect(b1,b2)){
                if (isDangerousIntersection(b1, b2, tree, b1.isSplitted() || b2.isSplitted())){
                    g.addEdge(bl[i].getId(),bl[j].getId());
                    std::cerr << bl[i].getId() << " -> " << bl[j].getId() <<"\n";
                }
                if (isDangerousIntersection(b2, b1, tree, b1.isSplitted() || b2.isSplitted())){
                    g.addEdge(bl[j].getId(),bl[i].getId());
                    std::cerr << bl[j].getId() << " -> " << bl[i].getId() <<"\n";
                }

            }
        }
    }
    return g;
}

std::pair<unsigned int, unsigned int> Splitting::getArcToRemove(const std::vector<std::vector<unsigned int> > &loops, const BoxList &bl, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs, const CGALInterface::AABBTree& tree){
    //looking for the more convinient box to split
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
    //bool found = false;

    std::pair<unsigned int, unsigned int> bestForCycles;
    std::pair<unsigned int, unsigned int> arcToRemove;
    while (it != rev.rend()){
        /*unsigned int size = rev.count((*it).first); // number of arcs that belongs with the maximum number of loops

        if (size == 1){ // there is just one arc associated to the maximum number of loops
            arcToRemove = (*it).second; // if I remove this arc, I will remove the maximum number of loops
            if (std::find(userArcs.begin(), userArcs.end(), arcToRemove) == userArcs.end()){
                bestForCycles = arcToRemove;
                found = true;
            }
        }
        else {*/ // I need to choose wich arc I want to eliminate -> maximum split criteria
            std::vector<std::pair<unsigned int, unsigned int> > candidateArcs;
            for (auto i=rev.equal_range((*it).first).first; i!=rev.equal_range((*it).first).second; ++i){
                std::pair<unsigned int, unsigned int> arc = (*i).second;
                candidateArcs.push_back(arc);
            }
            int nTrimax = -1;
            int maxarc = -1;

            for (unsigned int i = 0; i < candidateArcs.size(); i++) {
                std::pair<unsigned int, unsigned int> arc = candidateArcs[i];
                if (std::find(userArcs.begin(), userArcs.end(), arc) == userArcs.end()){
                    int tmp = getMinTrianglesCoveredIfBoxesSplitted(bl.find(arc.first), bl.find(arc.second), tree);
                    if (tmp >= nTrimax){
                        nTrimax = tmp;
                        maxarc = i;
                    }
                    tmp = getMinTrianglesCoveredIfBoxesSplitted(bl.find(arc.second), bl.find(arc.first), tree);
                    if (tmp >= nTrimax){
                        nTrimax = tmp;
                        maxarc = i;
                    }
                }
            }
            if (maxarc >= 0){
                arcToRemove = candidateArcs[maxarc];
                bestForCycles = arcToRemove;
                //found = true;
            }
        //}
        it++;
    }
    return arcToRemove;
}

void Splitting::chooseBestSplit(Box3D &b1, Box3D &b2, const BoxList &bl, const CGALInterface::AABBTree& tree, const std::set<unsigned int>& boxesToEliminate){
    Box3D bt3mp1, b3tmp2;
    getSplits(b2,b1,b3tmp2);
    //std::set<unsigned int> trianglesCoveredTmp2 = getTrianglesCovered(btmp2, tree, false);
    std::set<unsigned int> trianglesCoveredB3Tmp2 = getTrianglesCovered(b3tmp2, tree);
    trianglesCoveredB3Tmp2 = Common::setDifference(Common::setIntersection(trianglesCoveredB3Tmp2, b1.getTrianglesCovered()), b2.getTrianglesCovered());
    if (trianglesCoveredB3Tmp2.size() == 0 || ((b3tmp2.min() == b3tmp2.max()) && (b3tmp2.min() == Pointd()))){
        std::swap(b1, b2);
    }
    else {
        bool exit = false;

        for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
            if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b1.getId()){
                const std::set<unsigned int>& trianglesCoveredBi = bl[i].getTrianglesCovered();
                if (Common::isSubset(trianglesCoveredB3Tmp2, trianglesCoveredBi)){
                    exit = true;
                }
            }
        }
        if (exit)
            std::swap(b1, b2);
        else {
            getSplits(b1,b2,bt3mp1);
            std::set<unsigned int> trianglesCoveredB3Tmp1 = getTrianglesCovered(bt3mp1, tree);
            trianglesCoveredB3Tmp1 = Common::setDifference(Common::setIntersection(trianglesCoveredB3Tmp1, b2.getTrianglesCovered()), b1.getTrianglesCovered());
            if ((bt3mp1.min() != Pointd() || bt3mp1.max() != Pointd()) && trianglesCoveredB3Tmp1.size() != 0){
                bool exit = false;
                for (unsigned int i = 0; i < bl.getNumberBoxes() && !exit; i++){
                    if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b2.getId()){
                        const std::set<unsigned int>& trianglesCoveredBi = bl[i].getTrianglesCovered();
                        if (Common::isSubset(trianglesCoveredB3Tmp1, trianglesCoveredBi)){
                            exit = true;
                        }
                    }
                }
                if (!exit){
                    if (trianglesCoveredB3Tmp2.size() > trianglesCoveredB3Tmp1.size())
                        std::swap(b1, b2);
                }
            }
        }
    }
}

bool Splitting::checkDeleteBox(const Box3D &b, const std::set<unsigned int>& boxesToEliminate,  const BoxList &bl){
    bool bIsEliminated = false;
    if (b.getTrianglesCovered().size() == 0){
        bIsEliminated = true;
    }
    else {
        for (unsigned int i = 0; i < bl.getNumberBoxes() && !bIsEliminated; i++){
            if (boxesToEliminate.find(i) == boxesToEliminate.end() && (int)i != b.getId()){
                const std::set<unsigned int>& trianglesCoveredBi = bl[i].getTrianglesCovered();

                if (Common::isSubset(b.getTrianglesCovered(), trianglesCoveredBi)){
                    bIsEliminated = true;
                }
            }
        }
    }
    return bIsEliminated;
}

void Splitting::splitB2(const Box3D& b1, Box3D& b2, BoxList& bl, DirectedGraph& g, const CGALInterface::AABBTree& tree, std::set<unsigned int> &boxesToEliminate, std::map<unsigned int, unsigned int> &mappingNewToOld, int& numberOfSplits, int& deletedBoxes, std::set<std::pair<unsigned int, unsigned int>, cmpUnorderedStdPair<unsigned int>> &impossibleArcs) {
    int lastId = bl[0].getId();
    for (unsigned int i = 1; i < bl.getNumberBoxes(); i++){
        if (bl[i].getId() > lastId)
            lastId = bl[i].getId();
    }
    std::set<unsigned int> tcb1 = b1.getTrianglesCovered();
    std::set<unsigned int> tcb2 = b2.getTrianglesCovered();
    std::set<unsigned int> tcb23 = Common::setDifference(tcb2, tcb1);
    Box3D b3;
    splitBox(b1, b2, b3);
    //splitBox(b1, b2, b3, d.getAverageHalfEdgesLength()*LENGTH_MULTIPLIER);
    std::pair<unsigned int, unsigned int> impPair(b1.getId(), b2.getId());
    impossibleArcs.insert(impPair);
    if (!(b3.min() == Pointd() && b3.max() == Pointd())){ //se b3 esiste
        g.removeEdgeIfExists(b1.getId(), b2.getId());
        g.removeEdgeIfExists(b2.getId(), b1.getId());
        b3.setId(lastId+1);
        //std::set<unsigned int> tcb3 = Common::setIntersection(getTrianglesCovered(b3, tree, false), tcb23);
        std::set<unsigned int> tcb3 = Common::setIntersection(getTrianglesCovered(b3, tree), tcb23);

        /////gestione b2:
        b2.setTrianglesCovered(Common::setDifference(tcb23, tcb3));
        bl.setBox(b2.getId(), b2);
        ///
        //b1.getEigenMesh().saveOnObj("b1.obj");
        //b2.getEigenMesh().saveOnObj("b2.obj");
        //b3.getEigenMesh().saveOnObj("b3.obj");
        ///
        std::vector<unsigned int> incomingb2 = g.getIncomingNodes(b2.getId());
        std::vector<unsigned int> outgoingb2 = g.getOutgoingNodes(b2.getId());
        g.deleteAllIncomingNodes(b2.getId());
        g.deleteAllOutgoingNodes(b2.getId());

        //qualcuno copre già tutti i triangoli coperti da b2? se si, b2 viene aggiunta alle box da eliminare, e nessun arco punterà più ad essa
        bool b2IsEliminated = Splitting::checkDeleteBox(b2, boxesToEliminate, bl);

        if (b2IsEliminated){
            boxesToEliminate.insert(b2.getId());
            deletedBoxes++;
        }
        else{
            //ricontrollo tutti i conflitti di b2 (archi entranti e uscenti)
            for (unsigned int incoming : incomingb2){
                Box3D other = bl.find(incoming);
                std::pair<unsigned int, unsigned int> pp(b2.getId(), incoming);
                if (boxesIntersect(other,b2) && impossibleArcs.find(pp) == impossibleArcs.end()){
                    if (isDangerousIntersection(other, b2, tree, true)){
                        g.addEdge(incoming,b2.getId());
                    }
                }
            }
            for (unsigned int outgoing : outgoingb2){
                Box3D other = bl.find(outgoing);
                std::pair<unsigned int, unsigned int> pp(b2.getId(), outgoing);
                if (boxesIntersect(b2, other) && impossibleArcs.find(pp) == impossibleArcs.end()){
                    if (isDangerousIntersection(b2, other, tree, true)){
                        g.addEdge(b2.getId(), outgoing);
                    }
                }
            }
        }

        //////gestione b3:


        //qualcuno copre già tutti i triangoli coperti da b3? se si, b3 non viene aggiunta alla box list
        b3.setTrianglesCovered(tcb3);
        bool b3IsEliminated = Splitting::checkDeleteBox(b3, boxesToEliminate, bl);

        if (b3IsEliminated){
            deletedBoxes++;
        }
        else {
            bl.addBox(b3);
            bool cont = true;
            unsigned int idtmp = b2.getId();
            do {
                mappingNewToOld[b3.getId()]= idtmp;
                if (mappingNewToOld[idtmp] == idtmp)
                    cont = false;
                else {
                    idtmp = mappingNewToOld[idtmp];
                }
            } while (cont);
            std::pair<unsigned int, unsigned int> p1(b3.getId(), b2.getId()), p2(b3.getId(), b1.getId());
            impossibleArcs.insert(p1);
            impossibleArcs.insert(p2);
            //costruisco tutti i conflitti di b3 (archi entranti e uscenti)
            g.addNode();
            for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
                std::pair<unsigned int, unsigned int> pp (b3.getId(), i);
                if (impossibleArcs.find(pp) == impossibleArcs.end()){
                    Box3D other = bl.getBox(i);
                    if (boxesIntersect(other,b3)){
                        if (isDangerousIntersection(other, b3, tree, true)){
                            g.addEdge(i, b3.getId());
                        }
                    }
                    if (boxesIntersect(b3, other)){
                        if (isDangerousIntersection(b3, other, tree, true)){
                            g.addEdge(b3.getId(), i);
                        }
                    }
                }
            }
        }
        numberOfSplits++;


    }
    else { // altrimenti b3 non esiste, e b2 è uguale a b2-b1
        g.removeEdgeIfExists(b1.getId(), b2.getId());
        g.removeEdgeIfExists(b2.getId(), b1.getId());
        b2.setTrianglesCovered(tcb23);
        bl.setBox(b2.getId(), b2);
    }
}

Array2D<int> Splitting::getOrdering(BoxList& bl, const Dcel& d, std::map<unsigned int, unsigned int> &mappingNewToOld, std::list<unsigned int>& priorityBoxes, const std::vector<std::pair<unsigned int, unsigned int> >& userArcs) {
    CGALInterface::AABBTree tree(d);
    int lastId = bl[0].getId();
    for (unsigned int i = 1; i < bl.getNumberBoxes(); i++){
        if (bl[i].getId() > lastId)
            lastId = bl[i].getId();
    }
    std::set<unsigned int> boxesToEliminate; //set of boxes to eliminate after the splitting -> these boxes cannot removed from bl during the splitting
    std::vector<std::vector<unsigned int> > loops;
    std::set<std::pair<unsigned int, unsigned int>, cmpUnorderedStdPair<unsigned int>> impossibleArcs;

    DirectedGraph g = getGraph(bl, tree);

    for (const std::pair<unsigned int, unsigned int>& p : userArcs)
        g.addEdge(p.first, p.second);

    int numberOfSplits = 0;
    int deletedBoxes = 0;
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++)
        mappingNewToOld[bl[i].getId()] = bl[i].getId();

    if (priorityBoxes.size() != 0){
        for (unsigned int pb : priorityBoxes){
            Box3D b1 = bl.find(pb);
            /*std::vector<unsigned int> incoming = g.getIncomingNodes(pb);
            for (unsigned int inc : incoming){
                Box3D b2 = bl.find(inc);
                std::cerr << b1.getId() << " will split " << b2.getId() << "\n";
                splitB2(b1, b2, bl, g, d, tree, boxesToEliminate, mappingNewToOld, numberOfSplits, deletedBoxes, impossibleArcs);
            }*/

            // Don't need to split outgoings: if no arcs enter on b1, no cycles are possible and b1 will never be splitted.
            std::vector<unsigned int> outgoing = g.getOutgoingNodes(pb);
            for (unsigned int out : outgoing) {
                Box3D b2 = bl.find(out);
                std::cerr << b1.getId() << " will split " << b2.getId() << "\n";
                splitB2(b1, b2, bl, g, tree, boxesToEliminate, mappingNewToOld, numberOfSplits, deletedBoxes, impossibleArcs);
            }

            /*for (unsigned int inc : incoming){
                assert(! g.arcExists(pb, inc));
                assert(! g.arcExists(inc, pb));
            }*/
            #ifndef NDEBUG
            for (unsigned int out : outgoing) {
                assert(! g.arcExists(pb, out));
                assert(! g.arcExists(out, pb));
            }
            #endif

        }
    }

    ///Detect and delete cycles on graph (modifying bl)
    do {
        loops = g.getCircuits();
        std::cerr << "Number loops: " << loops.size() << "\n";
        if (loops.size() > 0){ // I need to modify bl

            std::pair<unsigned int, unsigned int> arcToRemove;
            arcToRemove = getArcToRemove(loops, bl, userArcs, tree);
            assert(std::find(userArcs.begin(), userArcs.end(), arcToRemove) == userArcs.end());

            std::cerr << "Arc to Remove: " << arcToRemove.first << ", " << arcToRemove.second << "\n";

            // now I can remove "arcToRemove"
            Box3D b1 = bl.find(arcToRemove.first), b2 = bl.find(arcToRemove.second);

            ///
            ///
            /// now I can choose which box split, b1 or b2

            if (std::find(userArcs.begin(), userArcs.end(), std::pair<unsigned int, unsigned int>(arcToRemove.second, arcToRemove.first)) == userArcs.end())
                chooseBestSplit(b1, b2, bl, tree, boxesToEliminate);
            //now b1 will split b2 in b2+b3

            ///
            ///
            ///

            splitB2(b1, b2, bl, g, tree, boxesToEliminate, mappingNewToOld, numberOfSplits, deletedBoxes, impossibleArcs);
        }
    }while (loops.size() > 0);

    for (const std::pair<unsigned int, unsigned int>& p : userArcs)
        g.addEdgeIfNotExists(p.first, p.second);

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
    // I create the new graph, and I put the arcs mapping new id to old id
    DirectedGraph newGraph(bl.getNumberBoxes());
    for (unsigned int node = 0; node < g.size(); ++node){
        std::vector<unsigned int> outgoing = g.getOutgoingNodes(node);
        for (unsigned int o : outgoing){
            newGraph.addEdge(mapping[node], mapping[o]);
        }
    }

    //get the ordering from the graph
    //works only if graph has no cycles
    loops = newGraph.getCircuits();
    assert(loops.size() == 0);
    lastId = bl[0].getId();
    for (unsigned int i = 1; i < bl.getNumberBoxes(); i++){
        if (bl[i].getId() > lastId)
            lastId = bl[i].getId();
    }
    Array2D<int> ordering(lastId+1, lastId+1, -1); // true -> "<", false -> ">=", undefined -> "-1"
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        std::set<unsigned int> visited;
        newGraph.visit(visited, i);
        for (unsigned int node : visited){ // i must be > than all nodes in visited
            ordering(bl[node].getId(),bl[i].getId()) = true;
            ordering(bl[i].getId(),bl[node].getId()) = false;
        }
    }
    for (unsigned int i = 0; i < bl.getNumberBoxes(); i++){
        for (unsigned int j = 0; j < i; j++){
            if (ordering(bl[i].getId(),bl[j].getId()) == -1) {
                ordering(bl[i].getId(),bl[j].getId()) = false;
                ordering(bl[j].getId(),bl[i].getId()) = true;
                for (unsigned int k = 0; k < bl.getNumberBoxes(); k++){
                    //j now is the row
                    if (ordering(bl[j].getId(),bl[k].getId()) == false){
                        assert(ordering(bl[i].getId(),bl[k].getId()) == 0 || ordering(bl[i].getId(),bl[k].getId()) == -1);
                        ordering(bl[i].getId(),bl[k].getId()) = false;
                        ordering(bl[k].getId(),bl[i].getId()) = true;
                    }
                }
            }
        }
    }

    for (std::list<unsigned int>::reverse_iterator it = priorityBoxes.rbegin(); it != priorityBoxes.rend(); ++it){
        unsigned int pb = *it;
        for (unsigned int j = 0; j < bl.getNumberBoxes(); j++){
            if (j != pb){
                ordering(pb, j) = true;
                ordering(j, pb) = false;
            }
        }
    }

    return ordering;
}
