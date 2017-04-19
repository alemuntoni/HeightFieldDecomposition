#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include <algorithm>
#include <map>

class Graph {
    public:
        Graph();
        Graph(unsigned int numberNodes);
        unsigned int size() const;
        unsigned int addNode(int n = -1);
        void addEdge(unsigned int node1, unsigned int node2);
        void removeNode(unsigned int n);
        void removeEdge(unsigned int node1, unsigned int node2);
        bool removeEdgeIfExists(unsigned int node1, unsigned int node2);
        std::vector<unsigned int> getIncomingNodes(unsigned int node);
        std::vector<unsigned int> getOutgoingNodes(unsigned int node);
        void deleteAllIncomingNodes(unsigned int node);
        void deleteAllOutgoingNodes(unsigned int node);
        bool arcExists(unsigned int n1, unsigned int n2);
        void visit(std::set<unsigned int>& visitedNodes, unsigned int startingNode);
        Graph subGraph(const std::set<unsigned int>& subNodes);
        std::vector<unsigned int> getStronglyConnectedComponent(unsigned int n);
        std::vector<std::vector<unsigned int> > getStronglyConnectedComponents();
        std::vector<std::vector<unsigned int> > getCircuits();

    private:
        bool circuit(const Graph &Ak, unsigned int v, unsigned int s, std::map<unsigned int, bool> &blocked, std::map<unsigned int, std::set<unsigned> > &B, std::vector<unsigned int> &stack, std::vector<std::vector<unsigned int> >& cycles);
        void unblock(unsigned int u, std::map<unsigned int, bool> &blocked, std::map<unsigned int, std::set<unsigned> > &B);
        void trajanSCC(unsigned int v, unsigned int& index, std::map<unsigned int, int>& nodeToIndex, std::map<unsigned int, unsigned int>& minDist, std::vector<unsigned int>& S, std::vector<std::vector<unsigned int> >& out);
        std::map<unsigned int, std::vector<unsigned int> > nodes;
};

inline Graph::Graph() {
}

inline Graph::Graph(unsigned int numberNodes){
    for (unsigned int i = 0; i < numberNodes; i++)
        nodes[i] = std::vector<unsigned int>();
}

inline unsigned int Graph::size() const {
    return nodes.size();
}

inline unsigned int Graph::addNode(int n) {
    if (n < 0){
        n = 0;
        while (nodes.find(n) != nodes.end()) n++;
    }
    else {
        assert(nodes.find(n) == nodes.end());
    }
    nodes[n] = std::vector<unsigned int>();
    return n;
}

inline void Graph::addEdge(unsigned int node1, unsigned int node2) {
    assert(nodes.find(node1) != nodes.end());
    assert(nodes.find(node2) != nodes.end());
    nodes[node1].push_back(node2);
}

inline void Graph::removeNode(unsigned int n) {
    assert(nodes.find(n) != nodes.end());
    nodes.erase(n);
}

inline void Graph::removeEdge(unsigned int node1, unsigned int node2) {
    assert(nodes.find(node1) != nodes.end());
    assert(nodes.find(node2) != nodes.end());
    std::vector<unsigned int>::iterator it = std::find(nodes[node1].begin(), nodes[node1].end(), node2);
    assert(it != nodes[node1].end());
    nodes[node1].erase(it);
}

inline bool Graph::removeEdgeIfExists(unsigned int node1, unsigned int node2) {
    assert(nodes.find(node1) != nodes.end());
    assert(nodes.find(node2) != nodes.end());
    std::vector<unsigned int>::iterator it = std::find(nodes[node1].begin(), nodes[node1].end(), node2);
    if (it != nodes[node1].end()){
        nodes[node1].erase(it);
        return true;
    }
    else {
        return false;
    }
}

inline std::vector<unsigned int> Graph::getIncomingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    std::vector<unsigned int> incoming;
    for(std::pair<unsigned int, std::vector<unsigned int> > p : nodes){
        if(std::find(p.second.begin(), p.second.end(), node) != p.second.end())
            incoming.push_back(p.first);
    }
    return incoming;
}

inline std::vector<unsigned int> Graph::getOutgoingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    return nodes[node];
}

inline void Graph::deleteAllIncomingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    for (std::map<unsigned int, std::vector<unsigned int> >::iterator it = nodes.begin(); it != nodes.end(); ++it){
        std::vector<unsigned int>::iterator vit = std::find(it->second.begin(), it->second.end(), node);
        if (vit != it->second.end())
            nodes[it->first].erase(vit);
    }
//    for (std::pair<unsigned int, std::vector<unsigned int> > p : nodes){
//        std::vector<unsigned int>::iterator it = std::find(p.second.begin(), p.second.end(), node);
//        if(it != p.second.end())
//            nodes[p.first].erase(node);
//    }
}

inline void Graph::deleteAllOutgoingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    nodes[node].clear();
}

inline bool Graph::arcExists(unsigned int n1, unsigned int n2) {
    if (nodes.find(n1) == nodes.end())
        return false;
    else {
        return std::find(nodes[n1].begin(), nodes[n1].end(), n2) != nodes[n1].end();
    }
}

inline void Graph::visit(std::set<unsigned int>& visitedNodes, unsigned int startingNode) {
    assert(nodes.find(startingNode) != nodes.end());
    visitedNodes.insert(startingNode);
    for(unsigned int adjacent : nodes[startingNode]){
        if (visitedNodes.find(adjacent) == visitedNodes.end()){
            visit(visitedNodes, adjacent);
        }
    }
}

inline Graph Graph::subGraph(const std::set<unsigned int>& subNodes) {
    Graph sg;
    for (unsigned int n : subNodes){
        std::map<unsigned int, std::vector<unsigned int> >::iterator it = nodes.find(n);
        if (it != nodes.end()){
            sg.addNode(n);

        }
    }

    for (unsigned int n : subNodes){
        std::map<unsigned int, std::vector<unsigned int> >::iterator it = nodes.find(n);
        if (it != nodes.end()){
            for (unsigned int ad : it->second){
                if (subNodes.find(ad) != subNodes.end()){
                    sg.addEdge(n, ad);
                }
            }
        }
    }
    return sg;
}

inline std::vector<unsigned int> Graph::getStronglyConnectedComponent(unsigned int n) {
    std::vector<std::vector<unsigned int> > out;
    unsigned int index = 0;
    std::vector<unsigned int> S;
    std::map<unsigned int, int> nodeToindex;
    std::map<unsigned int, unsigned int> minDist;
    for (std::pair<unsigned int, std::vector<unsigned int> > v : nodes){
        nodeToindex[v.first] = -1;
    }
    trajanSCC(n, index, nodeToindex, minDist, S, out);
    int found = -1;
    for (unsigned int i = 0; i < out.size() && found == -1; i++){
        auto it = std::find(out[i].begin(), out[i].end(), n);
        if (it != out[i].end())
            found = i;
    }
    assert(found != -1);
    return out[found];
}

inline std::vector<std::vector<unsigned int> > Graph::getStronglyConnectedComponents() {
    std::vector<std::vector<unsigned int> > out;
    unsigned int index = 0;
    std::vector<unsigned int> S;
    std::map<unsigned int, int> nodeToindex;
    std::map<unsigned int, unsigned int> minDist;
    for (std::pair<unsigned int, std::vector<unsigned int> > v : nodes){
        nodeToindex[v.first] = -1;
    }
    for (std::pair<unsigned int, std::vector<unsigned int> > v : nodes){
        assert(nodeToindex.find(v.first) != nodeToindex.end());
        if (nodeToindex[v.first] == -1){ // if v is not associated to a Strong Connected Component
            trajanSCC(v.first, index, nodeToindex, minDist, S, out);
        }
    }
    return out;
}

inline std::vector<std::vector<unsigned int> > Graph::getCircuits() {
        std::vector<unsigned int> stack;
        std::vector< std::vector<unsigned int> > circuits;
        unsigned int s = 0;
        auto it = nodes.end();
        --it;
        unsigned int n = it->first; // last node on graph
        while (s <= n){
            auto it = nodes.find(s);
            if (it != nodes.end()) { // s is the next node
                std::set<unsigned int> W;
                for (; it != nodes.end(); ++it)
                    W.insert(it->first); // {s, s+1, ..., n}
                Graph sg = subGraph(W);
                std::vector<unsigned int> scc = sg.getStronglyConnectedComponent(s);
                if (scc.size() > 1) {
                    Graph Ak = subGraph(std::set<unsigned int>(scc.begin(), scc.end()));
                    std::map<unsigned int, bool> blocked;
                    std::map<unsigned int, std::set<unsigned> > B;
                    for (unsigned int i = 0; i < scc.size(); i++){
                        blocked[scc[i]] = false;
                        B[scc[i]] = std::set<unsigned int>();
                    }
                    circuit(Ak, s, s, blocked, B, stack, circuits);
                }

            }
            s++;
        }

        return circuits;
}

inline bool Graph::circuit(const Graph& Ak, unsigned int v, unsigned int s, std::map<unsigned int, bool>& blocked, std::map<unsigned int, std::set<unsigned> >& B, std::vector<unsigned int>& stack, std::vector< std::vector<unsigned int> > &cycles) {
    bool f = false;
    stack.push_back(v);
    assert(blocked.find(v) != blocked.end());
    blocked[v] = true;
    assert(Ak.nodes.find(v) != Ak.nodes.end());
    for (unsigned int w : Ak.nodes.at(v)){
        if (w == s) {
            cycles.push_back(stack);
            cycles[cycles.size()-1].push_back(s);
            f = true;
        }
        else {
            assert(blocked.find(w) != blocked.end());
            if (!blocked[w])
                if (circuit(Ak, w, s, blocked, B, stack, cycles))
                    f = true;
        }
    }
    if (f)
        unblock(v, blocked, B);
    else{
        for (unsigned int w : Ak.nodes.at(v)){
            if (B[w].find(v) != B[w].end())
                B[w].insert(v);
        }
    }

    stack.pop_back();
    return f;

}

inline void Graph::unblock(unsigned int u, std::map<unsigned int, bool>& blocked, std::map<unsigned int, std::set<unsigned> >& B) {
    blocked[u] = false;
    assert(B.find(u) != B.end());
    std::set<unsigned> Bu = B[u];
    B[u].clear();
    for (unsigned int w : Bu){
        if (blocked[w])
            unblock(w, blocked, B);
    }
}

inline void Graph::trajanSCC(unsigned int v, unsigned int &index, std::map<unsigned int, int> &nodeToIndex, std::map<unsigned int, unsigned int> &minDist, std::vector<unsigned int> &S, std::vector<std::vector<unsigned int> > &out) {
    nodeToIndex[v] = index;
    minDist[v] = index;
    index++;
    S.push_back(v);
    for (unsigned int w : nodes[v]){
        if (nodeToIndex[w] == -1){
            trajanSCC(w, index, nodeToIndex, minDist, S, out);
            minDist[v] = std::min(minDist[v], minDist[w]);
        }
        else if (std::find(S.begin(), S.end(), w) != S.end()){
            minDist[v] = std::min(minDist[v], minDist[w]);
        }
    }
    if (minDist[v] == (unsigned int)nodeToIndex[v]){
        unsigned int w;
        std::vector<unsigned int> scc;
        do {
            w = S[S.size()-1];
            S.pop_back();
            scc.push_back(w);
        } while (w != v);
        out.push_back(scc);
    }
}

/*bool Graph::circuit(std::vector<unsigned int> &stack, std::map<int, bool> &blocked, std::vector<std::vector<unsigned int>> &circuits, unsigned int v, const Graph &connectedComponent) {
    bool f = false;
    stack.push_back(v);
    blocked[v] = true;
    for (unsigned int w: connectedComponent){

    }


}

void Graph::unblock(std::map<unsigned int, std::set<unsigned int> >& B, std::map<int, bool> &blocked,  int u) {
    blocked[u] = false;
    std::set<unsigned int>::iterator it;
    for (it = B[u].begin(); it != B[u].end(); ){
        int w = *it;
        it++;
        B[u].erase(w);
        if (blocked[w])
            unblock(B, blocked, u); ///??
    }
}*/

#endif // GRAPH_H
