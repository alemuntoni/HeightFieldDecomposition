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
        std::vector<std::vector<unsigned int> > getStronglyConnectedComponents();
        std::vector<std::vector<unsigned int> > getCircuits();

    private:
        void trajanSCC(unsigned int v, unsigned int& index, std::map<unsigned int, int>& nodeToIndex, std::map<unsigned int, unsigned int>& minDist, std::vector<unsigned int>& S, std::vector<std::vector<unsigned int> >& out);
        std::map<unsigned int, std::vector<unsigned int>> nodes;
        int numberNodes;
};

inline Graph::Graph() : numberNodes(0){
}

inline Graph::Graph(unsigned int numberNodes) : numberNodes(numberNodes){
    for (unsigned int i = 0; i < numberNodes; i++)
        nodes[i] = std::vector<unsigned int>();
}

inline unsigned int Graph::size() const {
    return numberNodes;
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
    for (std::pair<unsigned int, std::vector<unsigned int> > p : nodes){
        std::vector<unsigned int>::iterator it = std::find(p.second.begin(), p.second.end(), node);
        if(it != p.second.end())
            nodes[p.first].erase(it);
    }
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
    std::map<unsigned int, bool> blocked;
    std::map<unsigned int, std::set<unsigned int> > B;
    std::vector< std::vector<unsigned int> > scc = getStronglyConnectedComponents();
    for (unsigned int k = 0; k < scc.size(); k++){

    }
    return std::vector<std::vector<unsigned int> >();
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
