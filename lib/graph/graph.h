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

    private:
        //bool circuit(std::vector<unsigned int>& stack, std::vector<std::vector<unsigned int> >& circuits, unsigned int v);
        //void unblock(std::map<unsigned int, std::set<unsigned int> >& B, std::map<int, bool>& blocked, int u);
        std::map<unsigned int, std::vector<unsigned int>> nodes;
        int numberNodes;
};

Graph::Graph() : numberNodes(0){
}

Graph::Graph(unsigned int numberNodes) : numberNodes(numberNodes){
    for (unsigned int i = 0; i < numberNodes; i++)
        nodes[i] = std::vector<unsigned int>();
}

unsigned int Graph::size() const {
    return numberNodes;
}

unsigned int Graph::addNode(int n) {
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

void Graph::addEdge(unsigned int node1, unsigned int node2) {
    assert(nodes.find(node1) != nodes.end());
    assert(nodes.find(node2) != nodes.end());
    nodes[node1].push_back(node2);
}

void Graph::removeNode(unsigned int n) {
    assert(nodes.find(n) != nodes.end());
    nodes.erase(n);
}

void Graph::removeEdge(unsigned int node1, unsigned int node2) {
    assert(nodes.find(node1) != nodes.end());
    assert(nodes.find(node2) != nodes.end());
    std::vector<unsigned int>::iterator it = std::find(nodes[node1].begin(), nodes[node1].end(), node2);
    assert(it != nodes[node1].end());
    nodes[node1].erase(it);
}

bool Graph::removeEdgeIfExists(unsigned int node1, unsigned int node2) {
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

std::vector<unsigned int> Graph::getIncomingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    std::vector<unsigned int> incoming;
    for(std::pair<unsigned int, std::vector<unsigned int> > p : nodes){
        if(std::find(p.second.begin(), p.second.end(), node) != p.second.end())
            incoming.push_back(p.first);
    }
    return incoming;
}

std::vector<unsigned int> Graph::getOutgoingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    return nodes[node];
}

void Graph::deleteAllIncomingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    for (std::pair<unsigned int, std::vector<unsigned int> > p : nodes){
        std::vector<unsigned int>::iterator it = std::find(p.second.begin(), p.second.end(), node);
        if(it != p.second.end())
            nodes[p.first].erase(it);
    }
}

void Graph::deleteAllOutgoingNodes(unsigned int node) {
    assert(nodes.find(node) != nodes.end());
    nodes[node].clear();
}

bool Graph::arcExists(unsigned int n1, unsigned int n2) {
    if (nodes.find(n1) == nodes.end())
        return false;
    else {
        return std::find(nodes[n1].begin(), nodes[n1].end(), n2) != nodes[n1].end();
    }
}

void Graph::visit(std::set<unsigned int>& visitedNodes, unsigned int startingNode) {
    assert(nodes.find(startingNode) != nodes.end());
    visitedNodes.insert(startingNode);
    for(unsigned int adjacent : nodes[startingNode]){
        if (visitedNodes.find(adjacent) == visitedNodes.end()){
            visit(visitedNodes, adjacent);
        }
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
