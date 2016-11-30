#ifndef DIRECTEDGRAPH_H
#define DIRECTEDGRAPH_H
#include <vector>
#include <set>
#include <queue>
#include <assert.h>

class DirectedGraph {
    public:
        DirectedGraph();
        DirectedGraph(unsigned int numberNodes);
        unsigned int addNode();
        void addEdge(unsigned int node1, unsigned int node2);
        void removeEdge(unsigned int node1, unsigned int node2);
        std::vector<unsigned int> getIncomingNodes(unsigned int node);
        std::vector<unsigned int> getOutgoingNodes(unsigned int node);

        void visit(std::set<unsigned int> &visitedNodes, unsigned int startingNode);
        std::vector< std::vector<unsigned int> > getSpanningTree(unsigned int radix);
        void getLoops(std::vector< std::vector<unsigned int> >& loops);


    private:
        std::vector<std::vector <unsigned int> > spanningTree(std::vector<unsigned int> start, unsigned int thisNode, unsigned int radix);
        std::vector< std::vector<unsigned int> > nodes;
};

DirectedGraph::DirectedGraph() {
}

DirectedGraph::DirectedGraph(unsigned int numberNodes) {
    nodes.resize(numberNodes);
}

unsigned int DirectedGraph::addNode() {
    nodes.push_back(std::vector<unsigned int>());
    return nodes.size()-1;
}

void DirectedGraph::addEdge(unsigned int node1, unsigned int node2) {
    assert(node1 < nodes.size());
    assert(node2 < nodes.size());
    nodes[node1].push_back(node2);
}

void DirectedGraph::removeEdge(unsigned int node1, unsigned int node2) {
    assert(node1 < nodes.size());
    assert(node2 < nodes.size());
    std::vector<unsigned int>::iterator it = std::find(nodes[node1].begin(), nodes[node1].end(), node2);
    assert(it != nodes[node1].end());
    if (it != nodes[node1].end()){
        nodes[node1].erase(it);
    }
}

std::vector<unsigned int> DirectedGraph::getIncomingNodes(unsigned int node) {
    assert(node < nodes.size());
    std::vector<unsigned int> incoming;
    for (unsigned int i = 0; i < nodes.size(); i++){
        if(std::find(nodes[i].begin(), nodes[i].end(), node) != nodes[i].end())
            incoming.push_back(i);
    }
    return incoming;
}

std::vector<unsigned int> DirectedGraph::getOutgoingNodes(unsigned int node) {
    assert(node < nodes.size());
    return nodes[node];
}

void DirectedGraph::visit(std::set<unsigned int>& visitedNodes, unsigned int startingNode) {
    visitedNodes.insert(startingNode);
    for(unsigned int adjacent : nodes[startingNode]){
        if (visitedNodes.find(adjacent) == visitedNodes.end()){
            visit(visitedNodes, adjacent);
        }
    }
}

std::vector<std::vector<unsigned int> > DirectedGraph::getSpanningTree(unsigned int radix) {
    std::vector<unsigned int> start;
    start.push_back(radix);
    std::vector<std::vector<unsigned int>> ret;
    if (nodes[radix].size() == 0){
        ret.push_back(start);
        return ret;
    }
    for(unsigned int adjacent : nodes[radix]) {
        std::vector<std::vector<unsigned int> > tmp = spanningTree(start, adjacent, radix);
        ret.insert( ret.end(), tmp.begin(), tmp.end() );
    }
    return ret;
}

void DirectedGraph::getLoops(std::vector<std::vector<unsigned int> >& loops) {
    loops.clear();
    std::vector< std::vector <unsigned int> > tmp;
    for (unsigned int node = 0; node < nodes.size(); ++node){
        tmp = getSpanningTree(node);
        for (std::vector<unsigned int> loop : tmp){
            if (loop.size() > 1 && loop[0] == loop[loop.size()-1]){
                loops.push_back(loop);
            }
        }
    }
}

std::vector<std::vector<unsigned int> > DirectedGraph::spanningTree(std::vector<unsigned int> start, unsigned int thisNode, unsigned int radix) {
    start.push_back(thisNode);
    std::vector<std::vector<unsigned int>> ret;
    if (nodes[thisNode].size() == 0 || thisNode == radix){
        ret.push_back(start);
        return ret;
    }
    else {
        for(unsigned int adjacent : nodes[thisNode]) {
            if (adjacent == radix || std::find(start.begin(), start.end(), adjacent) == start.end()){
                std::vector<std::vector<unsigned int> > tmp = spanningTree(start, adjacent, radix);
                ret.insert( ret.end(), tmp.begin(), tmp.end() );
            }
        }
        return ret;
    }
}

#endif // DIRECTEDGRAPH_H
