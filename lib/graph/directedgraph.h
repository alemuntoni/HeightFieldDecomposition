#ifndef DIRECTEDGRAPH_H
#define DIRECTEDGRAPH_H
#include <vector>
#include <set>

class DirectedGraph {
    public:
        DirectedGraph();
        DirectedGraph(unsigned int numberNodes);
        unsigned int addNode();
        void addEdge(unsigned int node1, unsigned int node2);

        void visit(std::set<unsigned int> &visitedNodes, unsigned int startingNode);


    private:

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

void DirectedGraph::visit(std::set<unsigned int>& visitedNodes, unsigned int startingNode) {
    visitedNodes.insert(startingNode);
    for(unsigned int adjacent : nodes[startingNode]){
        if (visitedNodes.find(adjacent) == visitedNodes.end()){
            visit(visitedNodes, adjacent);
        }
    }
}

#endif // DIRECTEDGRAPH_H
