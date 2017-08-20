#include "aabbcsgtree.h"

//Node

using namespace cg3;

CSGTreeNode::CSGTreeNode() {
}

CSGTreeNode::CSGTreeNode(const Box& b) : nodeType(LEAF), leafBox(b), leftSubTree(nullptr), rightSubTree(nullptr){
}

CSGTreeNode::CSGTreeNode(const CSGTreeNode& left, const CSGTreeNode& right, CSGTreeNode::NodeType operation) : nodeType(operation) {
    leftSubTree = new CSGTreeNode(left);
    rightSubTree = new CSGTreeNode(right);
}

CSGTreeNode::CSGTreeNode(CSGTreeNode&& left, CSGTreeNode&& right, CSGTreeNode::NodeType operation) : nodeType(operation){
    leftSubTree = new CSGTreeNode(left);
    rightSubTree = new CSGTreeNode(right);
}

CSGTreeNode::CSGTreeNode(const CSGTreeNode& other) {
    if (other.nodeType == LEAF){
        leftSubTree = nullptr;
        rightSubTree = nullptr;
        nodeType = LEAF;
        leafBox = other.leafBox;
    }
    else {
        nodeType = other.nodeType;
        leftSubTree = new CSGTreeNode(*(other.leftSubTree));
        rightSubTree = new CSGTreeNode(*(other.rightSubTree));
    }
}

CSGTreeNode::CSGTreeNode(CSGTreeNode&& other) : nodeType(other.nodeType), leafBox(other.leafBox){
    leftSubTree = other.leftSubTree;
    rightSubTree = other.rightSubTree;
    other.leftSubTree = nullptr;
    other.rightSubTree = nullptr;
}

CSGTreeNode::~CSGTreeNode() {
    delete leftSubTree;
    delete rightSubTree;
}

bool CSGTreeNode::isInside(const Pointd& p) {
    switch (nodeType){
        case LEAF:
            return leafBox.isInside(p);
        case UNION:
            return leftSubTree->isInside(p) || rightSubTree->isInside(p);
        case INTERSECTION:
            return leftSubTree->isInside(p) && rightSubTree->isInside(p);
        case SUBTRACTION:
            return leftSubTree->isInside(p) && !(rightSubTree->isStrictlyInside(p));
        default:
            assert(false && "Unknown Node Type");
            return false;
    }
}

bool CSGTreeNode::isStrictlyInside(const Pointd& p) {
    switch (nodeType){
        case LEAF:
            return leafBox.isStrictlyInside(p);
        case UNION:
            return leftSubTree->isStrictlyInside(p) || rightSubTree->isStrictlyInside(p);
        case INTERSECTION:
            return leftSubTree->isStrictlyInside(p) && rightSubTree->isStrictlyInside(p);
        case SUBTRACTION:
            return leftSubTree->isStrictlyInside(p) && !(rightSubTree->isInside(p));
        default:
            assert(false && "Unknown Node Type");
            return false;
    }
}
