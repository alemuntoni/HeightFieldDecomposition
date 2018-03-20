#include "octree_node.h"

using namespace cg3;
/**
 * @brief OctreeNode::OctreeNode
 * Creates a node which
 */
OctreeNode::OctreeNode() : splitted(false), father(nullptr), position(RAD){
    for (unsigned int i = 0; i < children.size(); i++)
        children[i] = nullptr;
}

OctreeNode::OctreeNode(const Box& b, const OctreeNode* father, Position pos) : box(b), splitted(false), father(father), position(pos){
    for (unsigned int i = 0; i < children.size(); i++)
        children[i] = nullptr;
}

OctreeNode::~OctreeNode() {
    for (unsigned int i = 0; i < 8; i++){
        if (children[i] != nullptr){
            delete children[i];
        }
    }
}

void OctreeNode::split(const Pointd& p) {
    if (!splitted){
        splitted = true;
        splitPoint = p;
        for (unsigned int i = 0; i < 8; i++){
            Box b;
            if (i & 1){ // x
                b.min().x() = p.x();
                b.max().x() = box.max().x();
            }
            else {
                b.min().x() = box.min().x();
                b.max().x() = p.x() - std::numeric_limits<double>::epsilon();
            }
            if (i & 2){ // y
                b.min().y() = p.y();
                b.max().y() = box.max().y();
            }
            else {
                b.min().y() = box.min().y();
                b.max().y() = p.y() - std::numeric_limits<double>::epsilon();
            }
            if (i & 4){ // z
                b.min().z() = p.z();
                b.max().z() = box.max().z();
            }
            else {
                b.min().z() = box.min().z();
                b.max().z() = p.z() - std::numeric_limits<double>::epsilon();
            }
            children[i] = new OctreeNode(b, this, (Position)i);
        }
    }
    else {
        unsigned int pos = getPosLeaf(p);
        if (children[pos] != nullptr)
            children[pos]->split(p);
    }
}

OctreeNode* OctreeNode::getContainingLeaf(const Pointd& p)  {
    if (! splitted){
        assert(box.isIntern(p));
        return this;
    }
    else {
        unsigned int pos = getPosLeaf(p);
        if (children[pos] == nullptr)
            return nullptr;
        else
            return children[pos]->getContainingLeaf(p);
    }

}

void OctreeNode::deleteChildren(OctreeNode::Position pos) {
    assert(pos < 8);
    if (splitted){
        delete children[pos];
        children[pos] = nullptr;
    }
}

bool OctreeNode::isRadix() const {
    return position == RAD;
}

bool OctreeNode::isLeaf() const {
    return splitted == false;
}

OctreeNode::Position OctreeNode::getNodePosition() const {
    return position;
}

unsigned int OctreeNode::getPosLeaf(const Pointd& p) {
    unsigned int pos = 0;
    pos |= (p.x() < splitPoint.x() ? 0 : 1);
    pos |= (p.y() < splitPoint.y() ? 0 : 2);
    pos |= (p.z() < splitPoint.z() ? 0 : 4);
    return pos;
}
