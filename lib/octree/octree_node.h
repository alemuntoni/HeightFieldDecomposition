#ifndef OCTREE_NODE_H
#define OCTREE_NODE_H

#include <vector>
#include <array>
#include <iostream>

#include <cg3/geometry/bounding_box.h>

/**
 * @brief The OctreeNode class
 *
 * It represents a node of an Octree, defined by:
 * - an axis aligned box;
 * - a position of the box inside the node father;
 * - 8 children if the node is not a leaf;
 * - a split point that generates 8 sub-boxes if the node is not a leaf;
 */
class OctreeNode {
    public:
        //L = left; R = right;
        //B = back; F = front;
        //D = down; U = up;
        //RAD = radix.
        typedef enum  {LBD = 0, RBD, LBU, RBU, LFD, RFD, LFU, RFU, RAD} Position;

        OctreeNode();
        OctreeNode(const cg3::Box &b, const OctreeNode* father = nullptr, Position pos = RAD);
        virtual ~OctreeNode();

        void split(const cg3::Pointd &p);

        OctreeNode* getContainingLeaf(const cg3::Pointd &p);
        void deleteChildren(Position pos);
        bool isRadix() const;
        bool isLeaf() const;
        Position getNodePosition() const;
        const OctreeNode* getFather() const;

    private:

        unsigned int getPosLeaf(const cg3::Pointd& p);

        cg3::Box box;
        bool splitted;
        cg3::Pointd splitPoint;
        std::array<OctreeNode*, 8> children;
        const OctreeNode* father;
        Position position;
};

#endif // OCTREE_NODE_H
