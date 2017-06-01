#ifndef OCTREE_NODE_H
#define OCTREE_NODE_H

#include <vector>
#include <array>
#include <iostream>

#include <common/bounding_box.h>

class OctreeNode {
    public:
        typedef enum  {LBD = 0, RBD, LBU, RBU, LFD, RFD, LFU, RFU, RAD} Position;

        OctreeNode();
        OctreeNode(const Box &b, const OctreeNode* father = nullptr, Position pos = RAD);
        virtual ~OctreeNode();

        void split(const Pointd &p);

        OctreeNode* getContainingLeaf(const Pointd &p);
        void deleteChildren(Position pos);


    private:

        unsigned int getPosLeaf(const Pointd& p);

        Box box;
        bool splitted;
        Pointd splitPoint;
        std::array<OctreeNode*, 8> children;
        const OctreeNode* father;
        Position position;
};

#endif // OCTREE_NODE_H
