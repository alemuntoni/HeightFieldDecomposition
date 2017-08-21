#ifndef OCTREE_NODE_H
#define OCTREE_NODE_H

#include <vector>
#include <array>
#include <iostream>

#include <cg3/geometry/bounding_box.h>

class OctreeNode {
    public:
        typedef enum  {LBD = 0, RBD, LBU, RBU, LFD, RFD, LFU, RFU, RAD} Position;

        OctreeNode();
        OctreeNode(const cg3::Box &b, const OctreeNode* father = nullptr, Position pos = RAD);
        virtual ~OctreeNode();

        void split(const cg3::Pointd &p);

        OctreeNode* getContainingLeaf(const cg3::Pointd &p);
        void deleteChildren(Position pos);


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
