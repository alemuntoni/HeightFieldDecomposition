#ifndef AABBCSGTREE_H
#define AABBCSGTREE_H

#include <cg3/geometry/bounding_box.h>

class CSGTreeNode {
    public:
        typedef enum {UNION=0, INTERSECTION, SUBTRACTION, LEAF} NodeType;

        CSGTreeNode();
        CSGTreeNode(const cg3::Box& b);
        CSGTreeNode(const CSGTreeNode& left, const CSGTreeNode& right, NodeType operation);
        CSGTreeNode(CSGTreeNode&& left, CSGTreeNode&& right, NodeType operation);
        CSGTreeNode(const CSGTreeNode& other);
        CSGTreeNode(CSGTreeNode&& other);
        virtual ~CSGTreeNode();

        bool isInside(const cg3::Pointd& p);
        bool isStrictlyInside(const cg3::Pointd &p);

    private:

        NodeType nodeType;
        cg3::Box leafBox;
        CSGTreeNode* leftSubTree;
        CSGTreeNode* rightSubTree;

};

#endif // AABBCSGTREE_H
