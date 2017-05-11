#ifndef AABBCSGTREE_H
#define AABBCSGTREE_H

#include <common/bounding_box.h>

class CSGTreeNode {
    public:
        typedef enum {UNION=0, INTERSECTION, SUBTRACTION, LEAF} NodeType;

        CSGTreeNode();
        CSGTreeNode(const Box& b);
        CSGTreeNode(const CSGTreeNode& left, const CSGTreeNode& right, NodeType operation);
        CSGTreeNode(CSGTreeNode&& left, CSGTreeNode&& right, NodeType operation);
        CSGTreeNode(const CSGTreeNode& other);
        CSGTreeNode(CSGTreeNode&& other);
        virtual ~CSGTreeNode();

        bool isInside(const Pointd& p);
        bool isStrictlyInside(const Pointd &p);

    private:

        NodeType nodeType;
        Box leafBox;
        CSGTreeNode* leftSubTree;
        CSGTreeNode* rightSubTree;

};

#endif // AABBCSGTREE_H
