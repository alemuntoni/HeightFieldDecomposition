#ifndef BIPARTITEGRAPHITERATORS_H
#define BIPARTITEGRAPHITERATORS_H

#include "bipartitegraph.h"

template <class T1, class T2>
class BipartiteGraph<T1, T2>::AdjacentUNodeIterator{
        friend class BipartiteGraph;
    public:
        AdjacentUNodeIterator();

        const T2& operator *() const;
        bool operator == (const AdjacentUNodeIterator& otherIterator) const;
        bool operator != (const AdjacentUNodeIterator& otherIterator) const;

        AdjacentUNodeIterator operator ++ ();
        AdjacentUNodeIterator operator ++ (int);
        AdjacentUNodeIterator operator -- ();
        AdjacentUNodeIterator operator -- (int);

    protected:
        const BipartiteGraph* g;
        std::set<unsigned int>::iterator pos;
        AdjacentUNodeIterator(std::set<unsigned int>::iterator it, const BipartiteGraph &g);


};

template <class T1, class T2>
class BipartiteGraph<T1, T2>::AdjacentVNodeIterator{
        friend class BipartiteGraph;
    public:
        AdjacentVNodeIterator();

        const T1& operator *() const;
        bool operator == (const AdjacentVNodeIterator& otherIterator) const;
        bool operator != (const AdjacentVNodeIterator& otherIterator) const;

        AdjacentVNodeIterator operator ++ ();
        AdjacentVNodeIterator operator ++ (int);
        AdjacentVNodeIterator operator -- ();
        AdjacentVNodeIterator operator -- (int);

    protected:
        const BipartiteGraph* g;
        std::set<unsigned int>::iterator pos;
        AdjacentVNodeIterator(std::set<unsigned int>::iterator it, const BipartiteGraph &g);


};

template <class T1, class T2>
class BipartiteGraph<T1, T2>::AdjacentUNodeRangeBasedIterator {
        friend class BipartiteGraph;
    public:
        BipartiteGraph<T1, T2>::AdjacentUNodeIterator begin() const;
        BipartiteGraph<T1, T2>::AdjacentUNodeIterator end() const;
    protected:
        const BipartiteGraph* g;
        const T1& uNode;
        AdjacentUNodeRangeBasedIterator(const BipartiteGraph* g, const T1& uNode);
};

template <class T1, class T2>
class BipartiteGraph<T1, T2>::AdjacentVNodeRangeBasedIterator {
        friend class BipartiteGraph;
    public:
        BipartiteGraph<T1, T2>::AdjacentVNodeIterator begin() const;
        BipartiteGraph<T1, T2>::AdjacentVNodeIterator end() const;
    protected:
        const BipartiteGraph* g;
        const T2& vNode;
        AdjacentVNodeRangeBasedIterator(const BipartiteGraph* g, const T2& vNode);
};

//ConstAdjacentUNodeIterator

template <class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentUNodeIterator::AdjacentUNodeIterator() : g(nullptr) {
}

template <class T1, class T2>
const T2& BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator *() const {
    return g->nodesV[*pos].getInfo();
}

template <class T1, class T2>
bool BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator ==(const BipartiteGraph<T1, T2>::AdjacentUNodeIterator& otherIterator) const {
    return (g == otherIterator.g && pos == otherIterator.pos);
}

template <class T1, class T2>
bool BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator !=(const BipartiteGraph<T1, T2>::AdjacentUNodeIterator& otherIterator) const {
    return (g != otherIterator.g || pos != otherIterator.pos);
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator ++() {
    ++pos;
    return *this;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator ++(int) {
    AdjacentUNodeIterator oldIt = *this;
    ++pos;
    return oldIt;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator --() {
    --pos;
    return *this;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeIterator::operator --(int) {
    AdjacentUNodeIterator oldIt;
    --pos;
    return oldIt;
}

template <class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentUNodeIterator::AdjacentUNodeIterator(std::set<unsigned int>::iterator it, const BipartiteGraph& g): g(&g), pos(it){
}

//ConstAdjacentUNodeIterator

template <class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentVNodeIterator::AdjacentVNodeIterator() : g(nullptr) {
}

template <class T1, class T2>
const T1& BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator *() const {
    return g->nodesU[*pos].getInfo();
}

template <class T1, class T2>
bool BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator ==(const BipartiteGraph<T1, T2>::AdjacentVNodeIterator& otherIterator) const {
    return (g == otherIterator.g && pos == otherIterator.pos);
}

template <class T1, class T2>
bool BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator !=(const BipartiteGraph<T1, T2>::AdjacentVNodeIterator& otherIterator) const {
    return (g != otherIterator.g || pos != otherIterator.pos);
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator ++() {
    ++pos;
    return *this;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator ++(int) {
    AdjacentVNodeIterator oldIt = *this;
    ++pos;
    return oldIt;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator --() {
    --pos;
    return *this;
}

template <class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeIterator::operator --(int) {
    AdjacentVNodeIterator oldIt;
    --pos;
    return oldIt;
}

template <class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentVNodeIterator::AdjacentVNodeIterator(std::set<unsigned int>::iterator it, const BipartiteGraph& g): g(&g), pos(it){
}

//ConstAdjacentUNodeRangeBasedIterator

template<class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeRangeBasedIterator::begin() const {
    return g->adjacentUNodeBegin(uNode);
}

template<class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentUNodeIterator BipartiteGraph<T1, T2>::AdjacentUNodeRangeBasedIterator::end() const {
    return g->adjacentUNodeEnd(uNode);
}

template<class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentUNodeRangeBasedIterator::AdjacentUNodeRangeBasedIterator(const BipartiteGraph* g, const T1& uNode) : g(g), uNode(uNode) {
}

//ConstAdjacentVNodeRangeBasedIterator

template<class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeRangeBasedIterator::begin() const {
    return g->adjacentVNodeBegin(vNode);
}

template<class T1, class T2>
typename BipartiteGraph<T1, T2>::AdjacentVNodeIterator BipartiteGraph<T1, T2>::AdjacentVNodeRangeBasedIterator::end() const {
    return g->adjacentVNodeEnd(vNode);
}

template<class T1, class T2>
BipartiteGraph<T1, T2>::AdjacentVNodeRangeBasedIterator::AdjacentVNodeRangeBasedIterator(const BipartiteGraph* g, const T2& vNode) : g(g), vNode(vNode) {
}

#endif // BIPARTITEGRAPHITERATORS_H
