#ifndef IGLMESH_H
#define IGLMESH_H

#include <Eigen/Core>
#include "lib/common/serialize.h"


class IGLMesh : public SerializableObject {
    public:
        IGLMesh();
        IGLMesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F);
        void setVertex(unsigned int i, const Eigen::VectorXd &p);
        void addVertex(const Eigen::VectorXd &p);
        void resizeVertices(unsigned int i);

    protected:
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;

};

inline void IGLMesh::setVertex(unsigned int i, const Eigen::VectorXd& p) {
    assert (i < V.rows());
    assert (p.size() == 3);
    V.row(i) =  p;
}

inline void IGLMesh::addVertex(const Eigen::VectorXd& p) {
    assert (p.size() == 3);
    V.resize(V.rows()+1, 3);
    V.row(V.rows()-1) = p;
}

inline void IGLMesh::resizeVertices(unsigned int i) {
    V.resize(i,3);
}

#endif // IGLMESH_H
