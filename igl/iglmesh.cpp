#include "iglmesh.h"

IGLMesh::IGLMesh() {
}

IGLMesh::IGLMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) : V(V), F(F){
}
