#pragma once
#include "kimera_dsg/scene_graph_layer.h"

#include <Eigen/Dense>
#include <Eigen/SparseCore>

namespace kimera {

using SparseMatrixXd = Eigen::SparseMatrix<double>;

Eigen::MatrixXd getAdjacencyMatrix(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func);

Eigen::MatrixXd getLaplacian(const SceneGraphLayer& layer,
                             const std::map<NodeId, size_t>& ordering,
                             const std::function<double(NodeId, NodeId)>& weight_func);

inline Eigen::MatrixXd getAdjacencyMatrix(const SceneGraphLayer& layer,
                                          const std::map<NodeId, size_t>& ordering) {
  return getAdjacencyMatrix(layer, ordering, [](NodeId, NodeId) { return 1.0; });
}

inline Eigen::MatrixXd getLaplacian(const SceneGraphLayer& layer,
                                    const std::map<NodeId, size_t>& ordering) {
  return getLaplacian(layer, ordering, [](NodeId, NodeId) { return 1.0; });
}

SparseMatrixXd getSparseAdjacencyMatrix(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func);

SparseMatrixXd getSparseLaplacian(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func);

inline SparseMatrixXd getSparseAdjacencyMatrix(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering) {
  return getSparseAdjacencyMatrix(layer, ordering, [](NodeId, NodeId) { return 1.0; });
}

inline SparseMatrixXd getSparseLaplacian(const SceneGraphLayer& layer,
                                         const std::map<NodeId, size_t>& ordering) {
  return getSparseLaplacian(layer, ordering, [](NodeId, NodeId) { return 1.0; });
}

}  // namespace kimera
