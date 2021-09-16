#include "kimera_dsg/adjacency_matrix.h"

#include <glog/logging.h>
#include <Eigen/Dense>

namespace kimera {

// TODO(nathan) think about revising this API
// TODO(nathan) think about verifying ordering (to check for duplicates)
Eigen::MatrixXd getAdjacencyMatrix(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(ordering.size(), ordering.size());
  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const SceneGraphNode& node = layer.getNode(id_index_pair.first).value();
    for (const auto& sibling : node.siblings()) {
      if (!ordering.count(sibling)) {
        continue;
      }

      const size_t sibling_index = ordering.at(sibling);
      A(source_index, sibling_index) = weight_func(node.id, sibling);
      A(sibling_index, source_index) = A(source_index, sibling_index);
    }
  }

  return A;
}

Eigen::MatrixXd getLaplacian(const SceneGraphLayer& layer,
                             const std::map<NodeId, size_t>& ordering,
                             const std::function<double(NodeId, NodeId)>& weight_func) {
  Eigen::MatrixXd L = Eigen::MatrixXd::Zero(ordering.size(), ordering.size());
  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const SceneGraphNode& node = layer.getNode(id_index_pair.first).value();

    double degree = 0.0;
    for (const auto& sibling : node.siblings()) {
      if (!ordering.count(sibling)) {
        continue;
      }

      const size_t sibling_index = ordering.at(sibling);
      const double edge_weight = weight_func(node.id, sibling);
      L(source_index, sibling_index) = -edge_weight;
      L(sibling_index, source_index) = -edge_weight;

      degree += edge_weight;
    }
    L(source_index, source_index) = degree;
  }

  return L;
}

SparseMatrixXd getSparseAdjacencyMatrix(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func) {
  using Entry = Eigen::Triplet<double>;
  std::vector<Entry> entries;
  entries.reserve(2 * layer.numEdges());

  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const SceneGraphNode& node = layer.getNode(id_index_pair.first).value();
    for (const auto& sibling : node.siblings()) {
      if (!ordering.count(sibling)) {
        continue;
      }

      const size_t sibling_index = ordering.at(sibling);
      const double edge_weight = weight_func(node.id, sibling);
      entries.push_back(Entry(source_index, sibling_index, edge_weight));
    }
  }

  SparseMatrixXd A(ordering.size(), ordering.size());
  A.setFromTriplets(entries.begin(), entries.end());
  return A;
}

SparseMatrixXd getSparseLaplacian(
    const SceneGraphLayer& layer,
    const std::map<NodeId, size_t>& ordering,
    const std::function<double(NodeId, NodeId)>& weight_func) {
  using Entry = Eigen::Triplet<double>;
  std::vector<Entry> entries;
  entries.reserve(2 * layer.numEdges() + layer.numNodes());

  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const SceneGraphNode& node = layer.getNode(id_index_pair.first).value();

    double degree = 0.0;
    for (const auto& sibling : node.siblings()) {
      if (!ordering.count(sibling)) {
        continue;
      }

      const size_t sibling_index = ordering.at(sibling);
      const double edge_weight = weight_func(node.id, sibling);
      entries.push_back(Entry(source_index, sibling_index, -edge_weight));
      degree += edge_weight;
    }
    entries.push_back(Entry(source_index, source_index, degree));
  }

  SparseMatrixXd L(ordering.size(), ordering.size());
  L.setFromTriplets(entries.begin(), entries.end());
  return L;
}

}  // namespace kimera
