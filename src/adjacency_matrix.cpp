/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "spark_dsg/adjacency_matrix.h"

#include <Eigen/Dense>

namespace spark_dsg {

// TODO(nathan) think about revising this API
// TODO(nathan) think about verifying ordering (to check for duplicates)
Eigen::MatrixXd getAdjacencyMatrix(const SceneGraphLayer& layer,
                                   const std::map<NodeId, size_t>& ordering,
                                   const WeightFunc& weight_func) {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(ordering.size(), ordering.size());
  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const auto& node = layer.getNode(id_index_pair.first);
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
                             const WeightFunc& weight_func) {
  Eigen::MatrixXd L = Eigen::MatrixXd::Zero(ordering.size(), ordering.size());
  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const auto& node = layer.getNode(id_index_pair.first);

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

SparseMatrixXd getSparseAdjacencyMatrix(const SceneGraphLayer& layer,
                                        const std::map<NodeId, size_t>& ordering,
                                        const WeightFunc& weight_func) {
  using Entry = Eigen::Triplet<double>;
  std::vector<Entry> entries;
  entries.reserve(2 * layer.numEdges());

  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const auto& node = layer.getNode(id_index_pair.first);
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

SparseMatrixXd getSparseLaplacian(const SceneGraphLayer& layer,
                                  const std::map<NodeId, size_t>& ordering,
                                  const WeightFunc& weight_func) {
  using Entry = Eigen::Triplet<double>;
  std::vector<Entry> entries;
  entries.reserve(2 * layer.numEdges() + layer.numNodes());

  for (const auto& id_index_pair : ordering) {
    const size_t source_index = id_index_pair.second;
    const auto& node = layer.getNode(id_index_pair.first);

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

}  // namespace spark_dsg
