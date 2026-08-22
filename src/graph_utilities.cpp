
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
#include "spark_dsg/graph_utilities.h"

#include <functional>
#include <list>
#include <queue>
#include <unordered_map>
#include <vector>

namespace spark_dsg::graph_utilities {

struct NodeEntry {
  NodeId node;
  double cost;
  std::optional<NodeId> parent;

  bool operator>(const NodeEntry& other) const { return cost > other.cost; };
};

// priority_queue is greatest first, so we flip comparison
using NodeQueue =
    std::priority_queue<NodeEntry, std::vector<NodeEntry>, std::greater<NodeEntry>>;
using ParentMap = std::unordered_map<NodeId, std::pair<NodeId, double>>;

std::vector<NodeId> pathFromParents(const ParentMap& parents,
                                    const NodeId source,
                                    const NodeId target) {
  std::list<NodeId> path{target};
  auto curr_node = target;
  while (curr_node != source) {
    const auto [parent, cost] = parents.at(curr_node);
    path.push_front(parent);
    curr_node = parent;
  }

  return std::vector<NodeId>(path.begin(), path.end());
}

std::vector<NodeId> shortestPath(const SceneGraphLayer& graph,
                                 const NodeId source,
                                 const NodeId target,
                                 const NodeValidFilter& node_valid,
                                 const EdgeValidFilter& edge_valid) {
  if (source == target) {
    return {};
  }

  if (!graph.hasNode(source) && !graph.hasNode(target)) {
    return {};
  }

  const auto goal_pos = graph.getNode(target).attributes().position;

  NodeQueue frontier;
  frontier.push({source, 0.0, std::nullopt});
  std::unordered_map<NodeId, std::pair<NodeId, double>> parents;
  while (!frontier.empty()) {
    const auto [curr_id, curr_cost, parent] = frontier.top();
    frontier.pop();
    if (parent) {
      const auto entry = std::make_pair(*parent, curr_cost);
      auto iter = parents.find(curr_id);
      if (iter == parents.end()) {
        iter = parents.emplace(curr_id, entry).first;
      }

      if (iter->second.second < curr_cost) {
        continue;  // skip invalid node
      }

      iter->second = entry;
    }

    if (curr_id == target) {
      return pathFromParents(parents, source, target);
    }

    const auto& curr_node = graph.getNode(curr_id);
    const auto curr_pos = curr_node.attributes().position;
    for (const auto& neighbor : curr_node.siblings()) {
      const auto& neighbor_node = graph.getNode(neighbor);
      if (node_valid && !node_valid(neighbor_node)) {
        continue;
      }

      const auto& edge = graph.getEdge(curr_id, neighbor);
      if (edge_valid && !edge_valid(edge)) {
        continue;
      }

      const auto neighbor_pos = neighbor_node.attributes().position;
      const double g = curr_cost + (curr_pos - neighbor_pos).norm();
      const double h = (goal_pos - neighbor_pos).norm();
      const auto f = g + h;

      auto iter = parents.find(neighbor);
      if (iter != parents.end() && iter->second.second < f) {
        continue;
      }

      frontier.push({neighbor, f, curr_id});
    }
  }

  return {};
}

}  // namespace spark_dsg::graph_utilities
