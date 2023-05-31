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
#pragma once
#include <deque>
#include <functional>
#include <list>
#include <unordered_set>

#include "spark_dsg/node_symbol.h"
#include "spark_dsg/scene_graph_types.h"

namespace spark_dsg {
namespace graph_utilities {

// TODO(nathan) make inheritance work
template <typename Graph>
struct graph_traits {};

template <typename Graph, typename NodeSet>
void breadthFirstSearch(const Graph& graph,
                        std::deque<NodeId>& frontier,
                        NodeSet& seen,
                        typename graph_traits<Graph>::visitor callback_function) {
  while (!frontier.empty()) {
    NodeId curr_node = frontier.front();
    frontier.pop_front();

    callback_function(graph, curr_node);

    auto neighbors = graph_traits<Graph>::neighbors(graph, curr_node);
    for (const auto& neighbor : neighbors) {
      if (seen.count(neighbor)) {
        continue;
      }

      frontier.push_back(neighbor);
      seen.insert(neighbor);
    }
  }
}

template <typename Graph, typename NodeSet>
void breadthFirstSearch(const Graph& graph,
                        std::deque<NodeId>& frontier,
                        NodeSet& seen,
                        const NodeSet& valid_nodes,
                        typename graph_traits<Graph>::visitor callback_function) {
  while (!frontier.empty()) {
    NodeId curr_node = frontier.front();
    frontier.pop_front();

    callback_function(graph, curr_node);

    auto neighbors = graph_traits<Graph>::neighbors(graph, curr_node);
    for (const auto& neighbor : neighbors) {
      if (seen.count(neighbor)) {
        continue;
      }

      if (!valid_nodes.count(neighbor)) {
        continue;
      }

      frontier.push_back(neighbor);
      seen.insert(neighbor);
    }
  }
}

template <typename Graph, typename CostMap, bool AllowEqual = false>
void breadthFirstSearch(const Graph& graph,
                        std::deque<NodeId>& frontier,
                        typename CostMap::mapped_type limit,
                        CostMap& costs,
                        typename graph_traits<Graph>::visitor callback_function) {
  while (!frontier.empty()) {
    NodeId curr_node = frontier.front();
    frontier.pop_front();

    callback_function(graph, curr_node);

    if (costs[curr_node] >= limit) {
      continue;
    }

    const typename CostMap::mapped_type new_cost = costs[curr_node] + 1;
    auto neighbors = graph_traits<Graph>::neighbors(graph, curr_node);
    for (const auto& neighbor : neighbors) {
      if (!AllowEqual && costs.count(neighbor) && costs[neighbor] <= new_cost) {
        continue;
      }

      if (AllowEqual && costs.count(neighbor) && costs[neighbor] < new_cost) {
        continue;
      }

      frontier.push_back(neighbor);
      costs[neighbor] = new_cost;
    }
  }
}

template <typename Graph, typename NodeSet = std::unordered_set<NodeId>>
void breadthFirstSearch(const Graph& graph,
                        NodeId root_node,
                        typename graph_traits<Graph>::visitor callback_function) {
  if (!graph_traits<Graph>::contains(graph, root_node)) {
    return;
  }

  std::deque<NodeId> frontier{root_node};

  NodeSet seen{root_node};
  breadthFirstSearch(graph, frontier, seen, callback_function);
}

template <typename Graph, typename CostMap = std::unordered_map<NodeId, size_t>>
void breadthFirstSearch(const Graph& graph,
                        NodeId root_node,
                        size_t limit,
                        typename graph_traits<Graph>::visitor callback_function) {
  if (!graph_traits<Graph>::contains(graph, root_node)) {
    return;
  }

  std::deque<NodeId> frontier{root_node};

  CostMap costs;
  costs[root_node] = 0;
  breadthFirstSearch(graph, frontier, limit, costs, callback_function);
}

template <typename Graph, typename Nodes, typename NodeSet = std::unordered_set<NodeId>>
void breadthFirstSearch(const Graph& graph,
                        Nodes root_nodes,
                        typename graph_traits<Graph>::visitor callback_function) {
  NodeSet seen;
  std::deque<NodeId> frontier;

  // TODO(nathan) consider slightly more permissive policy
  for (const auto root_node : root_nodes) {
    if (!graph_traits<Graph>::contains(graph, root_node)) {
      return;
    }

    frontier.push_back(root_node);
    seen.insert(root_node);
  }

  breadthFirstSearch(graph, frontier, seen, callback_function);
}

template <typename Graph,
          typename Nodes,
          typename CostMap = std::unordered_map<NodeId, size_t>>
void breadthFirstSearch(const Graph& graph,
                        Nodes root_nodes,
                        size_t limit,
                        typename graph_traits<Graph>::visitor callback_function) {
  CostMap costs;
  std::deque<NodeId> frontier;

  // TODO(nathan) consider slightly more permissive policy
  for (const auto root_node : root_nodes) {
    if (!graph_traits<Graph>::contains(graph, root_node)) {
      return;
    }

    costs[root_node] = 0;
    frontier.push_back(root_node);
  }

  breadthFirstSearch(graph, frontier, limit, costs, callback_function);
}

template <typename Graph, typename NodeSet = std::unordered_set<NodeId>>
std::vector<std::vector<NodeId>> getConnectedComponents(const Graph& graph,
                                                        const NodeSet& root_nodes,
                                                        bool only_root_nodes = false) {
  std::vector<std::vector<NodeId>> components;

  NodeSet visited;
  for (const auto& node : root_nodes) {
    if (!graph_traits<Graph>::contains(graph, node)) {
      continue;
    }

    if (visited.count(node)) {
      continue;
    }

    std::vector<NodeId> component;
    std::deque<NodeId> frontier{node};
    visited.insert(node);
    if (only_root_nodes) {
      breadthFirstSearch(
          graph, frontier, visited, root_nodes, [&](const Graph&, NodeId visited) {
            component.push_back(visited);
          });
    } else {
      breadthFirstSearch(graph, frontier, visited, [&](const Graph&, NodeId visited) {
        component.push_back(visited);
      });
    }
    components.push_back(component);
  }

  return components;
}

using Components = std::vector<std::vector<NodeId>>;

template <typename NodeSet>
Components getMergedComponents(const std::vector<NodeSet>& unmerged_components) {
  Components components;

  if (unmerged_components.empty()) {
    return components;
  }

  std::unordered_set<size_t> added_components;
  for (size_t i = 0; i < unmerged_components.size(); ++i) {
    if (added_components.count(i)) {
      continue;
    }

    NodeSet curr_component = unmerged_components.at(i);

    for (size_t j = i + 1; j < unmerged_components.size(); ++j) {
      if (added_components.count(j)) {
        continue;
      }

      for (const auto node : unmerged_components[j]) {
        if (curr_component.count(node)) {
          added_components.insert(j);

          curr_component.insert(unmerged_components.at(j).begin(),
                                unmerged_components.at(j).end());
          break;
        }
      }
    }

    added_components.insert(i);

    std::vector<NodeId> new_component_vec(curr_component.begin(), curr_component.end());
    components.push_back(new_component_vec);
  }

  return components;
}

template <typename Graph, typename NodeSet = std::unordered_set<NodeId>>
Components getConnectedComponents(const Graph& graph,
                                  size_t max_depth,
                                  const NodeSet& root_nodes) {
  std::vector<NodeSet> unmerged_components;

  using CostMap = std::unordered_map<NodeId, size_t>;
  CostMap costs;
  NodeSet component;
  for (const auto& node : root_nodes) {
    if (!graph_traits<Graph>::contains(graph, node)) {
      continue;
    }

    if (!costs.count(node) && !component.empty()) {
      unmerged_components.push_back(component);
      component.clear();
      costs.clear();
    }

    std::deque<NodeId> frontier{node};
    costs[node] = 0;

    breadthFirstSearch<Graph, CostMap, true>(
        graph, frontier, max_depth, costs, [&](const Graph&, NodeId visited) {
          component.insert(visited);
        });
  }

  if (!component.empty()) {
    unmerged_components.push_back(component);
  }

  return getMergedComponents(unmerged_components);
}

template <typename Graph, typename NodeSet>
void breadthFirstSearch(
    const Graph& graph,
    std::deque<NodeId>& frontier,
    NodeSet& seen,
    const typename graph_traits<Graph>::node_valid_func& node_valid,
    const typename graph_traits<Graph>::edge_valid_func& edge_valid,
    const typename graph_traits<Graph>::visitor& callback_function) {
  while (!frontier.empty()) {
    NodeId curr_id = frontier.front();
    frontier.pop_front();

    callback_function(graph, curr_id);

    auto neighbors = graph_traits<Graph>::neighbors(graph, curr_id);
    for (const auto& neighbor : neighbors) {
      if (seen.count(neighbor)) {
        continue;
      }

      const auto& neighbor_node = graph_traits<Graph>::get_node(graph, neighbor);
      if (!node_valid(neighbor_node)) {
        seen.insert(neighbor);  // save some computation
        continue;
      }

      const auto& edge = graph_traits<Graph>::get_edge(graph, curr_id, neighbor);
      if (!edge_valid(edge)) {
        continue;
      }

      frontier.push_back(neighbor);
      seen.insert(neighbor);
    }
  }
}

template <typename Graph, typename NodeSet = std::unordered_set<NodeId>>
std::vector<std::vector<NodeId>> getConnectedComponents(
    const Graph& graph,
    const typename graph_traits<Graph>::node_valid_func& node_valid,
    const typename graph_traits<Graph>::edge_valid_func& edge_valid) {
  std::vector<std::vector<NodeId>> components;

  NodeSet visited;
  for (const auto& node_container : graph_traits<Graph>::nodes(graph)) {
    const auto& node = graph_traits<Graph>::unwrap_node(node_container);
    const NodeId node_id = graph_traits<Graph>::unwrap_node_id(node_container);
    if (!node_valid(node)) {
      continue;
    }

    if (visited.count(node_id)) {
      continue;
    }

    std::vector<NodeId> component;
    std::deque<NodeId> frontier{node_id};
    visited.insert(node_id);
    breadthFirstSearch(
        graph,
        frontier,
        visited,
        node_valid,
        edge_valid,
        [&](const Graph&, NodeId visited) { component.push_back(visited); });
    components.push_back(component);
  }

  return components;
}

}  // namespace graph_utilities
}  // namespace spark_dsg
