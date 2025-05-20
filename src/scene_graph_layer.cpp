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
#include "spark_dsg/scene_graph_layer.h"

#include <queue>
#include <sstream>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/graph_utilities.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/printing.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;

NodeId GraphMergeConfig::getMergedId(NodeId original) const {
  if (!previous_merges) {
    return original;
  }

  auto iter = previous_merges->find(original);
  return iter == previous_merges->end() ? original : iter->second;
}

bool GraphMergeConfig::shouldUpdateAttributes(LayerKey key) const {
  if (!update_layer_attributes) {
    return true;
  }

  auto iter = update_layer_attributes->find(key.layer);
  if (iter == update_layer_attributes->end()) {
    return true;
  }

  return iter->second;
}

SceneGraphLayer::SceneGraphLayer(LayerKey layer_id) : id(layer_id) {}

SceneGraphLayer::SceneGraphLayer(const std::string& name)
    : id(DsgLayers::nameToLayerId(name).value()) {}

bool SceneGraphLayer::hasNode(NodeId node_id) const {
  return nodes_.count(node_id) != 0;
}

NodeStatus SceneGraphLayer::checkNode(NodeId node_id) const {
  if (nodes_status_.count(node_id) == 0) {
    return NodeStatus::NONEXISTENT;
  }
  return nodes_status_.at(node_id);
}

const Node* SceneGraphLayer::findNode(NodeId node_id) const {
  auto iter = nodes_.find(node_id);
  return iter == nodes_.end() ? nullptr : iter->second.get();
}

const SceneGraphNode& SceneGraphLayer::getNode(NodeId node_id) const {
  const auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
  }

  return *node;
}

bool SceneGraphLayer::emplaceNode(NodeId node_id,
                                  std::unique_ptr<NodeAttributes>&& attrs) {
  nodes_status_[node_id] = NodeStatus::NEW;
  return nodes_.emplace(node_id, std::make_unique<Node>(node_id, id, std::move(attrs)))
      .second;
}

bool SceneGraphLayer::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  // remove all edges connecting to node
  std::set<NodeId> targets_to_erase = nodes_.at(node_id)->siblings_;
  for (const auto& target : targets_to_erase) {
    removeEdge(node_id, target);
  }

  // remove the actual node
  nodes_.erase(node_id);
  nodes_status_[node_id] = NodeStatus::DELETED;
  return true;
}

bool SceneGraphLayer::mergeNodes(NodeId node_from, NodeId node_to) {
  if (!hasNode(node_from) || !hasNode(node_to)) {
    return false;
  }

  if (node_from == node_to) {
    return false;
  }

  // rewire all edges connecting to merged node
  std::set<NodeId> targets_to_rewire = nodes_.at(node_from)->siblings_;
  for (const auto& target : targets_to_rewire) {
    rewireEdge(node_from, target, node_to, target);
  }

  // remove the actual node
  nodes_.erase(node_from);
  nodes_status_[node_from] = NodeStatus::MERGED;
  return true;
}

bool SceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
}

const Edge* SceneGraphLayer::findEdge(NodeId source, NodeId target) const {
  return edges_.find(source, target);
}

const SceneGraphEdge& SceneGraphLayer::getEdge(NodeId source, NodeId target) const {
  const auto edge = findEdge(source, target);
  if (!edge) {
    std::stringstream ss;
    ss << "Missing edge '" << EdgeKey(source, target) << "'";
    throw std::out_of_range(ss.str());
  }

  return *edge;
}

bool SceneGraphLayer::insertEdge(NodeId source,
                                 NodeId target,
                                 std::unique_ptr<EdgeAttributes>&& edge_info) {
  if (source == target) {
    SG_LOG(WARNING) << "Attempted to add a self-edge" << std::endl;
    return false;
  }

  if (hasEdge(source, target)) {
    return false;
  }

  if (!hasNode(source)) {
    // TODO(nathan) maybe consider logging here
    return false;
  }

  if (!hasNode(target)) {
    // TODO(nathan) maybe consider logging here
    return false;
  }

  nodes_[source]->siblings_.insert(target);
  nodes_[target]->siblings_.insert(source);

  edges_.insert(source, target, std::move(edge_info));
  return true;
}

bool SceneGraphLayer::removeEdge(NodeId source, NodeId target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  nodes_[source]->siblings_.erase(target);
  nodes_[target]->siblings_.erase(source);

  edges_.remove(source, target);
  return true;
}

bool SceneGraphLayer::rewireEdge(NodeId source,
                                 NodeId target,
                                 NodeId new_source,
                                 NodeId new_target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  if (!hasNode(new_source) || !hasNode(new_target)) {
    return false;
  }

  if (source == new_source && target == new_target) {
    return false;
  }

  if (new_source == new_target || hasEdge(new_source, new_target)) {
    removeEdge(source, target);
    return true;
  }

  edges_.rewire(source, target, new_source, new_target);

  // rewire siblings
  nodes_[source]->siblings_.erase(target);
  nodes_[target]->siblings_.erase(source);
  nodes_[new_source]->siblings_.insert(new_target);
  nodes_[new_target]->siblings_.insert(new_source);
  return true;
}

void SceneGraphLayer::mergeLayer(const SceneGraphLayer& other_layer,
                                 const GraphMergeConfig& config,
                                 std::vector<NodeId>* new_nodes,
                                 const Eigen::Isometry3d* transform_new_nodes) {
  const bool update_attributes = config.shouldUpdateAttributes(id);
  for (const auto& id_node_pair : other_layer.nodes_) {
    const auto siter = nodes_status_.find(id_node_pair.first);
    if (siter != nodes_status_.end() && siter->second == NodeStatus::MERGED) {
      continue;  // don't try to update or add previously merged nodes
    }

    const auto& other = *id_node_pair.second;
    auto iter = nodes_.find(id_node_pair.first);
    if (iter != nodes_.end()) {
      if (!update_attributes) {
        continue;
      }

      if (!config.update_archived_attributes && !iter->second->attributes_->is_active) {
        continue;
      }

      iter->second->attributes_ = other.attributes_->clone();
      continue;
    }

    auto attrs = other.attributes_->clone();
    if (transform_new_nodes) {
      attrs->transform(*transform_new_nodes);
    }
    nodes_[other.id] = Node::Ptr(new Node(other.id, id, std::move(attrs)));
    nodes_status_[other.id] = NodeStatus::NEW;
    if (new_nodes) {
      new_nodes->push_back(other.id);
    }
  }

  for (const auto& id_edge_pair : other_layer.edges_.edges) {
    const auto& edge = id_edge_pair.second;
    if (hasEdge(edge.source, edge.target)) {
      // TODO(nathan) clone attributes
      continue;
    }

    NodeId new_source = config.getMergedId(edge.source);
    NodeId new_target = config.getMergedId(edge.target);
    if (new_source == new_target) {
      continue;
    }

    insertEdge(new_source, new_target, edge.info->clone());
  }
}

void SceneGraphLayer::getNewNodes(std::vector<NodeId>& new_nodes,
                                  bool clear_new) const {
  auto iter = nodes_status_.begin();
  while (iter != nodes_status_.end()) {
    if (iter->second == NodeStatus::NEW) {
      new_nodes.push_back(iter->first);
      if (clear_new) {
        iter->second = NodeStatus::VISIBLE;
      }
    }

    ++iter;
  }
}

void SceneGraphLayer::getRemovedNodes(std::vector<NodeId>& removed_nodes,
                                      bool clear_removed) const {
  auto iter = nodes_status_.begin();
  while (iter != nodes_status_.end()) {
    if (iter->second != NodeStatus::DELETED && iter->second != NodeStatus::MERGED) {
      ++iter;
      continue;
    }

    removed_nodes.push_back(iter->first);

    if (clear_removed && iter->second == NodeStatus::DELETED) {
      iter = nodes_status_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void SceneGraphLayer::getNewEdges(std::vector<EdgeKey>& new_edges,
                                  bool clear_new) const {
  return edges_.getNew(new_edges, clear_new);
}

void SceneGraphLayer::getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                                      bool clear_removed) const {
  return edges_.getRemoved(removed_edges, clear_removed);
}

void SceneGraphLayer::reset() {
  nodes_.clear();
  nodes_status_.clear();
  edges_.reset();
}

using NodeSet = std::unordered_set<NodeId>;

NodeSet SceneGraphLayer::getNeighborhood(NodeId node, size_t num_hops) const {
  NodeSet result;
  graph_utilities::breadthFirstSearch(
      *this, node, num_hops, [&](const SceneGraphLayer&, NodeId visited) {
        result.insert(visited);
      });
  return result;
}

NodeSet SceneGraphLayer::getNeighborhood(const NodeSet& nodes, size_t num_hops) const {
  NodeSet result;
  graph_utilities::breadthFirstSearch(
      *this, nodes, num_hops, [&](const SceneGraphLayer&, NodeId visited) {
        result.insert(visited);
      });
  return result;
}

void SceneGraphLayer::cloneImpl(SceneGraphLayer& other,
                                const NodeChecker& is_valid) const {
  for (auto&& [id, node] : nodes_) {
    if (is_valid && !is_valid(*node)) {
      continue;
    }

    other.emplaceNode(id, node->attributes().clone());
    other.nodes_status_[id] = nodes_status_.at(id);
  }

  for (const auto& id_edge_pair : edges_.edges) {
    const auto& edge = id_edge_pair.second;
    other.insertEdge(edge.source, edge.target, edge.info->clone());
  }
}

SceneGraphLayer::Ptr SceneGraphLayer::clone(const NodeChecker& is_valid) const {
  SceneGraphLayer::Ptr new_layer(new SceneGraphLayer(id));
  cloneImpl(*new_layer, is_valid);
  return new_layer;
}

void SceneGraphLayer::transform(const Eigen::Isometry3d& transform) {
  for (auto&& [id, node] : nodes_) {
    node->attributes().transform(transform);
  }
}

namespace graph_utilities {

using LayerGraphTraits = graph_traits<SceneGraphLayer>;

std::set<NodeId> LayerGraphTraits::neighbors(const SceneGraphLayer& graph,
                                             NodeId node) {
  return get_node(graph, node).siblings();
}

bool LayerGraphTraits::contains(const SceneGraphLayer& graph, NodeId node) {
  return graph.hasNode(node);
}

const SceneGraphLayer::Nodes& LayerGraphTraits::nodes(const SceneGraphLayer& graph) {
  return graph.nodes();
}

const SceneGraphNode& LayerGraphTraits::unwrap_node(
    const SceneGraphLayer::Nodes::value_type& container) {
  return *container.second;
}

NodeId LayerGraphTraits::unwrap_node_id(
    const SceneGraphLayer::Nodes::value_type& container) {
  return container.first;
}

const SceneGraphNode& LayerGraphTraits::get_node(const SceneGraphLayer& graph,
                                                 NodeId node_id) {
  return graph.getNode(node_id);
}

const SceneGraphEdge& LayerGraphTraits::get_edge(const SceneGraphLayer& graph,
                                                 NodeId source,
                                                 NodeId target) {
  return graph.getEdge(source, target);
}

}  // namespace graph_utilities

}  // namespace spark_dsg
