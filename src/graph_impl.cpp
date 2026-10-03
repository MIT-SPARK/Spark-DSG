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
#include "spark_dsg/graph_impl.h"

#include <sstream>

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

GraphImpl::GraphImpl() = default;

GraphImpl::~GraphImpl() = default;

void GraphImpl::clear() {
  nodes_.clear();
  node_status_.clear();
  edges_.clear();
  edge_status_.clear();
}

GraphImpl::Ptr GraphImpl::clone() const {
  auto other = std::make_shared<GraphImpl>();
  for (auto&& [id, node] : nodes_) {
    other->emplace(node->layer, id, node->attributes().clone());
  }

  for (const auto& [key, edge] : edges_) {
    other->connect(edge->source, edge->target, edge->info->clone());
  }

  other->node_status_ = node_status_;
  other->edge_status_ = edge_status_;
  return other;
}

void GraphImpl::transform(const Eigen::Isometry3d& transform) {
  for (auto&& [id, node] : nodes_) {
    node->attributes().transform(transform);
  }
}

void GraphImpl::merge(const GraphImpl& other,
                      const GraphMergeConfig& config,
                      const Eigen::Isometry3d* new_node_transform) {
  for (const auto& node_id : other.removed_nodes(config.clear_removed)) {
    remove(node_id);
  }

  for (const auto& key : other.removed_edges(config.clear_removed)) {
    remove(key.k1, key.k2);
  }

  for (const auto& [node_id, node] : other.nodes_) {
    const auto siter = node_status_.find(node_id);
    if (siter != node_status_.end() && siter->second == NodeStatus::MERGED) {
      continue;  // don't try to update or add previously merged nodes
    }

    auto iter = nodes_.find(node_id);
    if (iter != nodes_.end()) {
      if (!config.update_archived_attributes && !iter->second->attributes_->is_active) {
        continue;
      }

      iter->second->attributes_ = node->attributes_->clone();
      continue;
    }

    auto attrs = node->attributes_->clone();
    if (new_node_transform) {
      attrs->transform(*new_node_transform);
    }

    emplace(node->layer, node_id, std::move(attrs));
  }

  for (const auto& [key, edge] : other.edges_) {
    const auto prev_edge = edges_.find(key);
    if (prev_edge != edges_.end()) {
      // Overwrite existing edge attributes if they already exist.
      prev_edge->second->info = edge->info->clone();
      continue;
    }

    const auto new_src = config.getMergedId(edge->source);
    const auto new_tgt = config.getMergedId(edge->target);
    if (new_src == new_tgt) {
      continue;
    }

    connect(new_src, new_tgt, edge->info->clone(), config.enforce_parent_constraints);
  }
}

size_t GraphImpl::num_nodes() const { return nodes_.size(); }

size_t GraphImpl::num_edges() const { return edges_.size(); }

size_t GraphImpl::memory_usage() const {
  size_t total_memory = sizeof(*this);

  // Estimate memory usage of nodes.
  total_memory += nodes_.size() * (sizeof(NodeId) + sizeof(Node::Ptr));
  for (const auto& [node_id, node] : nodes_) {
    total_memory += node->memoryUsage();
  }

  // Edges and attributes.
  total_memory += edges_.size() * sizeof(EdgeKey);
  for (const auto& [key, edge] : edges_) {
    total_memory += sizeof(SceneGraphEdge);
    if (edge->info) {
      total_memory += edge->info->memoryUsage();
    }
  }

  // Estimate memory usage of status maps.
  total_memory += node_status_.size() * (sizeof(NodeId) + sizeof(NodeStatus));
  total_memory += edge_status_.size() * (sizeof(EdgeKey) + sizeof(EdgeStatus));
  return total_memory;
}

bool GraphImpl::has(NodeId node_id) const { return nodes_.count(node_id) != 0; }

bool GraphImpl::has(NodeId source, NodeId target) const {
  return edges_.count(EdgeKey{source, target});
}

NodeStatus GraphImpl::status(NodeId node_id) const {
  auto iter = node_status_.find(node_id);
  return iter == node_status_.end() ? NodeStatus::NONEXISTENT : iter->second;
}

EdgeStatus GraphImpl::status(NodeId source, NodeId target) const {
  auto iter = edge_status_.find(EdgeKey{source, target});
  return iter == edge_status_.end() ? EdgeStatus::NONEXISTENT : iter->second;
}

const Node* GraphImpl::find(NodeId node_id) const {
  auto iter = nodes_.find(node_id);
  return iter == nodes_.end() ? nullptr : iter->second.get();
}

const Edge* GraphImpl::find(NodeId source, NodeId target) const {
  auto iter = edges_.find(EdgeKey{source, target});
  return iter == edges_.end() ? nullptr : iter->second.get();
}

const SceneGraphNode& GraphImpl::get(NodeId node_id) const {
  const auto node = find(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
  }

  return *node;
}

const SceneGraphEdge& GraphImpl::get(NodeId source, NodeId target) const {
  const auto edge = find(source, target);
  if (!edge) {
    std::stringstream ss;
    ss << "Missing edge '" << EdgeKey(source, target) << "'";
    throw std::out_of_range(ss.str());
  }

  return *edge;
}

bool GraphImpl::emplace(LayerKey layer,
                        NodeId node_id,
                        std::unique_ptr<NodeAttributes>&& attrs) {
  auto node = std::make_unique<Node>(node_id, layer, std::move(attrs));
  auto emplaced = nodes_.emplace(node_id, std::move(node)).second;
  if (emplaced) {
    node_status_[node_id] = NodeStatus::NEW;
  }

  return emplaced;
}

bool GraphImpl::update(LayerKey layer,
                       NodeId node_id,
                       std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    emplace(layer, node_id, std::move(attrs));
    return true;
  }

  if (iter->second->layer != layer) {
    return false;
  }

  iter->second->attributes_ = std::move(attrs);
  return true;
}

bool GraphImpl::set(NodeId node_id, std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return false;
  }

  iter->second->attributes_ = std::move(attrs);
  return true;
}

void GraphImpl::drop_parents(SceneGraphNode& source, SceneGraphNode& target) {
  // force single parent to exist
  const auto source_is_parent = source.layer.isParentOf(target.layer);
  const auto to_clear = source_is_parent ? target.parents_ : source.parents_;
  NodeId child = source_is_parent ? target.id : source.id;
  for (const auto parent_to_clear : to_clear) {
    remove(child, parent_to_clear);
  }
}

bool GraphImpl::connect(NodeId source,
                        NodeId target,
                        std::unique_ptr<EdgeAttributes>&& attrs,
                        bool enforce_parent_constraints) {
  if (source == target) {
    return false;
  }

  if (has(source, target)) {
    return false;
  }

  auto source_iter = nodes_.find(source);
  auto target_iter = nodes_.find(target);
  if (source_iter == nodes_.end() || target_iter == nodes_.end()) {
    return false;
  }

  auto& source_node = *source_iter->second;
  auto& target_node = *target_iter->second;
  const auto same_layer = source_node.layer.layer == target_node.layer.layer;
  if (enforce_parent_constraints && !same_layer) {
    drop_parents(source_node, target_node);
  }

  source_node.addConnection(target_node);
  target_node.addConnection(source_node);

  const EdgeKey key{source, target};
  auto edge = std::make_unique<Edge>(source, target, std::move(attrs));
  auto emplaced = edges_.emplace(std::make_pair(key, std::move(edge))).second;
  if (emplaced) {
    edge_status_[key] = EdgeStatus::NEW;
  }

  return emplaced;
}

bool GraphImpl::remove(NodeId node_id) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return false;
  }

  const auto connections = iter->second->connections();
  for (const auto target : connections) {
    const EdgeKey key{node_id, target};
    nodes_.at(target)->removeConnection(*iter->second);
    edges_.erase(key);
    edge_status_.erase(key);
  }

  // remove the actual node
  nodes_.erase(node_id);
  node_status_[node_id] = NodeStatus::DELETED;
  return true;
}

bool GraphImpl::remove(NodeId source, NodeId target) {
  const EdgeKey key{source, target};
  auto iter = edges_.find(key);
  if (iter == edges_.end()) {
    return false;
  }

  auto& source_node = nodes_.at(source);
  auto& target_node = nodes_.at(target);
  source_node->removeConnection(*target_node);
  target_node->removeConnection(*source_node);
  edge_status_[key] = EdgeStatus::DELETED;
  edges_.erase(iter);
  return true;
}

bool GraphImpl::contract(NodeId node_from, NodeId node_to) {
  if (node_from == node_to) {
    return false;
  }

  auto from = nodes_.find(node_from);
  auto to = nodes_.find(node_to);
  if (from == nodes_.end() || to == nodes_.end()) {
    return false;
  }

  // rewire all edges connecting to merged node
  const auto targets_to_rewire = from->second->connections();
  for (const auto target : targets_to_rewire) {
    if (target == node_to) {
      const EdgeKey key{node_from, node_to};
      edges_.erase(key);
      edge_status_[key] = EdgeStatus::DELETED;
      to->second->removeConnection(*from->second);
      continue;  // no self edges from contraction
    }

    // update ancestry for new connection
    auto& target_node = *nodes_.at(target);
    from->second->removeConnection(target_node);
    to->second->addConnection(target_node);

    const EdgeKey prev_key{node_from, target};
    const EdgeKey new_key{node_to, target};

    auto prev = edges_.find(prev_key);
    auto attrs = prev->second->info->clone();
    edges_.erase(prev);

    auto edge = std::make_unique<Edge>(node_to, target, std::move(attrs));
    edges_.emplace(new_key, std::move(edge));
    edge_status_[new_key] = edge_status_.at(prev_key);
    edge_status_[prev_key] = EdgeStatus::DELETED;
  }

  // TODO(nathan) push this to extra storage
  nodes_.erase(from);
  node_status_[node_from] = NodeStatus::MERGED;
  return true;
}

std::vector<NodeId> GraphImpl::new_nodes(bool clear_new) const {
  std::vector<NodeId> to_return;
  for (auto& [node_id, status] : node_status_) {
    if (status == NodeStatus::NEW) {
      to_return.push_back(node_id);
      if (clear_new) {
        status = NodeStatus::PRESENT;
      }
    }
  }

  return to_return;
}

std::vector<NodeId> GraphImpl::removed_nodes(bool clear_removed) const {
  std::vector<NodeId> removed;
  auto iter = node_status_.begin();
  while (iter != node_status_.end()) {
    if (iter->second != NodeStatus::DELETED && iter->second != NodeStatus::MERGED) {
      ++iter;
      continue;
    }

    removed.push_back(iter->first);

    if (clear_removed && iter->second == NodeStatus::DELETED) {
      iter = node_status_.erase(iter);
    } else {
      ++iter;
    }
  }

  return removed;
}

std::vector<EdgeKey> GraphImpl::new_edges(bool clear_new) const {
  std::vector<EdgeKey> to_return;
  for (auto& [key, status] : edge_status_) {
    if (status == EdgeStatus::NEW) {
      to_return.push_back(key);
      if (clear_new) {
        status = EdgeStatus::PRESENT;
      }
    }
  }

  return to_return;
}

std::vector<EdgeKey> GraphImpl::removed_edges(bool clear_removed) const {
  std::vector<EdgeKey> removed;
  auto iter = edge_status_.begin();
  while (iter != edge_status_.end()) {
    if (iter->second != EdgeStatus::DELETED) {
      ++iter;
      continue;
    }

    removed.push_back(iter->first);
    if (clear_removed) {
      iter = edge_status_.erase(iter);
    } else {
      ++iter;
    }
  }

  return removed;
}

}  // namespace spark_dsg
