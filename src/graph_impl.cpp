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

GraphImpl::GraphImpl() = default;

GraphImpl::~GraphImpl() = default;

size_t GraphImpl::num_nodes() const { return nodes_.size(); }

size_t GraphImpl::num_edges() const { return edges_.size(); }

bool GraphImpl::has(NodeId node_id) const { return nodes_.count(node_id) != 0; }

bool GraphImpl::has(NodeId source, NodeId target) const {
  return edges_.count(EdgeKey{source, target});
}

_NodeStatus GraphImpl::status(NodeId node_id) const {
  auto iter = node_status_.find(node_id);
  return iter == node_status_.end() ? _NodeStatus::NONEXISTENT : iter->second;
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
  return iter == edges_.end() ? nullptr : &iter->second;
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
    node_status_[node_id] = _NodeStatus::NEW;
  }

  return emplaced;
}

bool GraphImpl::connect(NodeId source,
                        NodeId target,
                        std::unique_ptr<EdgeAttributes>&& attrs) {
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

  const EdgeKey key{source, target};
  source_iter->second->addConnection(*target_iter->second);
  target_iter->second->addConnection(*source_iter->second);

  // TODO(nathan) drop piecewise_construct when edge iterator implemented
  edges_.emplace(std::piecewise_construct,
                 std::forward_as_tuple(key),
                 std::forward_as_tuple(source, target, std::move(attrs)));
  edge_status_[key] = EdgeStatus::NEW;
  return true;
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
  node_status_[node_id] = _NodeStatus::DELETED;
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
    auto prev = edges_.find(prev_key);
    auto attrs = prev->second.info->clone();
    edges_.erase(prev);

    const EdgeKey new_key{node_to, target};
    edges_.emplace(std::piecewise_construct,
                   std::forward_as_tuple(new_key),
                   std::forward_as_tuple(node_to, target, std::move(attrs)));
    edge_status_[new_key] = edge_status_.at(prev_key);
    edge_status_[prev_key] = EdgeStatus::DELETED;
  }

  // TODO(nathan) push this to extra storage
  nodes_.erase(from);
  node_status_[node_from] = _NodeStatus::MERGED;
  return true;
}

void GraphImpl::reset() {
  nodes_.clear();
  node_status_.clear();
  edges_.clear();
  edge_status_.clear();
  stale_edges_.clear();
}

GraphImpl::Ptr GraphImpl::clone() const {
  auto other = std::make_shared<GraphImpl>();
  for (auto&& [id, node] : nodes_) {
    other->emplace(node->layer, id, node->attributes().clone());
  }

  other->node_status_ = node_status_;

  for (const auto& [key, edge] : edges_) {
    other->connect(edge.source, edge.target, edge.info->clone());
  }

  return other;
}

void GraphImpl::transform(const Eigen::Isometry3d& transform) {
  for (auto&& [id, node] : nodes_) {
    node->attributes().transform(transform);
  }
}

size_t GraphImpl::memoryUsage() const {
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
    if (edge.info) {
      total_memory += edge.info->memoryUsage();
    }
  }

  // Estimate memory usage of status maps.
  total_memory += node_status_.size() * (sizeof(NodeId) + sizeof(_NodeStatus));
  total_memory += edge_status_.size() * (sizeof(EdgeKey) + sizeof(EdgeStatus));
  return total_memory;
}

}  // namespace spark_dsg
