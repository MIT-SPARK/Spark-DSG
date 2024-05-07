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
#include "spark_dsg/dynamic_scene_graph_layer.h"

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;

DynamicSceneGraphLayer::DynamicSceneGraphLayer(LayerId layer, LayerPrefix node_prefix)
    : id(layer), prefix(node_prefix), next_node_(0) {}

bool DynamicSceneGraphLayer::mergeLayer(const DynamicSceneGraphLayer& other,
                                        const GraphMergeConfig& config,
                                        std::map<NodeId, LayerKey>* layer_lookup) {
  LayerKey layer_key{id, prefix};
  Eigen::Vector3d last_update_delta = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < other.nodes_.size(); i++) {
    const auto& other_node = *other.nodes_[i];
    if (i < next_node_) {
      // update the last_update_delta
      const Eigen::Vector3d node_position = nodes_[i]->attributes_->position;
      last_update_delta = node_position - other_node.attributes_->position;

      if (!config.update_dynamic_attributes) {
        continue;
      }

      // Update node attributes (except for position)
      nodes_[i]->attributes_ = other_node.attributes_->clone();
      nodes_[i]->attributes_->position = node_position;
    } else {
      emplaceNode(other_node.timestamp.value(), other_node.attributes_->clone(), false);
      nodes_.back()->attributes_->position += last_update_delta;
      if (layer_lookup) {
        layer_lookup->insert({nodes_.back()->id, layer_key});
      }
    }
  }

  for (const auto& id_edge_pair : other.edges()) {
    const auto& edge = id_edge_pair.second;
    if (hasEdge(edge.source, edge.target)) {
      // TODO(nathan) clone attributes
      continue;
    }

    insertEdge(edge.source, edge.target, edge.info->clone());
  }

  return true;
}

bool DynamicSceneGraphLayer::emplaceNode(std::chrono::nanoseconds stamp,
                                         NodeAttributes::Ptr&& attrs,
                                         bool add_edge) {
  if (times_.count(stamp.count())) {
    return false;
  }

  const NodeId new_id = prefix.makeId(next_node_);
  times_.insert(stamp.count());
  nodes_.emplace_back(std::make_unique<Node>(new_id, id, stamp, std::move(attrs)));
  node_status_[nodes_.size() - 1] = NodeStatus::NEW;

  // TODO(nathan) track newest time and don't add edge if node is older than that.
  // TODO(nathan) handle incorrect time ordering better

  if (add_edge && nodes_.size() > 1u) {
    insertEdgeByIndex(nodes_.size() - 2, nodes_.size() - 1);
  }

  next_node_++;
  return true;
}

bool DynamicSceneGraphLayer::emplaceNodeAtIndex(std::chrono::nanoseconds stamp,
                                                size_t index,
                                                NodeAttributes::Ptr&& attrs) {
  if (hasNodeByIndex(index)) {
    return false;
  }

  if (index >= nodes_.size()) {
    size_t num_to_insert = index - nodes_.size();
    for (size_t i = 0; i <= num_to_insert; ++i) {
      nodes_.push_back(nullptr);
      node_status_[nodes_.size()] = NodeStatus::DELETED;
    }
  }

  const NodeId new_id = prefix.makeId(index);
  times_.insert(stamp.count());
  nodes_[index] = std::make_unique<Node>(new_id, id, stamp, std::move(attrs));
  node_status_[index] = NodeStatus::NEW;
  return true;
}

bool DynamicSceneGraphLayer::hasNode(NodeId node_id) const {
  if (!prefix.matches(node_id)) {
    return false;
  }

  return hasNodeByIndex(prefix.index(node_id));
}

NodeStatus DynamicSceneGraphLayer::checkNode(NodeId node_id) const {
  if (!prefix.matches(node_id)) {
    return NodeStatus::NONEXISTENT;
  }

  return node_status_.at(prefix.index(node_id));
}

bool DynamicSceneGraphLayer::hasNodeByIndex(size_t node_index) const {
  auto iter = node_status_.find(node_index);
  if (iter == node_status_.end()) {
    return false;
  }

  return iter->second == NodeStatus::VISIBLE || iter->second == NodeStatus::NEW;
}

bool DynamicSceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
}

bool DynamicSceneGraphLayer::hasEdgeByIndex(size_t source_idx,
                                            size_t target_idx) const {
  return hasEdge(prefix.makeId(source_idx), prefix.makeId(target_idx));
}

const Node* DynamicSceneGraphLayer::findNode(NodeId node_id) const {
  return findNodeByIndex(prefix.index(node_id));
}

const Node* DynamicSceneGraphLayer::findNodeByIndex(size_t index) const {
  return index >= nodes_.size() ? nullptr : nodes_.at(index).get();
}

const Node& DynamicSceneGraphLayer::getNodeByIndex(size_t index) const {
  return getNode(prefix.makeId(index));
}

const Edge* DynamicSceneGraphLayer::findEdge(NodeId source, NodeId target) const {
  return edges_.find(source, target);
}

const Edge* DynamicSceneGraphLayer::findEdgeByIndex(size_t source_idx,
                                                    size_t target_idx) const {
  return findEdge(prefix.makeId(source_idx), prefix.makeId(target_idx));
}

const Edge& DynamicSceneGraphLayer::getEdgeByIndex(size_t source_idx,
                                                   size_t target_idx) const {
  return getEdge(prefix.makeId(source_idx), prefix.makeId(target_idx));
}

bool DynamicSceneGraphLayer::insertEdge(NodeId source,
                                        NodeId target,
                                        EdgeAttributes::Ptr&& edge_info) {
  if (source == target) {
    SG_LOG(WARNING) << "Attempted to add a self-edge for "
                    << NodeSymbol(source).getLabel() << std::endl;
    return false;
  }

  if (hasEdge(source, target)) {
    return false;
  }

  if (!hasNode(source) || !hasNode(target)) {
    return false;
  }

  nodes_[prefix.index(source)]->siblings_.insert(target);
  nodes_[prefix.index(target)]->siblings_.insert(source);

  edges_.insert(source, target, std::move(edge_info));
  return true;
}

bool DynamicSceneGraphLayer::insertEdgeByIndex(size_t source,
                                               size_t target,
                                               EdgeAttributes::Ptr&& edge_info) {
  return insertEdge(prefix.makeId(source), prefix.makeId(target), std::move(edge_info));
}

bool DynamicSceneGraphLayer::removeEdge(NodeId source, NodeId target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  nodes_[prefix.index(source)]->siblings_.erase(target);
  nodes_[prefix.index(target)]->siblings_.erase(source);

  edges_.remove(source, target);
  return true;
}

bool DynamicSceneGraphLayer::removeEdgeByIndex(size_t source_index,
                                               size_t target_index) {
  return removeEdge(prefix.makeId(source_index), prefix.makeId(target_index));
}

bool DynamicSceneGraphLayer::removeNode(NodeId node) {
  if (!hasNode(node)) {
    return false;
  }

  const size_t index = prefix.index(node);
  const auto targets_to_erase = nodes_.at(index)->siblings_;

  // reconnect "odom" edges
  const auto prev_node = node - 1;
  const auto next_node = node + 1;
  const bool has_prev_edge = hasNode(prev_node) && targets_to_erase.count(prev_node);
  const bool has_next_edge = hasNode(next_node) && targets_to_erase.count(next_node);
  if (has_prev_edge && has_next_edge) {
    // TODO(nathan) this might need to be smarter about attributes, i.e. composing the
    // two edges
    insertEdge(prev_node, next_node);
  }

  for (const auto& target : targets_to_erase) {
    removeEdge(node, target);
  }

  // TODO(nathan) this is slightly brittle, maybe consider std::map instead
  node_status_[index] = NodeStatus::DELETED;
  times_.erase(nodes_.at(index)->timestamp->count());
  nodes_[index].reset();
  return true;
}

Eigen::Vector3d DynamicSceneGraphLayer::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    std::stringstream ss;
    ss << "node " << NodeSymbol(node).getLabel() << " is missing";
    throw std::out_of_range(ss.str());
  }

  return getPositionByIndex(prefix.index(node));
}

Eigen::Vector3d DynamicSceneGraphLayer::getPositionByIndex(size_t node_index) const {
  if (!hasNodeByIndex(node_index)) {
    std::stringstream ss;
    ss << "node index" << node_index << " >= " << nodes_.size();
    throw std::out_of_range(ss.str());
  }

  return nodes_.at(node_index)->attributes().position;
}

void DynamicSceneGraphLayer::getNewNodes(std::vector<NodeId>& new_nodes,
                                         bool clear_new) {
  auto iter = node_status_.begin();
  while (iter != node_status_.end()) {
    if (iter->second == NodeStatus::NEW) {
      new_nodes.push_back(prefix.makeId(iter->first));
      if (clear_new) {
        iter->second = NodeStatus::VISIBLE;
      }
    }

    ++iter;
  }
}

void DynamicSceneGraphLayer::getRemovedNodes(std::vector<NodeId>& removed_nodes,
                                             bool clear_removed) {
  auto iter = node_status_.begin();
  while (iter != node_status_.end()) {
    if (iter->second != NodeStatus::DELETED) {
      ++iter;
      continue;
    }

    removed_nodes.push_back(prefix.makeId(iter->first));

    if (clear_removed) {
      iter = node_status_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void DynamicSceneGraphLayer::getNewEdges(std::vector<EdgeKey>& new_edges,
                                         bool clear_new) {
  return edges_.getNew(new_edges, clear_new);
}

void DynamicSceneGraphLayer::getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                                             bool clear_removed) {
  return edges_.getRemoved(removed_edges, clear_removed);
}

}  // namespace spark_dsg
