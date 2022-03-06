#include "kimera_dsg/dynamic_scene_graph_layer.h"
#include "kimera_dsg/edge_attributes.h"

#include <glog/logging.h>

namespace kimera {

using EdgeRef = DynamicSceneGraphLayer::EdgeRef;
using NodeRef = DynamicSceneGraphLayer::NodeRef;

DynamicSceneGraphLayer::DynamicSceneGraphLayer(LayerId layer, LayerPrefix node_prefix)
    : id(layer), prefix(node_prefix), next_node_(0) {}

bool DynamicSceneGraphLayer::mergeLayer(const DynamicSceneGraphLayer& other,
                                        std::map<NodeId, LayerKey>* layer_lookup,
                                        bool update_attributes) {
  LayerKey layer_key{id, prefix};
  Eigen::Vector3d last_update_delta = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < other.nodes_.size(); i++) {
    const auto& other_node = *other.nodes_[i];
    if (i < next_node_) {
      // update the last_update_delta
      const Eigen::Vector3d node_position = nodes_[i]->attributes_->position;
      last_update_delta = node_position - other_node.attributes_->position;

      if (!update_attributes) {
        continue;
      }

      // Update node attributes (except for position)
      nodes_[i]->attributes_ = other_node.attributes_->clone();
      nodes_[i]->attributes_->position = node_position;
    } else {
      emplaceNode(other_node.timestamp, other_node.attributes_->clone(), false);
      nodes_.back()->attributes_->position += last_update_delta;
      if (layer_lookup) {
        layer_lookup->insert({next_node_ - 1, layer_key});
      }
    }
  }

  for (const auto& id_edge_pair : other.edges()) {
    if (id_edge_pair.first <= edges_.last_idx) {
      continue;
    }

    const auto& edge = id_edge_pair.second;
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
  nodes_.emplace_back(std::make_unique<Node>(new_id, id, std::move(attrs), stamp));
  node_status_[nodes_.size() - 1] = NodeStatus::VISIBLE;

  if (add_edge && nodes_.size() > 1u) {
    insertEdgeByIndex(nodes_.size() - 2, nodes_.size() - 1);
  }

  next_node_++;
  return true;
}

bool DynamicSceneGraphLayer::hasNode(NodeId node_id) const {
  if (!prefix.matches(node_id)) {
    return false;
  }

  return hasNodeByIndex(prefix.index(node_id));
}

bool DynamicSceneGraphLayer::hasNodeByIndex(size_t node_index) const {
  auto iter = node_status_.find(node_index);
  if (iter == node_status_.end()) {
    return false;
  }

  return iter->second == NodeStatus::VISIBLE;
}

bool DynamicSceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
}

bool DynamicSceneGraphLayer::hasEdgeByIndex(size_t source_idx,
                                            size_t target_idx) const {
  return hasEdge(prefix.makeId(source_idx), prefix.makeId(target_idx));
}

std::optional<NodeRef> DynamicSceneGraphLayer::getNodeByIndex(size_t node_index) const {
  if (!hasNodeByIndex(node_index)) {
    return std::nullopt;
  }

  return std::cref(*nodes_.at(node_index));
}

std::optional<NodeRef> DynamicSceneGraphLayer::getNode(NodeId node_id) const {
  if (!hasNode(node_id)) {
    return std::nullopt;
  }

  return getNodeByIndex(prefix.index(node_id));
}

std::optional<EdgeRef> DynamicSceneGraphLayer::getEdge(NodeId source,
                                                       NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  return std::cref(edges_.get(source, target));
}

std::optional<EdgeRef> DynamicSceneGraphLayer::getEdgeByIndex(size_t source_idx,
                                                              size_t target_idx) const {
  return getEdge(prefix.makeId(source_idx), prefix.makeId(target_idx));
}

bool DynamicSceneGraphLayer::insertEdge(NodeId source,
                                        NodeId target,
                                        EdgeAttributes::Ptr&& edge_info) {
  if (source == target) {
    LOG(WARNING) << "Attempted to add a self-edge for "
                 << NodeSymbol(source).getLabel();
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
  std::set<NodeId> targets_to_erase = nodes_.at(node)->siblings_;

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
  times_.erase(nodes_.at(index)->timestamp.count());
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

}  // namespace kimera
