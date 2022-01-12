#include "kimera_dsg/dynamic_scene_graph_layer.h"

namespace kimera {

using EdgeInfo = DynamicSceneGraphLayer::EdgeInfo;
using EdgeRef = DynamicSceneGraphLayer::EdgeRef;
using NodeRef = DynamicSceneGraphLayer::NodeRef;

DynamicSceneGraphLayer::DynamicSceneGraphLayer(LayerId layer, char node_prefix)
    : id(layer), prefix(node_prefix), next_node_(node_prefix, 0), last_edge_idx_(0) {}

bool DynamicSceneGraphLayer::mergeLayer(
    const DynamicSceneGraphLayer& other,
    std::map<NodeId, DynamicLayerKey>* layer_lookup,
    bool update_attributes) {
  DynamicLayerKey layer_key;
  layer_key.type = id;
  layer_key.prefix = prefix;
  Eigen::Vector3d last_update_delta = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < other.nodes_.size(); i++) {
    if (i < NodeSymbol(next_node_).categoryId()) {
      // update the last_update_delta
      last_update_delta =
          nodes_[i]->attributes_->position - other.nodes_[i]->attributes_->position;
      // Update node attributes (except for position)
      Eigen::Vector3d node_position = nodes_[i]->attributes_->position;
      if (update_attributes) {
        nodes_[i]->attributes_ = other.nodes_[i]->attributes_->clone();
      }
      nodes_[i]->attributes_->position = node_position;
    } else {
      emplaceNode(
          other.nodes_[i]->timestamp, other.nodes_[i]->attributes_->clone(), false);
      nodes_.back()->attributes_->position += last_update_delta;
      if (layer_lookup) {
        layer_lookup->insert({next_node_ - 1, layer_key});
      }
    }
  }

  for (const auto& id_edge_pair : other.edges_) {
    if (id_edge_pair.first > last_edge_idx_) {
      const Edge& edge = id_edge_pair.second;
      insertEdge(
          edge.source, edge.target, std::make_unique<SceneGraphEdgeInfo>(*edge.info));
    }
  }
  return true;
}

bool DynamicSceneGraphLayer::emplaceNode(std::chrono::nanoseconds timestamp,
                                         NodeAttributes::Ptr&& attrs,
                                         bool add_edge) {
  if (times_.count(timestamp.count())) {
    return false;
  }

  times_.insert(timestamp.count());
  nodes_.emplace_back(
      std::make_unique<Node>(next_node_, id, std::move(attrs), timestamp));

  if (add_edge && nodes_.size() > 1u) {
    insertEdgeByIndex(nodes_.size() - 2, nodes_.size() - 1);
  }

  next_node_++;
  return true;
}

bool DynamicSceneGraphLayer::hasNode(NodeId node_id) const {
  NodeSymbol node_symbol(node_id);
  if (node_symbol.category() != next_node_.category()) {
    return false;
  }

  return hasNodeByIndex(node_symbol.categoryId());
}

bool DynamicSceneGraphLayer::hasNodeByIndex(size_t node_index) const {
  return node_index < nodes_.size();
}

bool DynamicSceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_info_.count(source) != 0 && edges_info_.at(source).count(target) != 0;
}

bool DynamicSceneGraphLayer::hasEdgeByIndex(size_t source_idx,
                                            size_t target_idx) const {
  const NodeSymbol source(next_node_.category(), source_idx);
  const NodeSymbol target(next_node_.category(), target_idx);
  return hasEdge(source, target);
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

  // TODO(nathan) this is slightly inefficient, but less code
  return getNodeByIndex(NodeSymbol(node_id).categoryId());
}

std::optional<EdgeRef> DynamicSceneGraphLayer::getEdge(NodeId source,
                                                       NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  size_t edge_idx = edges_info_.at(source).at(target);
  return std::cref(edges_.at(edge_idx));
}

std::optional<EdgeRef> DynamicSceneGraphLayer::getEdgeByIndex(size_t source_idx,
                                                              size_t target_idx) const {
  const NodeSymbol source(next_node_.category(), source_idx);
  const NodeSymbol target(next_node_.category(), target_idx);
  return getEdge(source, target);
}

bool DynamicSceneGraphLayer::insertEdge(NodeId source,
                                        NodeId target,
                                        EdgeInfo::Ptr&& edge_info) {
  if (source == target) {
    LOG(WARNING) << "Attempted to add a self-edge for "
                 << NodeSymbol(source).getLabel();
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

  last_edge_idx_++;
  edges_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(last_edge_idx_),
      std::forward_as_tuple(source,
                            target,
                            (edge_info == nullptr) ? std::make_unique<EdgeInfo>()
                                                   : std::move(edge_info)));
  edges_info_[source][target] = last_edge_idx_;
  edges_info_[target][source] = last_edge_idx_;
  nodes_[NodeSymbol(source).categoryId()]->siblings_.insert(target);
  nodes_[NodeSymbol(source).categoryId()]->siblings_.insert(source);
  return true;
}

bool DynamicSceneGraphLayer::insertEdgeByIndex(size_t source_idx,
                                               size_t target_idx,
                                               EdgeInfo::Ptr&& edge_info) {
  const NodeSymbol source(next_node_.category(), source_idx);
  const NodeSymbol target(next_node_.category(), target_idx);
  return insertEdge(source, target, std::move(edge_info));
}

bool DynamicSceneGraphLayer::removeEdge(NodeId source, NodeId target) {
  if (NodeSymbol(source).category() != prefix ||
      NodeSymbol(target).category() != prefix) {
    return false;
  }

  return removeEdgeByIndex(NodeSymbol(source).categoryId(),
                           NodeSymbol(target).categoryId());
}

bool DynamicSceneGraphLayer::removeEdgeByIndex(size_t source_index,
                                               size_t target_index) {
  if (!hasEdgeByIndex(source_index, target_index)) {
    return false;
  }

  NodeSymbol source(prefix, source_index);
  NodeSymbol target(prefix, target_index);

  edges_.erase(edges_info_.at(source).at(target));

  edges_info_.at(source).erase(target);
  if (edges_info_.at(source).empty()) {
    edges_info_.erase(source);
  }

  edges_info_.at(target).erase(source);
  if (edges_info_.at(target).empty()) {
    edges_info_.erase(target);
  }

  nodes_[source_index]->siblings_.erase(target);
  nodes_[target_index]->siblings_.erase(source);
  return true;
}

Eigen::Vector3d DynamicSceneGraphLayer::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the layer");
  }

  return getPositionByIndex(NodeSymbol(node).categoryId());
}

Eigen::Vector3d DynamicSceneGraphLayer::getPositionByIndex(size_t node_index) const {
  if (!hasNodeByIndex(node_index)) {
    throw std::out_of_range("node index " + std::to_string(node_index) +
                            " >= " + std::to_string(nodes_.size()));
  }

  return nodes_.at(node_index)->attributes().position;
}

}  // namespace kimera
