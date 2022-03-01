#include "kimera_dsg/scene_graph_layer.h"
#include "kimera_dsg/edge_attributes.h"

#include <glog/logging.h>
#include <queue>

namespace kimera {

SceneGraphEdge::SceneGraphEdge(NodeId source, NodeId target, AttrPtr&& info)
      : source(source), target(target), info(std::move(info)) {}

SceneGraphEdge::~SceneGraphEdge() = default;

using EdgeAttributesPtr = SceneGraphLayer::EdgeAttributesPtr;
using EdgeRef = SceneGraphLayer::EdgeRef;
using NodeRef = SceneGraphLayer::NodeRef;

SceneGraphLayer::SceneGraphLayer(LayerId layer_id) : id(layer_id), last_edge_idx_(0u) {}

bool SceneGraphLayer::emplaceNode(NodeId node_id, NodeAttributes::Ptr&& attrs) {
  nodes_status_[node_id] = NodeStatus::VISIBLE;
  return nodes_.emplace(node_id, std::make_unique<Node>(node_id, id, std::move(attrs)))
      .second;
}

bool SceneGraphLayer::insertNode(SceneGraphNode::Ptr&& node) {
  if (!node) {
    LOG(ERROR) << "Attempted to add an unitialized node to layer " << id;
    return false;
  }

  if (node->layer != id) {
    LOG(WARNING) << "Attempted to add a node with layer " << node->layer << " to layer "
                 << id;
    return false;
  }

  if (hasNode(node->id)) {
    return false;
  }

  NodeId to_insert = node->id;
  nodes_status_[to_insert] = NodeStatus::VISIBLE;
  nodes_[to_insert] = std::move(node);
  return true;
}

bool SceneGraphLayer::insertEdge(NodeId source,
                                 NodeId target,
                                 EdgeAttributesPtr&& edge_info) {
  if (source == target) {
    LOG(WARNING) << "Attempted to add a self-edge";
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
                            (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                                   : std::move(edge_info)));
  edges_info_[source][target] = last_edge_idx_;
  edges_info_[target][source] = last_edge_idx_;
  nodes_[source]->siblings_.insert(target);
  nodes_[target]->siblings_.insert(source);
  return true;
}

bool SceneGraphLayer::hasNode(NodeId node_id) const {
  return nodes_.count(node_id) != 0;
}

NodeStatus SceneGraphLayer::checkNode(NodeId node_id) const {
  if (nodes_status_.count(node_id) == 0) {
    return NodeStatus::NONEXISTENT;
  }
  return nodes_status_.at(node_id);
}

bool SceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_info_.count(source) != 0 && edges_info_.at(source).count(target) != 0;
}

std::optional<NodeRef> SceneGraphLayer::getNode(NodeId node_id) const {
  if (!hasNode(node_id)) {
    return std::nullopt;
  }

  // TODO(nathan) consider assert instead
  CHECK(nodes_.at(node_id) != nullptr) << "Unitialized node found!";
  return std::cref(*(nodes_.at(node_id)));
}

std::optional<EdgeRef> SceneGraphLayer::getEdge(NodeId source, NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  size_t edge_idx = edges_info_.at(source).at(target);
  return std::cref(edges_.at(edge_idx));
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
  // gets rid of the leftover map if it exists
  edges_info_.erase(node_id);
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
  // gets rid of the leftover map if it exists
  edges_info_.erase(node_from);
  return true;
}

bool SceneGraphLayer::removeEdge(NodeId source, NodeId target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  edges_.erase(edges_info_.at(source).at(target));

  edges_info_.at(source).erase(target);
  if (edges_info_.at(source).empty()) {
    edges_info_.erase(source);
  }

  edges_info_.at(target).erase(source);
  if (edges_info_.at(target).empty()) {
    edges_info_.erase(target);
  }

  nodes_[source]->siblings_.erase(target);
  nodes_[target]->siblings_.erase(source);
  return true;
}

bool SceneGraphLayer::rewireEdge(NodeId source,
                                 NodeId target,
                                 NodeId new_source,
                                 NodeId new_target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  if (!hasNode(new_source)) {
    return false;
  }

  if (!hasNode(new_target)) {
    return false;
  }

  if (source == new_source && target == new_target) {
    return false;
  }

  if (new_source == new_target) {
    // Rewire edge to the same node
    removeEdge(source, target);
    return true;
  }

  if (hasEdge(new_source, new_target)) {
    // Edge already exist
    removeEdge(source, target);
    return true;
  }

  // Get edge index
  const size_t edge_idx = edges_info_[source][target];
  const Edge& orig_edge = edges_.at(edge_idx);
  // Remove old edge
  edges_.erase(edge_idx);
  // Add new rewired edge
  edges_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(edge_idx),
      std::forward_as_tuple(
          new_source, new_target, std::make_unique<EdgeAttributes>(*orig_edge.info)));

  // Insert edge index for new rewired edge
  edges_info_[new_source][new_target] = edge_idx;
  edges_info_[new_target][new_source] = edge_idx;

  // Remove old edge from edge_info_
  edges_info_.at(source).erase(target);
  if (edges_info_.at(source).empty()) {
    edges_info_.erase(source);
  }

  edges_info_.at(target).erase(source);
  if (edges_info_.at(target).empty()) {
    edges_info_.erase(target);
  }

  // Remove old
  nodes_[source]->siblings_.erase(target);
  nodes_[target]->siblings_.erase(source);

  // Connect new
  nodes_[new_source]->siblings_.insert(new_target);
  nodes_[new_target]->siblings_.insert(new_source);

  return true;
}

bool SceneGraphLayer::mergeLayer(const SceneGraphLayer& other,
                                 std::map<NodeId, LayerId>* layer_lookup,
                                 bool update_attributes) {
  Eigen::Vector3d last_update_delta = Eigen::Vector3d::Zero();

  for (const auto& id_node_pair : other.nodes_) {
    // TODO(yun)look at better interpolation methods for new nodes
    NodeStatus node_status = checkNode(id_node_pair.first);
    if (node_status == NodeStatus::VISIBLE) {
      // update the last_update_delta
      last_update_delta = nodes_[id_node_pair.first]->attributes_->position -
                          id_node_pair.second->attributes_->position;
      // Update node attributed (except for position)
      Eigen::Vector3d node_position = nodes_[id_node_pair.first]->attributes_->position;
      if (update_attributes) {
        nodes_[id_node_pair.first]->attributes_ =
            id_node_pair.second->attributes_->clone();
      }
      nodes_[id_node_pair.first]->attributes_->position = node_position;
    } else if (node_status == NodeStatus::NONEXISTENT) {
      nodes_[id_node_pair.first] = Node::Ptr(
          new Node(id_node_pair.first, id, id_node_pair.second->attributes_->clone()));
      nodes_[id_node_pair.first]->attributes_->position += last_update_delta;
      nodes_status_[id_node_pair.first] = NodeStatus::VISIBLE;
      if (nullptr != layer_lookup) {
        layer_lookup->insert({id_node_pair.first, id});
      }
    }
  }

  for (const auto& id_edge_pair : other.edges_) {
    if (id_edge_pair.first > last_edge_idx_) {
      const Edge& edge = id_edge_pair.second;
      insertEdge(
          edge.source, edge.target, std::make_unique<EdgeAttributes>(*edge.info));
    }
  }
  return true;
}

Eigen::Vector3d SceneGraphLayer::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the layer");
  }

  return nodes_.at(node)->attributes().position;
}

void SceneGraphLayer::getRemovedNodes(std::vector<NodeId>* removed_nodes) const {
  CHECK(nullptr != removed_nodes);
  for (const auto& id_status : nodes_status_) {
    if (id_status.second == NodeStatus::DELETED) {
      removed_nodes->push_back(id_status.first);
    }
  }
  return;
}

void SceneGraphLayer::reset() {
  last_edge_idx_ = 0;
  nodes_.clear();
  nodes_status_.clear();
  edges_.clear();
  edges_info_.clear();
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

}  // namespace kimera
