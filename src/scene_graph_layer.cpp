#include "kimera_dsg/scene_graph_layer.h"
#include "kimera_dsg/edge_attributes.h"

#include <glog/logging.h>
#include <queue>
#include <sstream>

namespace kimera {

using EdgeRef = SceneGraphLayer::EdgeRef;
using NodeRef = SceneGraphLayer::NodeRef;

SceneGraphLayer::SceneGraphLayer(LayerId layer_id) : id(layer_id) {}

bool SceneGraphLayer::emplaceNode(NodeId node_id, NodeAttributes::Ptr&& attrs) {
  nodes_status_[node_id] = NodeStatus::NEW;
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
  nodes_status_[to_insert] = NodeStatus::NEW;
  nodes_[to_insert] = std::move(node);
  return true;
}

bool SceneGraphLayer::insertEdge(NodeId source,
                                 NodeId target,
                                 EdgeAttributes::Ptr&& edge_info) {
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

  nodes_[source]->siblings_.insert(target);
  nodes_[target]->siblings_.insert(source);

  edges_.insert(source, target, std::move(edge_info));
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
  return edges_.contains(source, target);
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

  return std::cref(edges_.get(source, target));
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

bool SceneGraphLayer::mergeLayer(const SceneGraphLayer& other_layer,
                                 std::map<NodeId, LayerKey>* layer_lookup,
                                 bool update_attributes) {
  // TODO(yun)look at better interpolation methods for new nodes
  Eigen::Vector3d last_update_delta = Eigen::Vector3d::Zero();

  for (const auto& id_node_pair : other_layer.nodes_) {
    NodeStatus node_status = checkNode(id_node_pair.first);
    const auto& other = *id_node_pair.second;

    if (node_status == NodeStatus::VISIBLE || node_status == NodeStatus::NEW) {
      // update the last_update_delta
      const Eigen::Vector3d pos = nodes_[id_node_pair.first]->attributes_->position;
      last_update_delta = pos - other.attributes_->position;
      // Update node attributed (except for position)
      if (!update_attributes) {
        continue;
      }

      nodes_[id_node_pair.first]->attributes_ = other.attributes_->clone();
      nodes_[id_node_pair.first]->attributes_->position = pos;
    } else if (node_status == NodeStatus::NONEXISTENT) {
      auto attrs = other.attributes_->clone();
      attrs->position += last_update_delta;
      nodes_[other.id] = Node::Ptr(new Node(other.id, id, std::move(attrs)));
      nodes_status_[other.id] = NodeStatus::NEW;

      if (layer_lookup) {
        layer_lookup->insert({other.id, id});
      }
    }
  }

  for (const auto& id_edge_pair : other_layer.edges_.edges) {
    const auto& edge = id_edge_pair.second;
    if (hasEdge(edge.source, edge.target)) {
      // TODO(nathan) clone attributes
      continue;
    }

    insertEdge(edge.source, edge.target, edge.info->clone());
  }

  return true;
}

Eigen::Vector3d SceneGraphLayer::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    std::stringstream ss;
    ss << "node " << NodeSymbol(node).getLabel() << " not in layer";
    throw std::out_of_range(ss.str());
  }

  return nodes_.at(node)->attributes().position;
}

void SceneGraphLayer::getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) {
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

void SceneGraphLayer::getRemovedNodes(std::vector<NodeId>& removed_nodes) const {
  for (const auto& id_status_pair : nodes_status_) {
    if (id_status_pair.second == NodeStatus::DELETED) {
      removed_nodes.push_back(id_status_pair.first);
    }
  }
}

void SceneGraphLayer::getRemovedNodes(std::vector<NodeId>& removed_nodes,
                                      bool clear_removed) {
  auto iter = nodes_status_.begin();
  while (iter != nodes_status_.end()) {
    if (iter->second != NodeStatus::DELETED) {
      ++iter;
      continue;
    }

    removed_nodes.push_back(iter->first);

    if (clear_removed) {
      iter = nodes_status_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void SceneGraphLayer::getNewEdges(std::vector<EdgeKey>& new_edges, bool clear_new) {
  return edges_.getNew(new_edges, clear_new);
}

void SceneGraphLayer::getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                                      bool clear_removed) {
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

}  // namespace kimera
