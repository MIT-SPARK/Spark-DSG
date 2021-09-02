#include "kimera_dsg/scene_graph_layer.h"
#include "kimera_dsg/attribute_serialization.h"

#include <glog/logging.h>
#include <queue>

namespace kimera {

nlohmann::json SceneGraphEdgeInfo::toJson() const {
  nlohmann::json to_return{{"weighted", weighted}, {"weight", weight}};
  REGISTER_EDGE_INFO_TYPE(SceneGraphEdgeInfo, to_return);
  return to_return;
}

void SceneGraphEdgeInfo::fillFromJson(const nlohmann::json& record) {
  weighted = record.at("weighted").get<bool>();
  weight = record.at("weight").get<double>();
}

using EdgeInfo = SceneGraphLayer::EdgeInfo;
using EdgeRef = SceneGraphLayer::EdgeRef;
using NodeRef = SceneGraphLayer::NodeRef;

SceneGraphLayer::SceneGraphLayer(LayerId layer_id) : id(layer_id), last_edge_idx_(0u) {}

bool SceneGraphLayer::emplaceNode(NodeId node_id, NodeAttributes::Ptr&& attrs) {
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
  nodes_[to_insert] = std::move(node);
  return true;
}

bool SceneGraphLayer::insertEdge(NodeId source,
                                 NodeId target,
                                 EdgeInfo::Ptr&& edge_info) {
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
                            (edge_info == nullptr) ? std::make_unique<EdgeInfo>()
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

bool SceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_info_.count(source) != 0 && edges_info_.at(source).count(target) != 0;
}

// TODO(nathan) look at gtsam casting
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
  // gets rid of the leftover map if it exists
  edges_info_.erase(node_id);
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

Eigen::Vector3d SceneGraphLayer::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the layer");
  }

  return nodes_.at(node)->attributes().position;
}

void SceneGraphLayer::reset() {
  last_edge_idx_ = 0;
  nodes_.clear();
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

using nlohmann::json;

std::string SceneGraphLayer::serializeLayer(const NodeSet& nodes) const {
  json serialized_layer;
  serialized_layer["nodes"] = json::array();
  serialized_layer["edges"] = json::array();

  for (const auto& node_id : nodes) {
    if (!hasNode(node_id)) {
      continue;
    }

    const Node& node = *nodes_.at(node_id);
    json node_json = json{{"id", node.id}, {"layer", node.layer}};
    node_json["attributes"] = node.attributes().toJson();
    serialized_layer["nodes"].push_back(node_json);

    if (node.siblings().empty()) {
      continue;
    }

    for (const auto& sibling_edge_pair : edges_info_.at(node_id)) {
      const Edge& edge = edges_.at(sibling_edge_pair.second);
      json edge_json = json{{"source", edge.source}, {"target", edge.target}};
      edge_json["info"] = edge.info->toJson();
      serialized_layer["edges"].push_back(edge_json);
    }
  }

  return serialized_layer.dump();
}

std::unique_ptr<SceneGraphLayer::Edges> SceneGraphLayer::deserializeLayer(
    const std::string& info) {
  reset();
  auto node_factory = NodeAttributeFactory::Default();
  auto edge_factory = EdgeInfoFactory::Default();

  auto serialized_layer = json::parse(info);

  for (const auto& node : serialized_layer.at("nodes")) {
    auto node_id = node.at("id").get<NodeId>();
    auto layer = node.at("layer").get<LayerId>();
    if (layer != id) {
      LOG(ERROR) << "serialized node layer: " << layer
                 << " does not match layer id: " << id << ". skipping!";
      continue;
    }

    NodeAttributes::Ptr attrs = node_factory.create(node.at("attributes"));
    emplaceNode(node_id, std::move(attrs));
  }

  size_t temp_edge_idx = 0;
  std::unique_ptr<Edges> new_edges(new Edges());
  for (const auto& edge : serialized_layer.at("edges")) {
    auto source = edge.at("source").get<NodeId>();
    auto target = edge.at("target").get<NodeId>();
    EdgeInfo::Ptr info = edge_factory.create(edge.at("info"));
    new_edges->emplace(std::piecewise_construct,
                       std::forward_as_tuple(temp_edge_idx),
                       std::forward_as_tuple(source, target, std::move(info)));
    temp_edge_idx++;
  }

  return new_edges;
}

}  // namespace kimera
