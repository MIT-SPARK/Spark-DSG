#include "kimera_dsg/scene_graph.h"
#include "kimera_dsg/attribute_serialization.h"
#include "kimera_dsg/serialization_helpers.h"

#include <glog/logging.h>

#include <fstream>

namespace kimera {

using LayerRef = SceneGraph::LayerRef;
using EdgeRef = SceneGraph::EdgeRef;
using NodeRef = SceneGraph::NodeRef;
using Node = SceneGraph::Node;
using EdgeInfo = SceneGraph::EdgeInfo;

SceneGraph::SceneGraph() : SceneGraph(getDefaultLayerIds()) {}

SceneGraph::SceneGraph(const std::vector<LayerId>& layer_ids)
    : last_edge_idx_(0), layer_ids_(layer_ids) {
  if (layer_ids.empty()) {
    // TODO(nathan) custom exception
    throw std::runtime_error("Scene graph cannot be initialized with no layers");
  }

  initialize_();
}

void SceneGraph::initialize_() {
  for (const auto& id : layer_ids_) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }

  // TODO(nathan) reconsider if we actually need this
  for (const auto& id_layer_pair : layers_) {
    CHECK(id_layer_pair.second) << "Layer " << id_layer_pair.first << " was null!";
  }
}

void SceneGraph::clear() {
  layers_.clear();
  inter_layer_edges_.clear();
  initialize_();
}

bool SceneGraph::emplaceNode(LayerId layer_id,
                             NodeId node_id,
                             NodeAttributes::Ptr&& attrs) {
  if (!hasLayer(layer_id)) {
    LOG(WARNING) << "Invalid layer: " << layer_id;
    return false;
  }

  if (node_layer_lookup_.count(node_id) != 0) {
    return false;
  }

  bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_layer_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool SceneGraph::insertNode(Node::Ptr&& node) {
  if (node == nullptr) {
    LOG(ERROR) << "Attempting to add an uninitialized node";
    return false;
  }

  // we grab these here to avoid problems with move
  const LayerId node_layer = node->layer;
  const NodeId node_id = node->id;

  if (!hasLayer(node_layer)) {
    return false;
  }

  if (node_layer_lookup_.count(node_id) != 0) {
    return false;
  }

  bool successful = layers_[node_layer]->insertNode(std::move(node));
  if (successful) {
    node_layer_lookup_[node_id] = node_layer;
  }

  return successful;
}

// TODO(nathan) this duplicates work with some other stuff. maybe rethink
Node* SceneGraph::getParentNode_(NodeId source, NodeId target) const {
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  return source_layer > target_layer
             ? layers_.at(source_layer)->nodes_.at(source).get()
             : layers_.at(target_layer)->nodes_.at(target).get();
}

// TODO(nathan) this duplicates work with some other stuff. maybe rethink
Node* SceneGraph::getChildNode_(NodeId source, NodeId target) const {
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  return source_layer < target_layer
             ? layers_.at(source_layer)->nodes_.at(source).get()
             : layers_.at(target_layer)->nodes_.at(target).get();
}

bool SceneGraph::insertEdge(NodeId source, NodeId target, EdgeInfo::Ptr&& edge_info) {
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

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_[source_layer]->insertEdge(source, target, std::move(edge_info));
  }

  // TODO(nathan) consider warning for non-adjacent edges
  // non-adjacent edges have a good use case: coarse detections at building
  // level for lower layers

  Node* parent = getParentNode_(source, target);
  Node* child = getChildNode_(source, target);
  // TODO(nathan) asserts

  if (child->hasParent()) {
    return false;
  }

  child->setParent_(parent->id);
  parent->children_.insert(child->id);

  last_edge_idx_++;
  inter_layer_edges_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(last_edge_idx_),
      std::forward_as_tuple(source,
                            target,
                            (edge_info == nullptr) ? std::make_unique<EdgeInfo>()
                                                   : std::move(edge_info)));
  inter_layer_edges_info_[source][target] = last_edge_idx_;
  inter_layer_edges_info_[target][source] = last_edge_idx_;
  return true;
}

bool SceneGraph::hasLayer(LayerId layer_id) const {
  return layers_.count(layer_id) != 0;
}

bool SceneGraph::hasNode(NodeId node_id) const {
  return node_layer_lookup_.count(node_id) != 0;
}

bool SceneGraph::hasEdge(NodeId source, NodeId target) const {
  if (!(hasNode(source) && hasNode(target))) {
    return false;
  }

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->hasEdge(source, target);
  }

  return inter_layer_edges_info_.count(source) != 0 &&
         inter_layer_edges_info_.at(source).count(target) != 0;
}

std::optional<LayerRef> SceneGraph::getLayer(LayerId layer_id) const {
  if (!hasLayer(layer_id)) {
    return std::nullopt;
  }

  // TODO(nathan) consider assert here instead
  CHECK(layers_.at(layer_id)) << "Invalid layer";
  return std::cref(*layers_.at(layer_id));
}

// TODO(nathan) look at gtsam casting
std::optional<NodeRef> SceneGraph::getNode(NodeId node_id) const {
  if (!hasNode(node_id)) {
    return std::nullopt;
  }

  // TODO(nathan) consider assert
  LayerId layer = node_layer_lookup_.at(node_id);
  return layers_.at(layer)->getNode(node_id);
}

std::optional<EdgeRef> SceneGraph::getEdge(NodeId source, NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  // defer to layers if it is a intra-layer edge
  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->getEdge(source, target);
  }

  size_t edge_idx = inter_layer_edges_info_.at(source).at(target);
  return std::cref(inter_layer_edges_.at(edge_idx));
}

bool SceneGraph::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  LayerId layer = node_layer_lookup_.at(node_id);

  Node* node = layers_[layer]->nodes_.at(node_id).get();
  // TODO(nathan) consider asserts
  if (node->hasParent()) {
    removeInterLayerEdge(node_id, node->parent_);
  }

  std::set<NodeId> targets_to_erase = node->children_;
  for (const auto& target : targets_to_erase) {
    removeInterLayerEdge(node_id, target);
  }

  layers_[layer]->removeNode(node_id);
  node_layer_lookup_.erase(node_id);
  return true;
}

bool SceneGraph::mergeNodes(NodeId node_from, NodeId node_to) {
  if (!hasNode(node_from) || !hasNode(node_to)) {
    return false;
  }

  if (node_from == node_to) {
    return false;
  }

  LayerId layer = node_layer_lookup_.at(node_from);

  if (layer != node_layer_lookup_.at(node_to)) {
    return false;  // Cannot merge nodes of different layers
  }

  Node* node = layers_[layer]->nodes_.at(node_from).get();
  // Remove parent
  // TODO(nathan) consider asserts
  if (node->hasParent()) {
    rewireInterLayerEdge_(node_from, node->parent_, node_to, node->parent_);
  }

  // Reconnect children
  std::set<NodeId> targets_to_rewire = node->children_;
  for (const auto& target : targets_to_rewire) {
    rewireInterLayerEdge_(node_from, target, node_to, target);
  }

  layers_[layer]->mergeNodes(node_from, node_to);
  node_layer_lookup_.erase(node_from);
  return true;
}

void SceneGraph::removeInterLayerEdge(NodeId source, NodeId target) {
  inter_layer_edges_.erase(inter_layer_edges_info_.at(source).at(target));
  inter_layer_edges_info_.at(source).erase(target);
  if (inter_layer_edges_info_.at(source).empty()) {
    inter_layer_edges_info_.erase(source);
  }

  inter_layer_edges_info_.at(target).erase(source);
  if (inter_layer_edges_info_.at(target).empty()) {
    inter_layer_edges_info_.erase(target);
  }

  Node* parent = getParentNode_(source, target);
  Node* child = getChildNode_(source, target);
  parent->children_.erase(child->id);
  child->clearParent_();
}

void SceneGraph::rewireInterLayerEdge_(NodeId source,
                                       NodeId target,
                                       NodeId new_source,
                                       NodeId new_target) {
  if (source == new_source && target == new_target) {
    return;
  }

  if (hasEdge(new_source, new_target)) {
    removeInterLayerEdge(source, target);
    return;
  }

    // Clean up old parent and child
  Node* parent = getParentNode_(source, target);
  Node* child = getChildNode_(source, target);
  parent->children_.erase(child->id);
  child->clearParent_();

  // Connect new parent and child
  Node* new_parent = getParentNode_(new_source, new_target);
  Node* new_child = getChildNode_(new_source, new_target);
  if (!new_child->hasParent()) {
    new_child->setParent_(new_parent->id);
    new_parent->children_.insert(new_child->id);
  } else {
    // cannot have two parents
    removeInterLayerEdge(source, target);
    return;
  }

  const size_t edge_idx = inter_layer_edges_info_[source][target];
  const Edge& orig_edge = inter_layer_edges_.at(edge_idx);
  // Remove old
  inter_layer_edges_.erase(edge_idx);
  // Add new rewired edge
  inter_layer_edges_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(edge_idx),
      std::forward_as_tuple(
          new_source, new_target, std::make_unique<EdgeInfo>(*orig_edge.info)));

  inter_layer_edges_info_[new_source][new_target] = edge_idx;
  inter_layer_edges_info_[new_target][new_source] = edge_idx;

  // Remove old edge from info
  inter_layer_edges_info_.at(source).erase(target);
  if (inter_layer_edges_info_.at(source).empty()) {
    inter_layer_edges_info_.erase(source);
  }

  inter_layer_edges_info_.at(target).erase(source);
  if (inter_layer_edges_info_.at(target).empty()) {
    inter_layer_edges_info_.erase(target);
  }

}

bool SceneGraph::removeEdge(NodeId source, NodeId target) {
  if (!hasEdge(source, target)) {
    return false;
  }

  LayerId source_layer = node_layer_lookup_.at(source);
  LayerId target_layer = node_layer_lookup_.at(target);
  if (source_layer == target_layer) {
    return layers_.at(source_layer)->removeEdge(source, target);
  }

  removeInterLayerEdge(source, target);
  return true;
}

size_t SceneGraph::numLayers() const { return layers_.size(); }

size_t SceneGraph::numNodes() const {
  size_t total_nodes = 0u;
  for (const auto& id_layer_pair : layers_) {
    total_nodes += id_layer_pair.second->numNodes();
  }

  return total_nodes;
}

size_t SceneGraph::numEdges() const {
  size_t total_edges = inter_layer_edges_.size();
  for (const auto& id_layer_pair : layers_) {
    total_edges += id_layer_pair.second->numEdges();
  }

  return total_edges;
}

Eigen::Vector3d SceneGraph::getPosition(NodeId node) const {
  if (!hasNode(node)) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the graph");
  }

  LayerId layer = node_layer_lookup_.at(node);
  return layers_.at(layer)->getPosition(node);
}

bool SceneGraph::updateFromLayer(SceneGraphLayer& other_layer,
                                 std::unique_ptr<Edges>&& edges) {
  if (!layers_.count(other_layer.id)) {
    LOG(ERROR) << "Scene graph does not have layer: " << other_layer.id;
    return false;
  }

  Layer& internal_layer = *layers_.at(other_layer.id);
  for (auto& id_node_pair : other_layer.nodes_) {
    if (internal_layer.hasNode(id_node_pair.first)) {
      // just copy the attributes (prior edge information should be preserved)
      internal_layer.nodes_[id_node_pair.first]->attributes_ =
          std::move(id_node_pair.second->attributes_);
    } else {
      // we need to let the scene graph know about new nodes
      node_layer_lookup_[id_node_pair.first] = internal_layer.id;
      internal_layer.nodes_[id_node_pair.first] = std::move(id_node_pair.second);
      internal_layer.nodes_status_[id_node_pair.first] = NodeStatus::VISIBLE;
    }
  }

  // we just invalidated all the nodes in the other layer, so we better reset everything
  other_layer.reset();

  if (!edges) {
    return true;
  }

  for (auto& id_edge_pair : *edges) {
    Edge& edge = id_edge_pair.second;
    if (internal_layer.hasEdge(edge.source, edge.target)) {
      // just copy over info
      auto edge_id = internal_layer.edges_info_.at(edge.source).at(edge.target);
      internal_layer.edges_.at(edge_id).info = std::move(edge.info);
      continue;
    }

    internal_layer.insertEdge(edge.source, edge.target, std::move(edge.info));
  }

  // we just invalidated all the info for the new edges, so reset the edges
  edges.reset();
  return true;
}

bool SceneGraph::mergeGraph(const SceneGraph& other) {
  std::vector<NodeId> removed_nodes;
  for (const auto& id_layer : other.layers()) {
    if (hasLayer(id_layer.first)) {
      layers_[id_layer.first]->mergeLayer(*id_layer.second,
                                          &node_layer_lookup_);
    }
    id_layer.second->getRemovedNodes(&removed_nodes);
  }

  for (const auto& id_edge : other.inter_layer_edges()) {
    insertEdge(id_edge.second.source,
               id_edge.second.target,
               std::make_unique<SceneGraphEdgeInfo>(*id_edge.second.info));
  }

  for (const auto& removed_id : removed_nodes) {
    removeNode(removed_id);
  }

  return true;
}

json SceneGraph::toJson(const JsonExportConfig& config) const {
  json to_return;
  to_return["directed"] = false;
  to_return["multigraph"] = false;
  to_return["nodes"] = json::array();
  to_return[config.edge_key] = json::array();
  to_return["layer_ids"] = layer_ids_;

  for (const auto& id_layer_pair : layers_) {
    for (const auto& id_node_pair : id_layer_pair.second->nodes_) {
      const Node& node = *(id_node_pair.second);
      json node_json = json{{config.name_key, node.id}, {"layer", node.layer}};
      node_json["attributes"] = node.attributes().toJson();
      to_return["nodes"].push_back(node_json);
    }

    for (const auto& id_edge_pair : id_layer_pair.second->edges_) {
      const Edge& edge = id_edge_pair.second;
      json edge_json =
          json{{config.source_key, edge.source}, {config.target_key, edge.target}};
      edge_json["info"] = edge.info->toJson();
      to_return[config.edge_key].push_back(edge_json);
    }
  }

  for (const auto& id_edge_pair : inter_layer_edges_) {
    const Edge& edge = id_edge_pair.second;
    json edge_json =
        json{{config.source_key, edge.source}, {config.target_key, edge.target}};
    edge_json["info"] = edge.info->toJson();
    to_return[config.edge_key].push_back(edge_json);
  }

  return to_return;
}

void SceneGraph::fillFromJson(const JsonExportConfig& config,
                              const NodeAttributeFactory& node_attr_factory,
                              const EdgeInfoFactory& edge_info_factory,
                              const json& record) {
  // we have to clear after setting the layer ids to get the right layers initialized
  layer_ids_ = record.at("layer_ids").get<LayerIds>();
  SceneGraph::clear();  // dynamic layers are filled before we do this

  for (const auto& node : record.at("nodes")) {
    if (node.contains("timestamp")) {
      continue;  // we found a dynamic node
    }
    auto id = node.at(config.name_key).get<NodeId>();
    auto layer = node.at("layer").get<LayerId>();
    NodeAttributes::Ptr attrs = node_attr_factory.create(node.at("attributes"));
    emplaceNode(layer, id, std::move(attrs));
  }

  for (const auto& edge : record.at(config.edge_key)) {
    auto source = edge.at(config.source_key).get<NodeId>();
    auto target = edge.at(config.target_key).get<NodeId>();
    EdgeInfo::Ptr info = edge_info_factory.create(edge.at("info"));
    insertEdge(source, target, std::move(info));
  }
}

namespace {

inline bool extensionIsBson(const std::string& filepath) {
  if (filepath.size() < 4) {
    return false;
  }
  return filepath.substr(filepath.size() - 4) == "bson";
}

}  // namespace

void SceneGraph::save(const std::string& filepath, bool force_bson) const {
  nlohmann::json graph_json = toJson(JsonExportConfig());
  if (force_bson || extensionIsBson(filepath)) {
    graph_json["compact"] = true;
    graph_json["schema"] = 0;
    std::vector<uint8_t> data = nlohmann::json::to_bson(graph_json);
    std::ofstream outfile(filepath, std::ios::out | std::ios::binary);
    // uint8_t -> char should be a safe cast for this case
    outfile.write(reinterpret_cast<char*>(data.data()), data.size());
  } else {
    std::ofstream outfile(filepath);
    outfile << graph_json;
  }
}

void SceneGraph::load(const std::string& filepath, bool force_bson) {
  nlohmann::json graph_json;
  if (force_bson || extensionIsBson(filepath)) {
    std::ifstream infile(filepath, std::ios::in | std::ios::binary);
    infile.seekg(0, std::ios::end);
    std::streampos file_size = infile.tellg();
    infile.seekg(0, std::ios::beg);

    std::vector<uint8_t> data(file_size);
    // uint8_t -> char should be a safe cast for this case
    infile.read(reinterpret_cast<char*>(data.data()), data.size());
    graph_json = json::from_bson(data);
  } else {
    std::ifstream infile(filepath);
    infile >> graph_json;
  }

  fillFromJson(JsonExportConfig(),
               NodeAttributeFactory::Default(),
               EdgeInfoFactory::Default(),
               graph_json);
}

SceneGraph::LayerIds getDefaultLayerIds() {
  SceneGraph::LayerIds default_ids{KimeraDsgLayers::OBJECTS,
                                   KimeraDsgLayers::PLACES,
                                   KimeraDsgLayers::ROOMS,
                                   KimeraDsgLayers::BUILDINGS};
  return default_ids;
}

}  // namespace kimera
