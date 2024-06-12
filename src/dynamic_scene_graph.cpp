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
#include "spark_dsg/dynamic_scene_graph.h"

#include <filesystem>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/serialization/file_io.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;

DynamicSceneGraph::LayerIds getDefaultLayerIds() {
  return {
      DsgLayers::OBJECTS, DsgLayers::PLACES, DsgLayers::ROOMS, DsgLayers::BUILDINGS};
}

DynamicSceneGraph::DynamicSceneGraph() : DynamicSceneGraph(getDefaultLayerIds()) {}

DynamicSceneGraph::DynamicSceneGraph(const LayerIds& layer_ids) : layer_ids(layer_ids) {
  if (layer_ids.empty()) {
    throw std::domain_error("scene graph cannot be initialized without layers");
  }

  clear();
}

void DynamicSceneGraph::clear() {
  layers_.clear();
  dynamic_layers_.clear();

  node_lookup_.clear();

  interlayer_edges_.reset();
  dynamic_interlayer_edges_.reset();

  mesh_.reset();

  for (const auto& id : layer_ids) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }
}

void DynamicSceneGraph::reset(const LayerIds& new_layer_ids) {
  const_cast<LayerIds&>(layer_ids) = new_layer_ids;
  clear();
}

// TODO(nathan) consider refactoring to use operator[]
bool DynamicSceneGraph::createDynamicLayer(LayerId layer, LayerPrefix layer_prefix) {
  if (hasLayer(layer, layer_prefix)) {
    return false;
  }

  if (!dynamic_layers_.count(layer)) {
    dynamic_layers_[layer] = DynamicLayers();
  }

  dynamic_layers_[layer].emplace(
      layer_prefix, std::make_unique<DynamicSceneGraphLayer>(layer, layer_prefix));
  return true;
}

bool DynamicSceneGraph::emplaceNode(LayerId layer_id,
                                    NodeId node_id,
                                    NodeAttributes::Ptr&& attrs) {
  if (node_lookup_.count(node_id)) {
    return false;
  }

  if (!layers_.count(layer_id)) {
    SG_LOG(WARNING) << "Invalid layer: " << layer_id << std::endl;
    return false;
  }

  const bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool DynamicSceneGraph::emplaceNode(LayerId layer,
                                    LayerPrefix prefix,
                                    std::chrono::nanoseconds time,
                                    NodeAttributes::Ptr&& attrs,
                                    bool add_edge) {
  bool has_layer = false;
  NodeSymbol new_node_id = prefix.makeId(0);
  if (hasLayer(layer, prefix)) {
    has_layer = true;
    new_node_id = prefix.makeId(dynamic_layers_[layer][prefix]->next_node_);
  }

  if (hasNode(new_node_id)) {
    SG_LOG(ERROR) << "scene graph contains node " << new_node_id.getLabel()
                  << ". fix conflicting prefix: " << prefix.str() << std::endl;
    return false;
  }

  if (!has_layer) {
    createDynamicLayer(layer, prefix);
  }

  if (!dynamic_layers_[layer][prefix]->emplaceNode(time, std::move(attrs), add_edge)) {
    return false;
  }

  node_lookup_[new_node_id] = {layer, prefix};
  return true;
}

bool DynamicSceneGraph::emplacePrevDynamicNode(LayerId layer,
                                               NodeId prev_node_id,
                                               std::chrono::nanoseconds time,
                                               NodeAttributes::Ptr&& attrs) {
  if (hasNode(prev_node_id)) {
    SG_LOG(ERROR) << "scene graph already contains node "
                  << NodeSymbol(prev_node_id).getLabel() << std::endl;
    return false;
  }

  const auto prefix = LayerPrefix::fromId(prev_node_id);
  if (!hasLayer(layer, prefix)) {
    createDynamicLayer(layer, prefix);
  }

  const auto result = dynamic_layers_[layer][prefix]->emplaceNodeAtIndex(
      time, prefix.index(prev_node_id), std::move(attrs));
  if (!result) {
    return false;
  }

  node_lookup_[prev_node_id] = {layer, prefix};
  return true;
}

bool DynamicSceneGraph::insertNode(Node::Ptr&& node) {
  if (!node) {
    return false;
  }

  if (node_lookup_.count(node->id)) {
    return false;
  }

  // we grab these here to avoid problems with move
  const LayerId node_layer = node->layer;
  const NodeId node_id = node->id;

  if (!hasLayer(node_layer)) {
    return false;
  }

  const bool successful = layers_[node_layer]->insertNode(std::move(node));
  if (successful) {
    node_lookup_[node_id] = node_layer;
  }

  return successful;
}

bool DynamicSceneGraph::addOrUpdateNode(LayerId layer_id,
                                        NodeId node_id,
                                        NodeAttributes::Ptr&& attrs,
                                        std::optional<std::chrono::nanoseconds> stamp) {
  if (!layers_.count(layer_id)) {
    SG_LOG(WARNING) << "Invalid layer: " << layer_id << std::endl;
    return false;
  }

  auto iter = node_lookup_.find(node_id);
  if (iter != node_lookup_.end()) {
    getNodePtr(node_id, iter->second)->attributes_ = std::move(attrs);
    return true;
  }

  if (stamp) {
    return emplacePrevDynamicNode(layer_id, node_id, stamp.value(), std::move(attrs));
  }

  const bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool DynamicSceneGraph::insertEdge(NodeId source,
                                   NodeId target,
                                   EdgeAttributes::Ptr&& edge_info) {
  LayerKey source_key, target_key;
  if (hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  if (!source_key || !target_key) {
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  if (source_key == target_key) {
    return layerFromKey(source_key).insertEdge(source, target, std::move(attrs));
  }

  // add information to the nodes about the node relationship
  addAncestry(source, target, source_key, target_key);

  if (source_key.dynamic || target_key.dynamic) {
    dynamic_interlayer_edges_.insert(source, target, std::move(attrs));
  } else {
    interlayer_edges_.insert(source, target, std::move(attrs));
  }

  return true;
}

bool DynamicSceneGraph::insertParentEdge(NodeId source,
                                         NodeId target,
                                         EdgeAttributes::Ptr&& edge_info) {
  LayerKey source_key, target_key;
  if (hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  if (!source_key || !target_key) {
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  // force single parent to exist
  if (source_key != target_key) {
    dropAllParents(source, target, source_key, target_key);
  }

  addAncestry(source, target, source_key, target_key);

  if (source_key.dynamic || target_key.dynamic) {
    dynamic_interlayer_edges_.insert(source, target, std::move(attrs));
  } else {
    interlayer_edges_.insert(source, target, std::move(attrs));
  }

  return true;
}

bool DynamicSceneGraph::addOrUpdateEdge(NodeId source,
                                        NodeId target,
                                        EdgeAttributes::Ptr&& edge_info) {
  if (hasEdge(source, target)) {
    return setEdgeAttributes(source, target, std::move(edge_info));
  } else {
    return insertEdge(source, target, std::move(edge_info));
  }
}

bool DynamicSceneGraph::setNodeAttributes(NodeId node, NodeAttributes::Ptr&& attrs) {
  auto iter = node_lookup_.find(node);
  if (iter == node_lookup_.end()) {
    return false;
  }

  getNodePtr(node, iter->second)->attributes_ = std::move(attrs);
  return true;
}

bool DynamicSceneGraph::setEdgeAttributes(NodeId source,
                                          NodeId target,
                                          EdgeAttributes::Ptr&& attrs) {
  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  SceneGraphEdge* edge;
  if (source_key == target_key) {
    edge = layerFromKey(source_key).edgeContainer().find(source, target);
  } else {
    if (source_key.dynamic || target_key.dynamic) {
      edge = dynamic_interlayer_edges_.find(source, target);
    } else {
      edge = interlayer_edges_.find(source, target);
    }
  }

  if (!edge) {
    return false;
  }

  edge->info = std::move(attrs);
  return true;
}

bool DynamicSceneGraph::hasLayer(LayerId layer_id) const {
  return layers_.count(layer_id) != 0;
}

bool DynamicSceneGraph::hasLayer(LayerId layer, LayerPrefix layer_prefix) const {
  if (!dynamic_layers_.count(layer)) {
    return 0;
  }

  return dynamic_layers_.at(layer).count(layer_prefix) != 0;
}

bool DynamicSceneGraph::hasNode(NodeId node_id) const {
  return node_lookup_.count(node_id) != 0;
}

NodeStatus DynamicSceneGraph::checkNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return NodeStatus::NONEXISTENT;
  }

  return layerFromKey(iter->second).checkNode(node_id);
}

bool DynamicSceneGraph::hasEdge(NodeId source, NodeId target) const {
  return hasEdge(source, target, nullptr, nullptr);
}

const SceneGraphLayer& DynamicSceneGraph::getLayer(LayerId layer) const {
  if (!hasLayer(layer)) {
    std::stringstream ss;
    ss << "missing layer " << layer;
    throw std::out_of_range(ss.str());
  }

  return *layers_.at(layer);
}

const DynamicSceneGraphLayer& DynamicSceneGraph::getLayer(LayerId layer,
                                                          LayerPrefix prefix) const {
  if (!hasLayer(layer, prefix)) {
    std::stringstream ss;
    ss << "missing dynamic layer " << layer << "(" << prefix.str() << ")";
    throw std::out_of_range(ss.str());
  }

  return *dynamic_layers_.at(layer).at(prefix);
}
const Node& DynamicSceneGraph::getNode(NodeId node_id) const {
  const auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).getLabel() + "'");
  }

  return *node;
}

const Node* DynamicSceneGraph::findNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return nullptr;
  }

  return getNodePtr(node_id, iter->second);
}

std::optional<LayerKey> DynamicSceneGraph::getLayerForNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  return iter->second;
}

const Edge& DynamicSceneGraph::getEdge(NodeId source, NodeId target) const {
  const auto edge = findEdge(source, target);
  if (!edge) {
    std::stringstream ss;
    ss << "Missing edge '" << EdgeKey(source, target) << "'";
    throw std::out_of_range(ss.str());
  }

  return *edge;
}

const Edge* DynamicSceneGraph::findEdge(NodeId source, NodeId target) const {
  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  if (source_key == target_key) {
    return layerFromKey(source_key).findEdge(source, target);
  }

  if (source_key.dynamic || target_key.dynamic) {
    return dynamic_interlayer_edges_.find(source, target);
  } else {
    return interlayer_edges_.find(source, target);
  }
}

bool DynamicSceneGraph::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  const auto info = node_lookup_.at(node_id);
  auto node = getNodePtr(node_id, info);

  const auto children_to_erase = node->children_;
  for (const auto& target : children_to_erase) {
    removeInterlayerEdge(node_id, target);
  }

  const auto parents_to_erase = node->parents_;
  for (const auto& target : parents_to_erase) {
    removeInterlayerEdge(node_id, target);
  }

  layerFromKey(info).removeNode(node_id);
  node_lookup_.erase(node_id);
  return true;
}

bool DynamicSceneGraph::removeEdge(NodeId source, NodeId target) {
  LayerKey source_key, target_key;
  if (!hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  if (!source_key || !target_key) {
    return false;
  }

  if (source_key == target_key) {
    return layerFromKey(source_key).removeEdge(source, target);
  }

  removeInterlayerEdge(source, target, source_key, target_key);
  return true;
}

bool DynamicSceneGraph::isDynamic(NodeId source) const {
  auto iter = node_lookup_.find(source);
  if (iter == node_lookup_.end()) {
    return false;
  }

  return iter->second.dynamic;
}

size_t DynamicSceneGraph::numLayers() const {
  const size_t static_size = layers_.size();

  size_t unique_dynamic_layers = 0;
  for (const auto& id_layer_group_pair : dynamic_layers_) {
    if (!layers_.count(id_layer_group_pair.first)) {
      unique_dynamic_layers++;
    }
  }

  return static_size + unique_dynamic_layers;
}

size_t DynamicSceneGraph::numDynamicLayersOfType(LayerId layer) const {
  if (!dynamic_layers_.count(layer)) {
    return 0;
  }

  return dynamic_layers_.at(layer).size();
}

size_t DynamicSceneGraph::numDynamicLayers() const {
  size_t num_layers = 0;
  for (const auto& id_layer_group : dynamic_layers_) {
    num_layers += id_layer_group.second.size();
  }
  return num_layers;
}

size_t DynamicSceneGraph::numNodes(bool include_mesh) const {
  return numStaticNodes() + numDynamicNodes() +
         ((mesh_ == nullptr || !include_mesh) ? 0 : mesh_->numVertices());
}

size_t DynamicSceneGraph::numStaticNodes() const {
  size_t total_nodes = 0u;
  for (const auto& id_layer_pair : layers_) {
    total_nodes += id_layer_pair.second->numNodes();
  }

  return total_nodes;
}

size_t DynamicSceneGraph::numDynamicNodes() const {
  size_t total_nodes = 0u;
  for (const auto& layer_group : dynamic_layers_) {
    for (const auto& prefix_layer_pair : layer_group.second) {
      total_nodes += prefix_layer_pair.second->numNodes();
    }
  }

  return total_nodes;
}

size_t DynamicSceneGraph::numEdges() const {
  return numStaticEdges() + numDynamicEdges();
}

size_t DynamicSceneGraph::numStaticEdges() const {
  size_t total_edges = interlayer_edges_.size();
  for (const auto& id_layer_pair : layers_) {
    total_edges += id_layer_pair.second->numEdges();
  }

  return total_edges;
}

size_t DynamicSceneGraph::numDynamicEdges() const {
  size_t total_edges = dynamic_interlayer_edges_.size();

  for (const auto& id_group_pair : dynamic_layers_) {
    for (const auto& prefix_layer_pair : id_group_pair.second) {
      total_edges += prefix_layer_pair.second->numEdges();
    }
  }

  return total_edges;
}

bool DynamicSceneGraph::empty() const { return numNodes() == 0; }

Eigen::Vector3d DynamicSceneGraph::getPosition(NodeId node) const {
  auto iter = node_lookup_.find(node);
  if (iter == node_lookup_.end()) {
    throw std::out_of_range("node " + NodeSymbol(node).getLabel() +
                            " is not in the graph");
  }

  auto info = iter->second;
  if (info.dynamic) {
    return dynamic_layers_.at(info.layer).at(info.prefix)->getPosition(node);
  }

  return layers_.at(info.layer)->getPosition(node);
}

bool DynamicSceneGraph::mergeNodes(NodeId node_from, NodeId node_to) {
  if (!hasNode(node_from) || !hasNode(node_to)) {
    return false;
  }

  if (node_from == node_to) {
    return false;
  }

  const auto info = node_lookup_.at(node_from);

  if (info != node_lookup_.at(node_to)) {
    return false;  // Cannot merge nodes of different layers
  }

  Node* node = layers_[info.layer]->nodes_.at(node_from).get();

  // Remove parent
  const auto parents_to_rewire = node->parents_;
  for (const auto& target : parents_to_rewire) {
    rewireInterlayerEdge(node_from, node_to, target);
  }

  // Reconnect children
  const auto children_to_rewire = node->children_;
  for (const auto& target : children_to_rewire) {
    rewireInterlayerEdge(node_from, node_to, target);
  }

  // TODO(nathan) dynamic merge
  layers_[info.layer]->mergeNodes(node_from, node_to);
  node_lookup_.erase(node_from);
  return true;
}

bool DynamicSceneGraph::updateFromLayer(SceneGraphLayer& other_layer,
                                        std::unique_ptr<Edges>&& edges) {
  // TODO(nathan) consider condensing with mergeGraph
  if (!layers_.count(other_layer.id)) {
    SG_LOG(ERROR) << "Scene graph does not have layer: " << other_layer.id << std::endl;
    return false;
  }

  auto& internal_layer = *layers_.at(other_layer.id);
  for (auto& id_node_pair : other_layer.nodes_) {
    if (internal_layer.hasNode(id_node_pair.first)) {
      // just copy the attributes (prior edge information should be preserved)
      internal_layer.nodes_[id_node_pair.first]->attributes_ =
          std::move(id_node_pair.second->attributes_);
    } else {
      // we need to let the scene graph know about new nodes
      node_lookup_[id_node_pair.first] = internal_layer.id;
      internal_layer.nodes_[id_node_pair.first] = std::move(id_node_pair.second);
      internal_layer.nodes_status_[id_node_pair.first] = NodeStatus::NEW;
    }
  }

  // we just invalidated all the nodes in the other layer, so we better reset everything
  other_layer.reset();

  if (!edges) {
    return true;
  }

  for (auto& id_edge_pair : *edges) {
    auto& edge = id_edge_pair.second;
    if (internal_layer.hasEdge(edge.source, edge.target)) {
      internal_layer.edges_.edges.at(id_edge_pair.first).info = std::move(edge.info);
      continue;
    }

    internal_layer.insertEdge(edge.source, edge.target, std::move(edge.info));
  }

  // we just invalidated all the info for the new edges, so reset the edges
  edges.reset();
  return true;
}

bool DynamicSceneGraph::mergeGraph(const DynamicSceneGraph& other,
                                   const GraphMergeConfig& config) {
  for (auto&& [l_id, other_layers] : other.dynamicLayers()) {
    for (auto&& [prefix, other_layer] : other_layers) {
      if (!hasLayer(l_id, prefix)) {
        createDynamicLayer(l_id, prefix);
      }

      dynamic_layers_[l_id][prefix]->mergeLayer(*other_layer, config, &node_lookup_);
    }
  }

  for (auto&& [l_id, other_layer] : other.layers()) {
    if (!hasLayer(l_id)) {
      continue;
    }

    std::vector<NodeId> removed_nodes;
    other_layer->getRemovedNodes(removed_nodes, config.clear_removed);
    for (const auto& removed_id : removed_nodes) {
      removeNode(removed_id);
    }

    std::vector<EdgeKey> removed_edges;
    other_layer->edges_.getRemoved(removed_edges, config.clear_removed);
    for (const auto& removed_edge : removed_edges) {
      layers_[l_id]->removeEdge(removed_edge.k1, removed_edge.k2);
    }

    layers_[l_id]->mergeLayer(*other_layer, config, &node_lookup_);
  }

  for (const auto& id_edge_pair : other.interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    NodeId new_source = config.getMergedId(edge.source);
    NodeId new_target = config.getMergedId(edge.target);
    if (new_source == new_target) {
      continue;
    }

    if (config.enforce_parent_constraints) {
      insertParentEdge(new_source, new_target, edge.info->clone());
    } else {
      insertEdge(new_source, new_target, edge.info->clone());
    }
  }

  for (const auto& id_edge_pair : other.dynamic_interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    NodeId new_source = config.getMergedId(edge.source);
    NodeId new_target = config.getMergedId(edge.target);
    if (new_source == new_target) {
      continue;
    }

    if (config.enforce_parent_constraints) {
      insertParentEdge(new_source, new_target, edge.info->clone());
    } else {
      insertEdge(new_source, new_target, edge.info->clone());
    }
  }

  // TODO(Yun) check the other mesh info (faces, vertices etc. )
  return true;
}

std::vector<NodeId> DynamicSceneGraph::getRemovedNodes(bool clear_removed) {
  std::vector<NodeId> to_return;
  visitLayers([&](LayerKey, BaseLayer* layer) {
    layer->getRemovedNodes(to_return, clear_removed);
  });
  return to_return;
}

std::vector<NodeId> DynamicSceneGraph::getNewNodes(bool clear_new) {
  std::vector<NodeId> to_return;
  visitLayers(
      [&](LayerKey, BaseLayer* layer) { layer->getNewNodes(to_return, clear_new); });
  return to_return;
}

std::vector<EdgeKey> DynamicSceneGraph::getRemovedEdges(bool clear_removed) {
  std::vector<EdgeKey> to_return;
  visitLayers([&](LayerKey, BaseLayer* layer) {
    layer->getRemovedEdges(to_return, clear_removed);
  });

  interlayer_edges_.getRemoved(to_return, clear_removed);
  dynamic_interlayer_edges_.getRemoved(to_return, clear_removed);
  return to_return;
}

std::vector<EdgeKey> DynamicSceneGraph::getNewEdges(bool clear_new) {
  std::vector<EdgeKey> to_return;
  visitLayers(
      [&](LayerKey, BaseLayer* layer) { layer->getNewEdges(to_return, clear_new); });

  interlayer_edges_.getNew(to_return, clear_new);
  dynamic_interlayer_edges_.getNew(to_return, clear_new);
  return to_return;
}

void DynamicSceneGraph::markEdgesAsStale() {
  for (auto& id_layer_pair : layers_) {
    id_layer_pair.second->edges_.setStale();
  }
  for (auto& layer_map : dynamic_layers_) {
    for (auto& id_layer_pair : layer_map.second) {
      id_layer_pair.second->edges_.setStale();
    }
  }

  dynamic_interlayer_edges_.setStale();
  interlayer_edges_.setStale();
}

void DynamicSceneGraph::removeAllStaleEdges() {
  for (auto& id_layer_pair : layers_) {
    removeStaleEdges(id_layer_pair.second->edges_);
  }
  for (auto& layer_map : dynamic_layers_) {
    for (auto& id_layer_pair : layer_map.second) {
      removeStaleEdges(id_layer_pair.second->edges_);
    }
  }

  removeStaleEdges(interlayer_edges_);
  removeStaleEdges(dynamic_interlayer_edges_);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::clone() const {
  auto to_return = std::make_shared<DynamicSceneGraph>(layer_ids);
  for (const auto id_layer_pair : node_lookup_) {
    auto node = getNodePtr(id_layer_pair.first, id_layer_pair.second);
    if (id_layer_pair.second.dynamic) {
      to_return->emplacePrevDynamicNode(
          node->layer, node->id, node->timestamp.value(), node->attributes_->clone());
    } else {
      to_return->emplaceNode(node->layer, node->id, node->attributes_->clone());
    }
  }

  for (const auto& id_layer_pair : layers_) {
    for (const auto& id_edge_pair : id_layer_pair.second->edges()) {
      const auto& edge = id_edge_pair.second;
      to_return->insertEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  for (const auto& id_layer_group : dynamic_layers_) {
    for (const auto& prefix_layer_pair : id_layer_group.second) {
      for (const auto& id_edge_pair : prefix_layer_pair.second->edges()) {
        const auto& edge = id_edge_pair.second;
        to_return->insertEdge(edge.source, edge.target, edge.info->clone());
      }
    }
  }

  for (const auto& id_edge_pair : interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    to_return->insertEdge(edge.source, edge.target, edge.info->clone());
  }

  for (const auto& id_edge_pair : dynamic_interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    to_return->insertEdge(edge.source, edge.target, edge.info->clone());
  }

  if (mesh_) {
    to_return->mesh_ = mesh_->clone();
  }

  return to_return;
}

void DynamicSceneGraph::save(std::string filepath, bool include_mesh) const {
  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    io::saveDsgJson(*this, filepath, include_mesh);
    return;
  }

  // Can only be binary after verification.
  io::saveDsgBinary(*this, filepath, include_mesh);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::load(std::string filepath) {
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("graph file does not exist: " + filepath);
  }

  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    return io::loadDsgJson(filepath);
  }

  // Can only be binary after verification (worstcase: throws meaningful error)
  return io::loadDsgBinary(filepath);
}

void DynamicSceneGraph::setMesh(const std::shared_ptr<Mesh>& mesh) { mesh_ = mesh; }

bool DynamicSceneGraph::hasMesh() const { return mesh_ != nullptr; }

Mesh::Ptr DynamicSceneGraph::mesh() const { return mesh_; }

BaseLayer& DynamicSceneGraph::layerFromKey(const LayerKey& key) {
  const auto& layer = static_cast<const DynamicSceneGraph*>(this)->layerFromKey(key);
  return const_cast<BaseLayer&>(layer);
}

const BaseLayer& DynamicSceneGraph::layerFromKey(const LayerKey& key) const {
  if (key.dynamic) {
    return *dynamic_layers_.at(key.layer).at(key.prefix);
  } else {
    return *layers_.at(key.layer);
  }
}

SceneGraphNode* DynamicSceneGraph::getNodePtr(NodeId node, const LayerKey& info) const {
  if (info.dynamic) {
    const auto idx = NodeSymbol(node).categoryId();
    return dynamic_layers_.at(info.layer).at(info.prefix)->nodes_.at(idx).get();
  } else {
    return layers_.at(info.layer)->nodes_.at(node).get();
  }
}

bool DynamicSceneGraph::hasEdge(NodeId source,
                                NodeId target,
                                LayerKey* source_key,
                                LayerKey* target_key) const {
  auto source_iter = node_lookup_.find(source);
  if (source_iter == node_lookup_.end()) {
    return false;
  }

  auto target_iter = node_lookup_.find(target);
  if (target_iter == node_lookup_.end()) {
    return false;
  }

  if (source_key != nullptr) {
    *source_key = source_iter->second;
  }

  if (target_key != nullptr) {
    *target_key = target_iter->second;
  }

  if (source_iter->second == target_iter->second) {
    return layerFromKey(source_iter->second).hasEdge(source, target);
  }

  if (source_iter->second.dynamic || target_iter->second.dynamic) {
    return dynamic_interlayer_edges_.contains(source, target);
  } else {
    return interlayer_edges_.contains(source, target);
  }
}

void DynamicSceneGraph::addAncestry(NodeId source,
                                    NodeId target,
                                    const LayerKey& source_key,
                                    const LayerKey& target_key) {
  auto* source_node = getNodePtr(source, source_key);
  auto* target_node = getNodePtr(target, target_key);
  if (source_key.isParent(target_key)) {
    source_node->children_.insert(target);
    target_node->parents_.insert(source);
  } else if (target_key.isParent(source_key)) {
    target_node->children_.insert(source);
    source_node->parents_.insert(target);
  } else {
    source_node->siblings_.insert(target);
    target_node->siblings_.insert(source);
  }
}

void DynamicSceneGraph::removeAncestry(NodeId source,
                                       NodeId target,
                                       const LayerKey& source_key,
                                       const LayerKey& target_key) {
  auto* source_node = getNodePtr(source, source_key);
  auto* target_node = getNodePtr(target, target_key);
  if (source_key.isParent(target_key)) {
    source_node->children_.erase(target);
    target_node->parents_.erase(source);
  } else if (target_key.isParent(source_key)) {
    target_node->children_.erase(source);
    source_node->parents_.erase(target);
  } else {
    source_node->siblings_.erase(target);
    target_node->siblings_.erase(source);
  }
}

void DynamicSceneGraph::dropAllParents(NodeId source,
                                       NodeId target,
                                       const LayerKey& source_key,
                                       const LayerKey& target_key) {
  auto* source_node = getNodePtr(source, source_key);
  auto* target_node = getNodePtr(target, target_key);
  const auto source_is_parent = source_key.isParent(target_key);
  std::set<NodeId> parents_to_clear =
      source_is_parent ? target_node->parents_ : source_node->parents_;
  NodeId child_to_clear = source_is_parent ? target : source;
  for (const auto parent_to_clear : parents_to_clear) {
    removeEdge(child_to_clear, parent_to_clear);
  }
}

void DynamicSceneGraph::removeInterlayerEdge(NodeId source,
                                             NodeId target,
                                             const LayerKey& source_key,
                                             const LayerKey& target_key) {
  removeAncestry(source, target, source_key, target_key);
  if (source_key.dynamic || target_key.dynamic) {
    dynamic_interlayer_edges_.remove(source, target);
  } else {
    interlayer_edges_.remove(source, target);
  }
}

void DynamicSceneGraph::removeInterlayerEdge(NodeId n1, NodeId n2) {
  removeInterlayerEdge(n1, n2, node_lookup_.at(n1), node_lookup_.at(n2));
}

void DynamicSceneGraph::rewireInterlayerEdge(NodeId source,
                                             NodeId new_source,
                                             NodeId target) {
  if (source == new_source) {
    return;
  }

  const auto& source_key = node_lookup_.at(source);

  LayerKey new_source_key, target_key;
  if (hasEdge(new_source, target, &new_source_key, &target_key)) {
    removeInterlayerEdge(source, target, source_key, target_key);
    return;
  }

  // removes record of source -> target in nodes and adds new_source -> target instead
  removeAncestry(source, target, source_key, target_key);
  addAncestry(new_source, target, new_source_key, target_key);

  // TODO(nathan) edges can technically jump from dynamic to static, so problems with
  // index not being available in other container
  EdgeAttributes::Ptr attrs;
  if (source_key.dynamic || target_key.dynamic) {
    const auto edge = dynamic_interlayer_edges_.find(source, target);
    if (edge) {
      attrs = edge->info->clone();
      dynamic_interlayer_edges_.remove(source, target);
    }
  } else {
    const auto edge = interlayer_edges_.find(source, target);
    if (edge) {
      attrs = edge->info->clone();
      interlayer_edges_.remove(source, target);
    }
  }

  if (!attrs) {
    // we somehow didn't have the edge
    return;
  }

  if (new_source_key.dynamic || target_key.dynamic) {
    dynamic_interlayer_edges_.insert(new_source, target, std::move(attrs));
  } else {
    interlayer_edges_.insert(new_source, target, std::move(attrs));
  }
}

void DynamicSceneGraph::removeStaleEdges(EdgeContainer& edges) {
  for (const auto& edge_key_pair : edges.stale_edges) {
    if (edge_key_pair.second) {
      removeEdge(edge_key_pair.first.k1, edge_key_pair.first.k2);
    }
  }
}

void DynamicSceneGraph::visitLayers(const LayerVisitor& cb) {
  for (auto& id_layer_pair : layers_) {
    cb(id_layer_pair.first, id_layer_pair.second.get());
  }

  for (auto& id_group_pair : dynamic_layers_) {
    for (auto& prefix_layer_pair : id_group_pair.second) {
      cb(LayerKey(id_group_pair.first, prefix_layer_pair.first),
         prefix_layer_pair.second.get());
    }
  }
}

}  // namespace spark_dsg
