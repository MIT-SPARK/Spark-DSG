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
#include "spark_dsg/mesh.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/printing.h"
#include "spark_dsg/serialization/file_io.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;
using Layer = SceneGraphLayer;
using LayerCallback = std::function<void(LayerKey, Layer&)>;
using ConstLayerCallback = std::function<void(LayerKey, const Layer&)>;

DynamicSceneGraph::DynamicSceneGraph(bool empty)
    : DynamicSceneGraph(empty ? LayerIds{} : LayerIds{2, 3, 4, 5},
                        empty ? LayerNames{}
                              : LayerNames{{DsgLayers::OBJECTS, 2},
                                           {DsgLayers::AGENTS, 2},
                                           {DsgLayers::PLACES, 3},
                                           {DsgLayers::ROOMS, 4},
                                           {DsgLayers::BUILDINGS, 5}}) {}

DynamicSceneGraph::DynamicSceneGraph(const LayerIds& layer_ids,
                                     const std::map<std::string, LayerId>& layer_names)
    : metadata(nlohmann::json::object()),
      layer_ids_(layer_ids),
      layer_names_(layer_names) {
  // TODO(nathan) fill layer_ids_ from layer_names
  clear();
}

void DynamicSceneGraph::clear() {
  layers_.clear();
  intralayer_groups_.clear();

  node_lookup_.clear();

  interlayer_edges_.reset();

  mesh_.reset();

  for (const auto& id : layer_ids_) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }
}

void DynamicSceneGraph::reset(const LayerIds& new_layer_ids) {
  layer_ids_ = new_layer_ids;
  clear();
}

bool DynamicSceneGraph::hasLayer(LayerId layer_id, IntralayerId intralayer_id) const {
  return findLayer(layer_id, intralayer_id) != nullptr;
}

bool DynamicSceneGraph::hasLayer(const std::string& layer_name,
                                 IntralayerId intralayer_id) const {
  auto iter = layer_names_.find(layer_name);
  if (iter == layer_names_.end()) {
    return false;
  }

  return hasLayer(iter->second, intralayer_id);
}

const Layer* DynamicSceneGraph::findLayer(LayerId layer,
                                          IntralayerId intralayer_id) const {
  if (!intralayer_id) {
    auto iter = layers_.find(layer);
    return iter == layers_.end() ? nullptr : iter->second.get();
  }

  auto group = intralayer_groups_.find(layer);
  if (group == intralayer_groups_.end()) {
    return nullptr;
  }

  auto iter = group->second.find(intralayer_id);
  return iter == group->second.end() ? nullptr : iter->second.get();
}

const Layer* DynamicSceneGraph::findLayer(const std::string& name,
                                          IntralayerId intralayer_id) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    return nullptr;
  }

  return findLayer(iter->second, intralayer_id);
}

const Layer& DynamicSceneGraph::getLayer(LayerId layer_id,
                                         IntralayerId intralayer_id) const {
  auto layer = findLayer(layer_id, intralayer_id);
  if (!layer) {
    std::stringstream ss;
    ss << "missing layer " << LayerKey{layer_id, intralayer_id};
    throw std::out_of_range(ss.str());
  }

  return *layer;
}

const Layer& DynamicSceneGraph::getLayer(const std::string& name,
                                         IntralayerId intralayer_id) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    std::stringstream ss;
    ss << "missing layer '" << name << "'";
    if (intralayer_id) {
      ss << " [intralayer_id: " << intralayer_id << "]";
    }

    throw std::out_of_range(ss.str());
  }

  return getLayer(name, intralayer_id);
}

const Layer& DynamicSceneGraph::addLayer(LayerId layer, IntralayerId intralayer_id) {
  return layerFromKey({layer, intralayer_id});
}

void DynamicSceneGraph::removeLayer(LayerId layer, IntralayerId intralayer_id) {
  if (!intralayer_id) {
    layers_.erase(layer);
  }

  auto iter = intralayer_groups_.find(layer);
  if (iter == intralayer_groups_.end()) {
    return;
  }

  iter->second.erase(intralayer_id);
  if (iter->second.empty()) {
    intralayer_groups_.erase(iter);
  }
}

bool DynamicSceneGraph::emplaceNode(LayerKey layer_key,
                                    NodeId node_id,
                                    std::unique_ptr<NodeAttributes>&& attrs) {
  if (node_lookup_.count(node_id)) {
    return false;
  }

  auto& layer = layerFromKey(layer_key);
  const auto successful = layer.emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_.emplace(node_id, layer_key);
  }

  return successful;
}

bool DynamicSceneGraph::addOrUpdateNode(LayerKey layer_key,
                                        NodeId node_id,
                                        std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = node_lookup_.find(node_id);
  if (iter != node_lookup_.end()) {
    getNodePtr(node_id, iter->second)->attributes_ = std::move(attrs);
    return true;
  }

  auto& layer = layerFromKey(layer_key);
  const auto successful = layer.emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_.emplace(node_id, layer_key);
  }

  return successful;
}

bool DynamicSceneGraph::insertEdge(NodeId source,
                                   NodeId target,
                                   std::unique_ptr<EdgeAttributes>&& edge_info) {
  LayerKey source_key, target_key;
  if (hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  if (source_key == target_key) {
    return layerFromKey(source_key).insertEdge(source, target, std::move(attrs));
  }

  // add information to the nodes about the node relationship
  addAncestry(source, target, source_key, target_key);
  interlayer_edges_.insert(source, target, std::move(attrs));
  return true;
}

bool DynamicSceneGraph::insertParentEdge(NodeId source,
                                         NodeId target,
                                         std::unique_ptr<EdgeAttributes>&& edge_info) {
  LayerKey source_key, target_key;
  if (hasEdge(source, target, &source_key, &target_key)) {
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  // force single parent to exist
  if (source_key != target_key) {
    dropAllParents(source, target, source_key, target_key);
  }

  addAncestry(source, target, source_key, target_key);
  interlayer_edges_.insert(source, target, std::move(attrs));
  return true;
}

bool DynamicSceneGraph::addOrUpdateEdge(NodeId source,
                                        NodeId target,
                                        std::unique_ptr<EdgeAttributes>&& edge_info) {
  if (hasEdge(source, target)) {
    return setEdgeAttributes(source, target, std::move(edge_info));
  } else {
    return insertEdge(source, target, std::move(edge_info));
  }
}

bool DynamicSceneGraph::setNodeAttributes(NodeId node,
                                          std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = node_lookup_.find(node);
  if (iter == node_lookup_.end()) {
    return false;
  }

  getNodePtr(node, iter->second)->attributes_ = std::move(attrs);
  return true;
}

bool DynamicSceneGraph::setEdgeAttributes(NodeId source,
                                          NodeId target,
                                          std::unique_ptr<EdgeAttributes>&& attrs) {
  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  SceneGraphEdge* edge;
  if (source_key == target_key) {
    edge = layerFromKey(source_key).edges_.find(source, target);
  } else {
    edge = interlayer_edges_.find(source, target);
  }

  if (!edge) {
    return false;
  }

  edge->info = std::move(attrs);
  return true;
}

bool DynamicSceneGraph::hasNode(NodeId node_id) const {
  return node_lookup_.count(node_id);
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

  return interlayer_edges_.find(source, target);
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

  if (source_key == target_key) {
    return layerFromKey(source_key).removeEdge(source, target);
  }

  removeInterlayerEdge(source, target, source_key, target_key);
  return true;
}

size_t DynamicSceneGraph::numLayers() const {
  const size_t static_size = layers_.size();

  size_t unique_layer_groups = 0;
  for (const auto& [layer_id, group] : intralayer_groups_) {
    if (!layers_.count(layer_id)) {
      ++unique_layer_groups;
    }
  }

  return static_size + unique_layer_groups;
}

size_t DynamicSceneGraph::numNodes() const {
  size_t total_nodes = 0u;
  for (const auto& [layer_id, layer] : layers_) {
    total_nodes += layer->numNodes();
  }

  for (const auto& [layer_id, group] : intralayer_groups_) {
    for (const auto& [intralayer_id, layer] : group) {
      total_nodes += layer->numNodes();
    }
  }

  return total_nodes;
}

size_t DynamicSceneGraph::numEdges() const {
  size_t total_edges = interlayer_edges_.size();
  for (const auto& [layer_id, layer] : layers_) {
    total_edges += layer->numEdges();
  }

  for (const auto& [layer_id, group] : intralayer_groups_) {
    for (const auto& [intralayer_id, layer] : group) {
      total_edges += layer->numEdges();
    }
  }

  return total_edges;
}

bool DynamicSceneGraph::empty() const { return numNodes() == 0; }

Eigen::Vector3d DynamicSceneGraph::getPosition(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    throw std::out_of_range("node " + NodeSymbol(node_id).getLabel() +
                            " is not in the graph");
  }

  const auto node = getNodePtr(node_id, iter->second);
  return node->attributes().position;
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

bool DynamicSceneGraph::updateFromLayer(const SceneGraphLayer& other_layer,
                                        const Edges& edges) {
  // TODO(nathan) consider condensing with mergeGraph
  for (auto& [node_id, node] : other_layer.nodes_) {
    addOrUpdateNode(node->layer, node_id, node->attributes_->clone());
  }

  for (auto& [edge_key, edge] : edges) {
    addOrUpdateEdge(edge_key.k1, edge_key.k2, edge.info->clone());
  }

  return true;
}

bool DynamicSceneGraph::mergeGraph(const DynamicSceneGraph& other,
                                   const GraphMergeConfig& config) {
  other.visitLayers([&](LayerKey layer_key, const SceneGraphLayer& other_layer) {
    auto& layer = layerFromKey(layer_key);

    std::vector<NodeId> removed_nodes;
    other_layer.getRemovedNodes(removed_nodes, config.clear_removed);
    for (const auto& removed_id : removed_nodes) {
      removeNode(removed_id);
    }

    std::vector<EdgeKey> removed_edges;
    other_layer.edges_.getRemoved(removed_edges, config.clear_removed);
    for (const auto& removed_edge : removed_edges) {
      layer.removeEdge(removed_edge.k1, removed_edge.k2);
    }

    std::vector<NodeId> new_nodes;
    layer.mergeLayer(other_layer, config, &new_nodes);
    for (const auto node_id : new_nodes) {
      node_lookup_[node_id] = layer_key;
    }
  });

  for (const auto& [edge_id, edge] : other.interlayer_edges()) {
    NodeId source = config.getMergedId(edge.source);
    NodeId target = config.getMergedId(edge.target);
    if (source == target) {
      continue;
    }

    if (config.enforce_parent_constraints) {
      insertParentEdge(source, target, edge.info->clone());
    } else {
      insertEdge(source, target, edge.info->clone());
    }
  }

  // TODO(Yun) check the other mesh info (faces, vertices etc. )
  return true;
}

std::vector<NodeId> DynamicSceneGraph::getRemovedNodes(bool clear_removed) {
  std::vector<NodeId> to_return;
  visitLayers(
      [&](LayerKey, Layer& layer) { layer.getRemovedNodes(to_return, clear_removed); });
  return to_return;
}

std::vector<NodeId> DynamicSceneGraph::getNewNodes(bool clear_new) {
  std::vector<NodeId> to_return;
  visitLayers([&](LayerKey, Layer& layer) { layer.getNewNodes(to_return, clear_new); });
  return to_return;
}

std::vector<EdgeKey> DynamicSceneGraph::getRemovedEdges(bool clear_removed) {
  std::vector<EdgeKey> to_return;
  visitLayers(
      [&](LayerKey, Layer& layer) { layer.getRemovedEdges(to_return, clear_removed); });

  interlayer_edges_.getRemoved(to_return, clear_removed);
  return to_return;
}

std::vector<EdgeKey> DynamicSceneGraph::getNewEdges(bool clear_new) {
  std::vector<EdgeKey> to_return;
  visitLayers([&](LayerKey, Layer& layer) { layer.getNewEdges(to_return, clear_new); });

  interlayer_edges_.getNew(to_return, clear_new);
  return to_return;
}

void DynamicSceneGraph::markEdgesAsStale() {
  for (auto& [layer_id, layer] : layers_) {
    layer->edges_.setStale();
  }
  for (auto& [layer_id, groups] : intralayer_groups_) {
    for (auto& [intralayer_id, layer] : groups) {
      layer->edges_.setStale();
    }
  }

  interlayer_edges_.setStale();
}

void DynamicSceneGraph::removeAllStaleEdges() {
  for (auto& [layer_id, layer] : layers_) {
    removeStaleEdges(layer->edges_);
  }

  for (auto& [layer_id, group] : intralayer_groups_) {
    for (auto& [intralayer_id, layer] : group) {
      removeStaleEdges(layer->edges_);
    }
  }

  removeStaleEdges(interlayer_edges_);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::clone() const {
  auto to_return = std::make_shared<DynamicSceneGraph>(layer_ids_);
  for (const auto [node_id, layer_key] : node_lookup_) {
    auto node = getNodePtr(node_id, layer_key);
    to_return->addOrUpdateNode(node->layer, node->id, node->attributes_->clone());
  }

  for (const auto& [layer_id, layer] : layers_) {
    for (const auto& [edge_id, edge] : layer->edges()) {
      to_return->insertEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  for (const auto& [layer_id, group] : intralayer_groups_) {
    for (const auto& [prefix, layer] : group) {
      for (const auto& [edge_id, edge] : layer->edges()) {
        to_return->insertEdge(edge.source, edge.target, edge.info->clone());
      }
    }
  }

  for (const auto& [edge_id, edge] : interlayer_edges()) {
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

const DynamicSceneGraph::LayerIds& DynamicSceneGraph::layer_ids() const {
  return layer_ids_;
}

Layer& DynamicSceneGraph::layerFromKey(const LayerKey& key) {
  if (!key.intralayer_id) {
    auto iter = layers_.emplace(key.layer, std::make_unique<Layer>(key.layer)).first;
    return *iter->second;
  }

  auto iter = intralayer_groups_.find(key.layer);
  if (iter == intralayer_groups_.end()) {
    iter = intralayer_groups_.emplace(key.layer, IntralayerGroup()).first;
  }

  auto id_layer_pair =
      iter->second.emplace(key.intralayer_id, std::make_unique<Layer>(key.layer)).first;
  return *id_layer_pair->second;
}

const Layer& DynamicSceneGraph::layerFromKey(const LayerKey& key) const {
  return const_cast<DynamicSceneGraph*>(this)->layerFromKey(key);
}

SceneGraphNode* DynamicSceneGraph::getNodePtr(NodeId node, const LayerKey& info) const {
  return layerFromKey(info).nodes_.at(node).get();
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

  return interlayer_edges_.contains(source, target);
}

void DynamicSceneGraph::addAncestry(NodeId source,
                                    NodeId target,
                                    const LayerKey& source_key,
                                    const LayerKey& target_key) {
  auto* source_node = getNodePtr(source, source_key);
  auto* target_node = getNodePtr(target, target_key);
  if (source_key.isParentOf(target_key)) {
    source_node->children_.insert(target);
    target_node->parents_.insert(source);
  } else if (target_key.isParentOf(source_key)) {
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
  if (source_key.isParentOf(target_key)) {
    source_node->children_.erase(target);
    target_node->parents_.erase(source);
  } else if (target_key.isParentOf(source_key)) {
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
  const auto source_is_parent = source_key.isParentOf(target_key);
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
  interlayer_edges_.remove(source, target);
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

  EdgeAttributes::Ptr attrs;
  const auto edge = interlayer_edges_.find(source, target);
  if (edge) {
    attrs = edge->info->clone();
    interlayer_edges_.remove(source, target);
  }

  if (!attrs) {
    // we somehow didn't have the edge
    return;
  }

  interlayer_edges_.insert(new_source, target, std::move(attrs));
}

void DynamicSceneGraph::removeStaleEdges(EdgeContainer& edges) {
  for (const auto& edge_key_pair : edges.stale_edges) {
    if (edge_key_pair.second) {
      removeEdge(edge_key_pair.first.k1, edge_key_pair.first.k2);
    }
  }
}

void DynamicSceneGraph::visitLayers(const LayerCallback& cb) {
  for (auto& [layer_id, layer] : layers_) {
    cb(layer_id, *layer);
  }

  for (auto& [layer_id, group] : intralayer_groups_) {
    for (auto& [intralayer_id, layer] : group) {
      cb(LayerKey(layer_id, intralayer_id), *layer);
    }
  }
}

void DynamicSceneGraph::visitLayers(const ConstLayerCallback& cb) const {
  const_cast<DynamicSceneGraph*>(this)->visitLayers(
      [&cb](LayerKey key, Layer& layer) { cb(key, layer); });
}

}  // namespace spark_dsg
