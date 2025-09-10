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

using Partitions = DynamicSceneGraph::Partitions;
using LayerNames = DynamicSceneGraph::LayerNames;
using LayerKeys = DynamicSceneGraph::LayerKeys;

std::set<LayerKey> layersFromNames(const LayerNames& layer_names,
                                   const LayerKeys& prev_layers = {}) {
  std::set<LayerKey> layers(prev_layers.begin(), prev_layers.end());
  for (const auto& [name, key] : layer_names) {
    layers.insert(key);
  }

  return layers;
}

bool EdgeLayerInfo::isSameLayer() const { return source == target; }

DynamicSceneGraph::DynamicSceneGraph(bool empty)
    : DynamicSceneGraph(empty ? LayerKeys{} : LayerKeys{2, 3, 4, 5},
                        empty ? LayerNames{}
                              : LayerNames{{DsgLayers::OBJECTS, 2},
                                           {DsgLayers::AGENTS, 2},
                                           {DsgLayers::PLACES, 3},
                                           {DsgLayers::ROOMS, 4},
                                           {DsgLayers::BUILDINGS, 5}}) {}

DynamicSceneGraph::DynamicSceneGraph(const LayerKeys& layer_keys,
                                     const LayerNames& layer_names)
    : layer_keys_(layersFromNames(layer_names, layer_keys)), layer_names_(layer_names) {
  clear();
}

DynamicSceneGraph::Ptr DynamicSceneGraph::fromNames(const LayerNames& layers) {
  return std::make_shared<DynamicSceneGraph>(LayerKeys{}, layers);
}

void DynamicSceneGraph::clear(bool include_mesh) {
  layers_.clear();
  layer_partitions_.clear();

  node_lookup_.clear();
  interlayer_edges_.reset();

  if (include_mesh) {
    mesh_.reset();
  }

  for (const auto& key : layer_keys_) {
    addLayer(key.layer, key.partition);
  }
}

void DynamicSceneGraph::reset(const LayerKeys& layer_keys,
                              const LayerNames& layer_names) {
  layer_keys_ = layersFromNames(layer_names, layer_keys);
  layer_names_ = layer_names;
  clear();
}

bool DynamicSceneGraph::hasLayer(LayerId layer_id, PartitionId partition) const {
  return findLayer(layer_id, partition) != nullptr;
}

bool DynamicSceneGraph::hasLayer(const std::string& layer_name) const {
  auto iter = layer_names_.find(layer_name);
  if (iter == layer_names_.end()) {
    return false;
  }

  return hasLayer(iter->second.layer, iter->second.partition);
}

const Layer* DynamicSceneGraph::findLayer(LayerId layer, PartitionId partition) const {
  if (!partition) {
    auto iter = layers_.find(layer);
    return iter == layers_.end() ? nullptr : iter->second.get();
  }

  auto partitions = layer_partitions_.find(layer);
  if (partitions == layer_partitions_.end()) {
    return nullptr;
  }

  auto iter = partitions->second.find(partition);
  return iter == partitions->second.end() ? nullptr : iter->second.get();
}

const Layer* DynamicSceneGraph::findLayer(const std::string& name) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    return nullptr;
  }

  return findLayer(iter->second.layer, iter->second.partition);
}

const Layer& DynamicSceneGraph::getLayer(LayerId layer_id,
                                         PartitionId partition) const {
  auto layer = findLayer(layer_id, partition);
  if (!layer) {
    std::stringstream ss;
    ss << "missing layer " << LayerKey{layer_id, partition};
    throw std::out_of_range(ss.str());
  }

  return *layer;
}

const Layer& DynamicSceneGraph::getLayer(const std::string& name) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    throw std::out_of_range("missing layer '" + name + "'");
  }

  return getLayer(iter->second.layer, iter->second.partition);
}

const Layer& DynamicSceneGraph::addLayer(LayerId layer_id,
                                         PartitionId partition,
                                         const std::string& name) {
  const LayerKey key{layer_id, partition};
  if (!name.empty()) {
    layer_names_.emplace(name, key);
  }

  return layerFromKey(key);
}

void DynamicSceneGraph::removeLayer(LayerId layer_id, PartitionId partition) {
  LayerKey key{layer_id, partition};
  auto niter = layer_names_.begin();
  while (niter != layer_names_.end()) {
    if (niter->second == key) {
      niter = layer_names_.erase(niter);
    } else {
      ++niter;
    }
  }

  auto layer = findLayer(layer_id, partition);
  if (!layer) {
    return;
  }

  std::vector<NodeId> to_remove;
  for (const auto& [node_id, node] : layer->nodes()) {
    to_remove.push_back(node_id);
  }

  for (const auto& node_id : to_remove) {
    removeNode(node_id);
  }

  if (!partition) {
    layers_.erase(layer_id);
  } else {
    auto iter = layer_partitions_.find(layer_id);
    if (iter == layer_partitions_.end()) {
      return;
    }

    iter->second.erase(partition);
    if (iter->second.empty()) {
      layer_partitions_.erase(iter);
    }
  }

  layer_keys_.erase(key);
}

bool DynamicSceneGraph::emplaceNode(LayerKey key,
                                    NodeId node_id,
                                    std::unique_ptr<NodeAttributes>&& attrs) {
  if (node_lookup_.count(node_id)) {
    return false;
  }

  auto& layer = layerFromKey(key);
  const auto successful = layer.emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_.emplace(node_id, key);
  }

  return successful;
}

bool DynamicSceneGraph::emplaceNode(LayerId layer_id,
                                    NodeId node_id,
                                    std::unique_ptr<NodeAttributes>&& attrs,
                                    PartitionId partition) {
  return emplaceNode(LayerKey{layer_id, partition}, node_id, std::move(attrs));
}

bool DynamicSceneGraph::emplaceNode(const std::string& layer,
                                    NodeId node_id,
                                    std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = layer_names_.find(layer);
  if (iter == layer_names_.end()) {
    return false;
  }

  return emplaceNode(iter->second, node_id, std::move(attrs));
}

bool DynamicSceneGraph::addOrUpdateNode(const std::string& layer,
                                        NodeId node_id,
                                        std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = layer_names_.find(layer);
  if (iter == layer_names_.end()) {
    return false;
  }

  return addOrUpdateNode(
      iter->second.layer, node_id, std::move(attrs), iter->second.partition);
}

bool DynamicSceneGraph::addOrUpdateNode(LayerId layer_id,
                                        NodeId node_id,
                                        std::unique_ptr<NodeAttributes>&& attrs,
                                        PartitionId partition) {
  auto iter = node_lookup_.find(node_id);
  if (iter != node_lookup_.end()) {
    getNodePtr(node_id, iter->second)->attributes_ = std::move(attrs);
    return true;
  }

  const LayerKey key{layer_id, partition};
  auto& layer = layerFromKey(key);
  const auto successful = layer.emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_.emplace(node_id, key);
  }

  return successful;
}

bool DynamicSceneGraph::setNodeAttributes(NodeId node_id,
                                          std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = node_lookup_.find(node_id);
  if (iter != node_lookup_.end()) {
    getNodePtr(node_id, iter->second)->attributes_ = std::move(attrs);
    return true;
  }

  return false;
}

bool DynamicSceneGraph::insertEdge(NodeId source,
                                   NodeId target,
                                   std::unique_ptr<EdgeAttributes>&& edge_info,
                                   bool enforce_parent_constraints) {
  const auto lookup = lookupEdge(source, target);
  if (!lookup.valid || lookup.exists) {
    // skip adding edge if nodes don't exist or if it already exists
    return false;
  }

  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  if (lookup.isSameLayer()) {
    return layerFromKey(lookup.source).insertEdge(source, target, std::move(attrs));
  } else if (enforce_parent_constraints) {
    // force single parent to exist
    dropAllParents(source, target, lookup.source, lookup.target);
  }

  // add information to the nodes about the node relationship
  addAncestry(source, target, lookup.source, lookup.target);
  interlayer_edges_.insert(source, target, std::move(attrs));
  return true;
}

bool DynamicSceneGraph::addOrUpdateEdge(NodeId source,
                                        NodeId target,
                                        std::unique_ptr<EdgeAttributes>&& edge_info,
                                        bool enforce_parent_constraints) {
  auto edge = const_cast<Edge*>(findEdge(source, target));
  if (!edge) {
    return insertEdge(source, target, std::move(edge_info), enforce_parent_constraints);
  }

  edge->info = std::move(edge_info);
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
  return lookupEdge(source, target).exists;
}

const Node& DynamicSceneGraph::getNode(NodeId node_id) const {
  const auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
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
  auto source_key = node_lookup_.find(source);
  if (source_key == node_lookup_.end()) {
    return nullptr;
  }

  auto target_key = node_lookup_.find(target);
  if (target_key == node_lookup_.end()) {
    return nullptr;
  }

  // defer to layers if it is a intralayer edge
  if (source_key->second == target_key->second) {
    return layerFromKey(source_key->second).findEdge(source, target);
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
  const auto lookup = lookupEdge(source, target);
  if (!lookup.exists) {
    // also excludes invalid edges
    return false;
  }

  if (lookup.isSameLayer()) {
    return layerFromKey(lookup.source).removeEdge(source, target);
  }

  removeInterlayerEdge(source, target, lookup.source, lookup.target);
  return true;
}

size_t DynamicSceneGraph::numLayers() const {
  const size_t static_size = layers_.size();

  size_t unique_layer_groups = 0;
  for (const auto& [layer_id, partitions] : layer_partitions_) {
    if (!layers_.count(layer_id)) {
      ++unique_layer_groups;
    }
  }

  return static_size + unique_layer_groups;
}

size_t DynamicSceneGraph::numNodes() const {
  size_t total_nodes = numUnpartitionedNodes();
  for (const auto& [layer_id, partitions] : layer_partitions_) {
    for (const auto& [partition_id, partition] : partitions) {
      total_nodes += partition->numNodes();
    }
  }

  return total_nodes;
}

size_t DynamicSceneGraph::numUnpartitionedNodes() const {
  size_t total_nodes = 0u;
  for (const auto& [layer_id, layer] : layers_) {
    total_nodes += layer->numNodes();
  }

  return total_nodes;
}

size_t DynamicSceneGraph::numEdges() const {
  size_t total_edges = interlayer_edges_.size();
  for (const auto& [layer_id, layer] : layers_) {
    total_edges += layer->numEdges();
  }

  for (const auto& [layer_id, partitions] : layer_partitions_) {
    for (const auto& [partition_id, partition] : partitions) {
      total_edges += partition->numEdges();
    }
  }

  return total_edges;
}

size_t DynamicSceneGraph::numUnpartitionedEdges() const {
  size_t total_edges = 0;
  for (const auto& [layer_id, layer] : layers_) {
    total_edges += layer->numEdges();
  }

  for (const auto& [edge_key, edge] : interlayer_edges_.edges) {
    const auto lookup = lookupEdge(edge_key.k1, edge_key.k2);
    if (!lookup.source.partition && !lookup.target.partition) {
      ++total_edges;
    }
  }

  return total_edges;
}

bool DynamicSceneGraph::empty() const { return numNodes() == 0; }

Eigen::Vector3d DynamicSceneGraph::getPosition(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    throw std::out_of_range("node " + NodeSymbol(node_id).str() +
                            " is not in the graph");
  }

  const auto node = getNodePtr(node_id, iter->second);
  return node->attributes().position;
}

bool DynamicSceneGraph::mergeNodes(NodeId from_id, NodeId to_id) {
  if (from_id == to_id) {
    return false;
  }

  auto node_from = findNode(from_id);
  auto node_to = findNode(to_id);
  if (!node_from || !node_to) {
    return false;
  }

  if (node_from->layer != node_to->layer) {
    return false;  // Cannot merge nodes of different layers
  }

  // Remove parent
  const auto parents_to_rewire = node_from->parents_;
  for (const auto& target : parents_to_rewire) {
    rewireInterlayerEdge(from_id, to_id, target);
  }

  // Reconnect children
  const auto children_to_rewire = node_from->children_;
  for (const auto& target : children_to_rewire) {
    rewireInterlayerEdge(from_id, to_id, target);
  }

  layerFromKey(node_from->layer).mergeNodes(from_id, to_id);
  node_lookup_.erase(from_id);
  return true;
}

bool DynamicSceneGraph::updateFromLayer(const SceneGraphLayer& other_layer,
                                        const Edges& edges) {
  // TODO(nathan) consider condensing with mergeGraph
  const auto key = other_layer.id;
  for (auto& [node_id, node] : other_layer.nodes_) {
    addOrUpdateNode(key.layer, node_id, node->attributes_->clone(), key.partition);
  }

  for (auto& [edge_key, edge] : edges) {
    addOrUpdateEdge(edge_key.k1, edge_key.k2, edge.info->clone());
  }

  return true;
}

bool DynamicSceneGraph::mergeGraph(const DynamicSceneGraph& other,
                                   const GraphMergeConfig& config,
                                   const Eigen::Isometry3d* transform_new_nodes) {
  metadata.add(other.metadata());

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
    layer.mergeLayer(other_layer, config, &new_nodes, transform_new_nodes);
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

    insertEdge(source, target, edge.info->clone(), config.enforce_parent_constraints);
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

bool DynamicSceneGraph::edgeToPartition(const SceneGraphEdge& edge) const {
  const auto lookup = lookupEdge(edge.source, edge.target);
  return lookup.source.partition || lookup.target.partition;
}

void DynamicSceneGraph::markEdgesAsStale() {
  for (auto& [layer_id, layer] : layers_) {
    layer->edges_.setStale();
  }
  for (auto& [layer_id, partitions] : layer_partitions_) {
    for (auto& [partition_id, partition] : partitions) {
      partition->edges_.setStale();
    }
  }

  interlayer_edges_.setStale();
}

void DynamicSceneGraph::removeAllStaleEdges() {
  for (auto& [layer_id, layer] : layers_) {
    removeStaleEdges(layer->edges_);
  }

  for (auto& [layer_id, partitions] : layer_partitions_) {
    for (auto& [partition_id, partition] : partitions) {
      removeStaleEdges(partition->edges_);
    }
  }

  removeStaleEdges(interlayer_edges_);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::clone() const {
  auto to_return = std::make_shared<DynamicSceneGraph>(layer_keys(), layer_names_);
  to_return->metadata = metadata;

  for (const auto [node_id, key] : node_lookup_) {
    auto node = getNodePtr(node_id, key);
    to_return->addOrUpdateNode(
        key.layer, node_id, node->attributes_->clone(), key.partition);
  }

  for (const auto& [layer_id, layer] : layers_) {
    for (const auto& [edge_id, edge] : layer->edges()) {
      to_return->insertEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  for (const auto& [layer_id, partitions] : layer_partitions_) {
    for (const auto& [partition_id, partition] : partitions) {
      for (const auto& [edge_id, edge] : partition->edges()) {
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

void DynamicSceneGraph::transform(const Eigen::Isometry3d& transform) {
  visitLayers([&](LayerKey, Layer& layer) { layer.transform(transform); });
  if (mesh_) {
    mesh_->transform(transform.cast<float>());
  }
}

void DynamicSceneGraph::save(std::filesystem::path filepath, bool include_mesh) const {
  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    io::saveDsgJson(*this, filepath, include_mesh);
    return;
  }

  // Can only be binary after verification.
  io::saveDsgBinary(*this, filepath, include_mesh);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::load(std::filesystem::path filepath) {
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("graph file does not exist: " + filepath.string());
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

Layer& DynamicSceneGraph::layerFromKey(const LayerKey& key) {
  layer_keys_.insert(key);
  if (!key.partition) {
    auto iter = layers_.emplace(key.layer, std::make_unique<Layer>(key.layer)).first;
    return *iter->second;
  }

  auto iter = layer_partitions_.find(key.layer);
  if (iter == layer_partitions_.end()) {
    iter = layer_partitions_.emplace(key.layer, Partitions()).first;
  }

  auto id_layer_pair =
      iter->second.emplace(key.partition, std::make_unique<Layer>(key)).first;
  return *id_layer_pair->second;
}

const Layer& DynamicSceneGraph::layerFromKey(const LayerKey& key) const {
  return const_cast<DynamicSceneGraph*>(this)->layerFromKey(key);
}

SceneGraphNode* DynamicSceneGraph::getNodePtr(NodeId node, const LayerKey& info) const {
  return layerFromKey(info).nodes_.at(node).get();
}

EdgeLayerInfo DynamicSceneGraph::lookupEdge(NodeId source, NodeId target) const {
  EdgeLayerInfo lookup;

  auto source_iter = node_lookup_.find(source);
  if (source_iter == node_lookup_.end()) {
    return lookup;
  }

  auto target_iter = node_lookup_.find(target);
  if (target_iter == node_lookup_.end()) {
    return lookup;
  }

  // lookup is valid: both nodes exist
  lookup.source = source_iter->second;
  lookup.target = target_iter->second;
  lookup.valid = true;

  if (lookup.isSameLayer()) {
    lookup.exists = layerFromKey(source_iter->second).hasEdge(source, target);
  } else {
    lookup.exists = interlayer_edges_.contains(source, target);
  }

  return lookup;
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

  const auto source_key = node_lookup_.at(source);
  const auto lookup = lookupEdge(new_source, target);
  if (lookup.exists) {
    removeInterlayerEdge(source, target, source_key, lookup.target);
    return;
  }

  // removes record of source -> target in nodes and adds new_source -> target instead
  removeAncestry(source, target, source_key, lookup.target);
  addAncestry(new_source, target, lookup.source, lookup.target);

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

  for (auto& [layer_id, partitions] : layer_partitions_) {
    for (auto& [partition_id, partition] : partitions) {
      cb(LayerKey(layer_id, partition_id), *partition);
    }
  }
}

void DynamicSceneGraph::visitLayers(const ConstLayerCallback& cb) const {
  const_cast<DynamicSceneGraph*>(this)->visitLayers(
      [&cb](LayerKey key, Layer& layer) { cb(key, layer); });
}

const Partitions& DynamicSceneGraph::layer_partition(LayerId layer_id) const {
  auto iter = layer_partitions_.find(layer_id);
  if (iter == layer_partitions_.end()) {
    static Partitions empty;  // avoid invalid reference
    return empty;
  }

  return iter->second;
}

LayerKeys DynamicSceneGraph::layer_keys() const {
  return LayerKeys(layer_keys_.begin(), layer_keys_.end());
}

std::vector<LayerId> DynamicSceneGraph::layer_ids() const {
  std::set<LayerId> layers;
  for (const auto& key : layer_keys_) {
    layers.insert(key.layer);
  }
  return std::vector<LayerId>(layers.begin(), layers.end());
}

}  // namespace spark_dsg
