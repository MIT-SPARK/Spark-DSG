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
#include "spark_dsg/scene_graph.h"

#include <filesystem>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/mesh.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/serialization/file_io.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;
using Layer = SceneGraphLayer;
using LayerCallback = std::function<void(LayerKey, Layer&)>;
using ConstLayerCallback = std::function<void(LayerKey, const Layer&)>;

using Partitions = SceneGraph::Partitions;
using LayerNames = SceneGraph::LayerNames;
using LayerKeys = SceneGraph::LayerKeys;

namespace {

struct EdgeLayerInfo {
  LayerKey source;
  LayerKey target;
  bool valid = false;
};

EdgeLayerInfo lookupEdge(const std::map<NodeId, LayerKey>& lookup,
                         NodeId source,
                         NodeId target) {
  auto source_iter = lookup.find(source);
  if (source_iter == lookup.end()) {
    return {};
  }

  auto target_iter = lookup.find(target);
  if (target_iter == lookup.end()) {
    return {};
  }

  return {source_iter->second, target_iter->second, true};
}

std::set<LayerKey> layersFromNames(const LayerNames& layer_names,
                                   const LayerKeys& prev_layers = {}) {
  std::set<LayerKey> layers(prev_layers.begin(), prev_layers.end());
  for (const auto& [name, key] : layer_names) {
    layers.insert(key);
  }

  return layers;
}

}  // namespace

SceneGraph::SceneGraph(bool empty)
    : SceneGraph(empty ? LayerKeys{} : LayerKeys{2, 3, 4, 5},
                 empty ? LayerNames{}
                       : LayerNames{{DsgLayers::OBJECTS, 2},
                                    {DsgLayers::AGENTS, 2},
                                    {DsgLayers::PLACES, 3},
                                    {DsgLayers::ROOMS, 4},
                                    {DsgLayers::BUILDINGS, 5}}) {}

SceneGraph::SceneGraph(const LayerKeys& layer_keys, const LayerNames& layer_names)
    : layer_names_(layer_names), layer_keys_(layersFromNames(layer_names, layer_keys)) {
  clear();
}

SceneGraph::Ptr SceneGraph::fromNames(const LayerNames& layers) {
  return std::make_shared<SceneGraph>(LayerKeys{}, layers);
}

void SceneGraph::clear(bool include_mesh) {
  layers_.clear();
  layer_partitions_.clear();

  nodes_.clear();
  edges_.clear();
  node_lookup_.clear();
  node_status_.clear();
  if (include_mesh) {
    mesh_.reset();
  }

  for (const auto& key : layer_keys_) {
    addLayer(key.layer, key.partition);
  }
}

void SceneGraph::reset(const LayerKeys& layer_keys, const LayerNames& layer_names) {
  layer_keys_ = layersFromNames(layer_names, layer_keys);
  layer_names_ = layer_names;
  clear();
}

bool SceneGraph::hasLayer(LayerId layer_id, PartitionId partition) const {
  return findLayer(layer_id, partition) != nullptr;
}

bool SceneGraph::hasLayer(const std::string& layer_name) const {
  auto iter = layer_names_.find(layer_name);
  if (iter == layer_names_.end()) {
    return false;
  }

  return hasLayer(iter->second.layer, iter->second.partition);
}

const Layer* SceneGraph::findLayer(LayerId layer, PartitionId partition) const {
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

const Layer* SceneGraph::findLayer(const std::string& name) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    return nullptr;
  }

  return findLayer(iter->second.layer, iter->second.partition);
}

const Layer& SceneGraph::getLayer(LayerId layer_id, PartitionId partition) const {
  auto layer = findLayer(layer_id, partition);
  if (!layer) {
    const LayerKey key{layer_id, partition};
    throw std::out_of_range("missing layer '" + key.str() + "'");
  }

  return *layer;
}

const Layer& SceneGraph::getLayer(const std::string& name) const {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    throw std::out_of_range("missing layer '" + name + "'");
  }

  return getLayer(iter->second.layer, iter->second.partition);
}

const Layer& SceneGraph::addLayer(LayerId layer_id,
                                  PartitionId partition,
                                  const std::string& name) {
  const LayerKey key{layer_id, partition};
  if (!name.empty()) {
    layer_names_.emplace(name, key);
  }

  return layerFromKey(key);
}

void SceneGraph::removeLayer(LayerId layer_id, PartitionId partition) {
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

bool SceneGraph::emplaceNode(LayerKey key,
                             NodeId node_id,
                             std::unique_ptr<NodeAttributes>&& attrs) {
  if (node_lookup_.count(node_id)) {
    return false;
  }

  const auto& layer = layerFromKey(key);
  node_status_[node_id] = NodeStatus::NEW;
  nodes_.emplace(node_id, std::make_unique<Node>(node_id, layer.id, std::move(attrs)));
  node_lookup_.emplace(node_id, key);
  return true;
}

bool SceneGraph::emplaceNode(LayerId layer_id,
                             NodeId node_id,
                             std::unique_ptr<NodeAttributes>&& attrs,
                             PartitionId partition) {
  return emplaceNode(LayerKey{layer_id, partition}, node_id, std::move(attrs));
}

bool SceneGraph::emplaceNode(const std::string& layer,
                             NodeId node_id,
                             std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = layer_names_.find(layer);
  if (iter == layer_names_.end()) {
    return false;
  }

  return emplaceNode(iter->second, node_id, std::move(attrs));
}

bool SceneGraph::addOrUpdateNode(const std::string& layer,
                                 NodeId node_id,
                                 std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = layer_names_.find(layer);
  if (iter == layer_names_.end()) {
    return false;
  }

  addOrUpdateNode(iter->second.layer, node_id, std::move(attrs));
  return true;
}

void SceneGraph::addOrUpdateNode(LayerId layer_id,
                                 NodeId node_id,
                                 std::unique_ptr<NodeAttributes>&& attrs,
                                 PartitionId partition) {
  addOrUpdateNode(LayerKey{layer_id, partition}, node_id, std::move(attrs));
}

bool SceneGraph::setNodeAttributes(NodeId node_id,
                                   std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter != nodes_.end()) {
    iter->second->attributes_ = std::move(attrs);
    return true;
  }

  return false;
}

bool SceneGraph::insertEdge(NodeId source,
                            NodeId target,
                            std::unique_ptr<EdgeAttributes>&& edge_info,
                            bool enforce_parent_constraints) {
  if (source == target) {
    return false;  // no self edges
  }

  auto source_node = const_cast<SceneGraphNode*>(findNode(source));
  auto target_node = const_cast<SceneGraphNode*>(findNode(target));
  if (!source_node || !target_node) {
    return false;  // no edges to nonexistent nodes
  }

  if (!edges_.insert(source, target, std::move(edge_info))) {
    return false;  // edge already exists
  }

  const auto same_layer = source_node->layer == target_node->layer;
  if (!same_layer && enforce_parent_constraints) {
    dropAllParents(*source_node, *target_node);  // force single parent to exist
  }

  // add information to the nodes about the node relationship
  addConnections(*source_node, *target_node);
  return true;
}

bool SceneGraph::addOrUpdateEdge(NodeId source,
                                 NodeId target,
                                 std::unique_ptr<EdgeAttributes>&& edge_info,
                                 bool enforce_parent_constraints) {
  auto edge = edges_.find(source, target);
  if (!edge) {
    edge->info = std::move(edge_info);
    return true;
  }

  return insertEdge(source, target, std::move(edge_info), enforce_parent_constraints);
}

bool SceneGraph::hasNode(NodeId node_id) const { return node_lookup_.count(node_id); }

NodeStatus SceneGraph::checkNode(NodeId node_id) const {
  auto iter = node_status_.find(node_id);
  return iter == node_status_.end() ? NodeStatus::NONEXISTENT : iter->second;
}

bool SceneGraph::hasEdge(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
}

const Node& SceneGraph::getNode(NodeId node_id) const {
  const auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
  }

  return *node;
}

const Node* SceneGraph::findNode(NodeId node_id) const {
  auto iter = nodes_.find(node_id);
  return iter == nodes_.end() ? nullptr : iter->second.get();
}

const Edge& SceneGraph::getEdge(NodeId source, NodeId target) const {
  const auto edge = findEdge(source, target);
  if (!edge) {
    throw std::out_of_range("Missing edge '" + EdgeKey(source, target).str() + "'");
  }

  return *edge;
}

const Edge* SceneGraph::findEdge(NodeId source, NodeId target) const {
  return edges_.find(source, target);
}

bool SceneGraph::removeNode(NodeId node_id) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return false;
  }

  auto node = iter->second.get();
  const auto to_erase = node->connections();
  for (const auto& target : to_erase) {
    removeEdge(node_id, target);
  }

  // remove the actual node
  nodes_.erase(iter);
  node_status_[node_id] = NodeStatus::DELETED;
  node_lookup_.erase(node_id);
  return true;
}

bool SceneGraph::removeEdge(NodeId source, NodeId target) {
  if (!edges_.remove(source, target)) {
    return false;
  }

  // unlike when adding, nodes are guaranteed to exist
  auto source_node = nodes_.at(source).get();
  auto target_node = nodes_.at(target).get();
  removeConnections(*source_node, *target_node);
  return true;
}

size_t SceneGraph::numLayers() const {
  const size_t static_size = layers_.size();

  size_t unique_layer_groups = 0;
  for (const auto& [layer_id, partitions] : layer_partitions_) {
    if (!layers_.count(layer_id)) {
      ++unique_layer_groups;
    }
  }

  return static_size + unique_layer_groups;
}

size_t SceneGraph::numNodes() const {
  size_t total_nodes = numUnpartitionedNodes();
  for (const auto& [layer_id, partitions] : layer_partitions_) {
    for (const auto& [partition_id, partition] : partitions) {
      total_nodes += partition->numNodes();
    }
  }

  return total_nodes;
}

size_t SceneGraph::numUnpartitionedNodes() const {
  size_t total_nodes = 0u;
  for (const auto& [layer_id, layer] : layers_) {
    total_nodes += layer->numNodes();
  }

  return total_nodes;
}

size_t SceneGraph::numEdges() const { return edges_.size(); }

size_t SceneGraph::numUnpartitionedEdges() const {
  size_t total_edges = 0;
  for (const auto& [edge_key, edge] : edges_.edges) {
    const auto lookup = lookupEdge(node_lookup_, edge_key.k1, edge_key.k2);
    if (!lookup.source.partition && !lookup.target.partition) {
      ++total_edges;
    }
  }

  return total_edges;
}

bool SceneGraph::empty() const { return numNodes() == 0; }

Eigen::Vector3d SceneGraph::getPosition(NodeId node_id) const {
  auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range(NodeSymbol(node_id).str() + " is not in the graph");
  }

  return node->attributes().position;
}

bool SceneGraph::mergeNodes(NodeId from_id, NodeId to_id) {
  if (from_id == to_id) {
    return false;
  }

  auto node_from = findNode(from_id);
  auto node_to = findNode(to_id);
  if (!node_from || !node_to) {
    return false;
  }

  const auto to_rewire = node_from->connections();
  for (const auto& target : to_rewire) {
    rewireEdge(node_from->id, node_to->id, target);
  }

  nodes_.erase(from_id);
  node_lookup_.erase(from_id);
  node_status_[from_id] = NodeStatus::MERGED;
  return true;
}

bool SceneGraph::mergeGraph(const SceneGraph& other,
                            const GraphMergeConfig& config,
                            const Eigen::Isometry3d* transform_new_nodes) {
  metadata.add(other.metadata());
  const auto removed_nodes = other.getRemovedNodes(config.clear_removed);
  for (const auto removed_id : removed_nodes) {
    removeNode(removed_id);
  }

  const auto removed_edges = other.edges_.getRemoved(config.clear_removed);
  for (const auto& removed_edge : removed_edges) {
    removeEdge(removed_edge.k1, removed_edge.k2);
  }

  const auto update_archived = config.update_archived_attributes;
  for (const auto& [other_id, other] : other.nodes_) {
    const bool update_attributes = config.shouldUpdateAttributes(other->layer);
    const auto status = checkNode(other_id);
    if (status == NodeStatus::MERGED) {
      continue;  // don't try to update or add previously merged nodes
    }

    auto iter = nodes_.find(other_id);
    if (iter == nodes_.end()) {
      auto attrs = other->attributes_->clone();
      if (transform_new_nodes) {
        attrs->transform(*transform_new_nodes);
      }

      emplaceNode(other->layer, other_id, std::move(attrs));
      continue;
    }

    bool skip_archive = !update_archived && !iter->second->attributes_->is_active;
    if (!update_attributes || skip_archive) {
      continue;
    }

    iter->second->attributes_ = other->attributes_->clone();
    continue;
  }

  for (const auto& [key, edge] : other.edges_.edges) {
    if (hasEdge(edge.source, edge.target)) {
      // TODO(nathan) clone attributes
      continue;
    }

    const auto source = config.getMergedId(edge.source);
    const auto target = config.getMergedId(edge.target);
    if (source == target) {
      continue;
    }

    insertEdge(source, target, edge.info->clone(), config.enforce_parent_constraints);
  }

  return true;
}

std::vector<NodeId> SceneGraph::getRemovedNodes(bool clear_removed) const {
  std::vector<NodeId> to_return;
  auto iter = node_status_.begin();
  while (iter != node_status_.end()) {
    const auto& [node_id, status] = *iter;
    if (status != NodeStatus::DELETED && status != NodeStatus::MERGED) {
      ++iter;
      continue;
    }

    to_return.push_back(node_id);
    if (clear_removed && status == NodeStatus::DELETED) {
      iter = node_status_.erase(iter);
    } else {
      ++iter;
    }
  }

  return to_return;
}

std::vector<NodeId> SceneGraph::getNewNodes(bool clear_new) const {
  std::vector<NodeId> to_return;
  auto iter = node_status_.begin();
  while (iter != node_status_.end()) {
    if (iter->second == NodeStatus::NEW) {
      to_return.push_back(iter->first);
      if (clear_new) {
        iter->second = NodeStatus::VISIBLE;
      }
    }

    ++iter;
  }

  return to_return;
}

std::vector<EdgeKey> SceneGraph::getRemovedEdges(bool clear_removed) const {
  std::vector<EdgeKey> to_return;
  edges_.getRemoved(to_return, clear_removed);
  return to_return;
}

std::vector<EdgeKey> SceneGraph::getNewEdges(bool clear_new) const {
  std::vector<EdgeKey> to_return;
  edges_.getNew(to_return, clear_new);
  return to_return;
}

bool SceneGraph::edgeToPartition(const SceneGraphEdge& edge) const {
  return nodes_.at(edge.source)->layer.partition ||
         nodes_.at(edge.target)->layer.partition;
}

void SceneGraph::markEdgesAsStale() { edges_.setStale(); }

void SceneGraph::removeAllStaleEdges() {
  const auto stale = edges_.stale_edges;
  for (const auto& [key, edge] : stale) {
    if (edge) {
      removeEdge(key.k1, key.k2);
    }
  }
}

SceneGraph::Ptr SceneGraph::clone() const {
  // TODO(nathan) node filter
  auto to_return = std::make_shared<SceneGraph>(layer_keys(), layer_names_);
  to_return->metadata = metadata;
  for (const auto& [node_id, node] : nodes_) {
    to_return->emplaceNode(node->layer, node_id, node->attributes_->clone());
    to_return->node_status_[node_id] = node_status_[node_id];
  }

  for (const auto& [edge_id, edge] : edges_.edges) {
    to_return->insertEdge(edge.source, edge.target, edge.info->clone());
  }

  if (mesh_) {
    to_return->mesh_ = mesh_->clone();
  }

  return to_return;
}

void SceneGraph::transform(const Eigen::Isometry3d& transform) {
  for (auto&& [id, node] : nodes_) {
    node->attributes().transform(transform);
  }

  if (mesh_) {
    mesh_->transform(transform.cast<float>());
  }
}

void SceneGraph::save(std::filesystem::path filepath, bool include_mesh) const {
  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    io::saveDsgJson(*this, filepath, include_mesh);
    return;
  }

  // Can only be binary after verification.
  io::saveDsgBinary(*this, filepath, include_mesh);
}

SceneGraph::Ptr SceneGraph::load(std::filesystem::path filepath) {
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

void SceneGraph::setMesh(const std::shared_ptr<Mesh>& mesh) { mesh_ = mesh; }

bool SceneGraph::hasMesh() const { return mesh_ != nullptr; }

Mesh::Ptr SceneGraph::mesh() const { return mesh_; }

size_t SceneGraph::memoryUsage() const {
  size_t total_memory = sizeof(*this);

  // Estimate memory usage of state tracking.
  total_memory += layer_keys_.size() * sizeof(LayerKey);
  for (const auto& [name, key] : layer_names_) {
    total_memory += name.size() + sizeof(LayerKey);
  }
  total_memory += node_lookup_.size() * (sizeof(NodeId) + sizeof(LayerKey));
  for (const auto& [layer_id, partitions] : layer_partitions_) {
    total_memory +=
        sizeof(layer_id) + sizeof(partitions) +
        partitions.size() * (sizeof(PartitionId) + sizeof(SceneGraphLayer::Ptr));
  }

  // Estimate memory usage of nodes.
  total_memory += nodes_.size() * (sizeof(NodeId) + sizeof(Node::Ptr));
  for (const auto& [node_id, node] : nodes_) {
    total_memory += node->memoryUsage();
  }

  // Estimate memory usage of nodes status map.
  total_memory += node_status_.size() * (sizeof(NodeId) + sizeof(NodeStatus));

  // Estimate memory usage of interlayer edges.
  total_memory += edges_.memoryUsage();

  // Estimate memory usage of the mesh.
  if (mesh_) {
    total_memory += mesh_->memoryUsage();
  }

  // Add metadata memory usage.
  total_memory += metadata.memoryUsage() - sizeof(metadata);
  return total_memory;
}

Layer& SceneGraph::layerFromKey(const LayerKey& key) {
  layer_keys_.insert(key);
  if (!key.partition) {
    auto iter = layers_.emplace(key.layer, std::make_unique<Layer>(key.layer)).first;
    return *iter->second;
  }

  auto part = layer_partitions_.find(key.layer);
  if (part == layer_partitions_.end()) {
    part = layer_partitions_.emplace(key.layer, Partitions()).first;
  }

  auto& [layer_id, partitions] = *part;
  auto iter = partitions.emplace(key.partition, std::make_unique<Layer>(key)).first;
  return *iter->second;
}

const Layer& SceneGraph::layerFromKey(const LayerKey& key) const {
  return const_cast<SceneGraph*>(this)->layerFromKey(key);
}

void SceneGraph::addOrUpdateNode(LayerKey key,
                                 NodeId node_id,
                                 std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = nodes_.find(node_id);
  if (iter != nodes_.end()) {
    iter->second->attributes_ = std::move(attrs);
  }

  emplaceNode(key, node_id, std::move(attrs));
}

void SceneGraph::addConnections(SceneGraphNode& source, SceneGraphNode& target) {
  if (source.layer.isParentOf(target.layer)) {
    source.children_.insert(target.id);
    target.parents_.insert(source.id);
  } else if (target.layer.isParentOf(source.layer)) {
    target.children_.insert(source.id);
    source.parents_.insert(target.id);
  } else {
    source.siblings_.insert(target.id);
    target.siblings_.insert(source.id);
  }
}

void SceneGraph::removeConnections(SceneGraphNode& source, SceneGraphNode& target) {
  if (source.layer.isParentOf(target.layer)) {
    source.children_.erase(target.id);
    target.parents_.erase(source.id);
  } else if (target.layer.isParentOf(source.layer)) {
    target.children_.erase(source.id);
    source.parents_.erase(target.id);
  } else {
    source.siblings_.erase(target.id);
    target.siblings_.erase(source.id);
  }
}

void SceneGraph::dropAllParents(SceneGraphNode& source, SceneGraphNode& target) {
  const auto source_parent = source.layer.isParentOf(target.layer);
  const auto to_clear = source_parent ? target.parents_ : source.parents_;
  const auto child_to_clear = source_parent ? target.id : source.id;
  for (const auto parent_to_clear : to_clear) {
    removeEdge(child_to_clear, parent_to_clear);
  }
}

bool SceneGraph::rewireEdge(NodeId source,
                            NodeId new_source,
                            NodeId target,
                            std::optional<NodeId> new_target_opt) {
  const auto new_target = new_target_opt.value_or(target);
  auto new_source_node = const_cast<SceneGraphNode*>(findNode(new_source));
  auto new_target_node = const_cast<SceneGraphNode*>(findNode(new_target));
  if (!new_source_node || !new_target_node) {
    return false;  // new endpoints don't exist
  }

  if (source == new_source && target == new_target) {
    return false;  // no work to do, edge already rewired
  }

  if (!edges_.rewire(source, target, new_source, new_target)) {
    return false;  // edge didn't exist, so no work to do
  }

  // removes record of source -> target in nodes and adds new_source -> new_target
  // doesn't change anything if new edge already exists
  removeConnections(*nodes_.at(source), *nodes_.at(target));
  addConnections(*new_source_node, *new_target_node);
  return true;
}

const Partitions& SceneGraph::layer_partition(LayerId layer_id) const {
  auto iter = layer_partitions_.find(layer_id);
  if (iter == layer_partitions_.end()) {
    static Partitions empty;  // avoid invalid reference
    return empty;
  }

  return iter->second;
}

LayerKeys SceneGraph::layer_keys() const {
  return LayerKeys(layer_keys_.begin(), layer_keys_.end());
}

std::vector<LayerId> SceneGraph::layer_ids() const {
  std::set<LayerId> layers;
  for (const auto& key : layer_keys_) {
    layers.insert(key.layer);
  }

  return std::vector<LayerId>(layers.begin(), layers.end());
}

}  // namespace spark_dsg
