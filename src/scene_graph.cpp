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

#include "spark_dsg/mesh.h"
#include "spark_dsg/printing.h"
#include "spark_dsg/serialization/file_io.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;
using Layer = SceneGraphLayer;
using LayerCallback = std::function<void(LayerKey, Layer&)>;
using ConstLayerCallback = std::function<void(LayerKey, const Layer&)>;
using UniqueGraph = std::unique_ptr<SceneGraph>;

using Partitions = SceneGraph::Partitions;
using LayerNames = SceneGraph::LayerNames;
using LayerKeys = SceneGraph::LayerKeys;

std::set<LayerKey> layersFromNames(const LayerNames& layer_names,
                                   const LayerKeys& prev_layers = {}) {
  std::set<LayerKey> layers(prev_layers.begin(), prev_layers.end());
  for (const auto& [name, key] : layer_names) {
    layers.insert(key);
  }

  return layers;
}

SceneGraph::SceneGraph(bool empty)
    : SceneGraph(empty ? LayerKeys{} : LayerKeys{2, 3, 4, 5},
                 empty ? LayerNames{}
                       : LayerNames{{DsgLayers::OBJECTS, 2},
                                    {DsgLayers::AGENTS, 2},
                                    {DsgLayers::PLACES, 3},
                                    {DsgLayers::ROOMS, 4},
                                    {DsgLayers::BUILDINGS, 5}}) {}

SceneGraph::SceneGraph(const LayerKeys& layer_keys, const LayerNames& layer_names)
    : layer_keys_(layersFromNames(layer_names, layer_keys)),
      layer_names_(layer_names),
      impl_(std::make_shared<GraphImpl>()) {
  clear();
}

SceneGraph::Ptr SceneGraph::fromNames(const LayerNames& layers) {
  return std::make_shared<SceneGraph>(LayerKeys{}, layers);
}

void SceneGraph::clear(bool include_mesh) {
  layers_.clear();
  layer_partitions_.clear();

  impl_->clear();
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
    std::stringstream ss;
    ss << "missing layer " << LayerKey{layer_id, partition};
    throw std::out_of_range(ss.str());
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
  for (const auto& node : layer->nodes()) {
    to_remove.push_back(node.id);
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
  return impl_->emplace(key, node_id, std::move(attrs));
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

bool SceneGraph::addOrUpdateNode(const std::string& name,
                                 NodeId node_id,
                                 std::unique_ptr<NodeAttributes>&& attrs) {
  auto iter = layer_names_.find(name);
  if (iter == layer_names_.end()) {
    return false;
  }

  const auto& [layer, partition] = iter->second;
  return addOrUpdateNode(layer, node_id, std::move(attrs), partition);
}

bool SceneGraph::addOrUpdateNode(LayerId layer_id,
                                 NodeId node_id,
                                 std::unique_ptr<NodeAttributes>&& attrs,
                                 PartitionId partition) {
  const LayerKey key{layer_id, partition};
  return impl_->update(key, node_id, std::move(attrs));
}

bool SceneGraph::setNodeAttributes(NodeId node_id,
                                   std::unique_ptr<NodeAttributes>&& attrs) {
  return impl_->set(node_id, std::move(attrs));
}

bool SceneGraph::insertEdge(NodeId source,
                            NodeId target,
                            std::unique_ptr<EdgeAttributes>&& attrs,
                            bool enforce_parent_constraints) {
  return impl_->connect(source, target, std::move(attrs), enforce_parent_constraints);
}

bool SceneGraph::addOrUpdateEdge(NodeId source,
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

bool SceneGraph::hasNode(NodeId node_id) const { return impl_->has(node_id); }

NodeStatus SceneGraph::checkNode(NodeId node) const { return impl_->status(node); }

bool SceneGraph::hasEdge(NodeId source, NodeId target) const {
  return impl_->has(source, target);
}

const Node& SceneGraph::getNode(NodeId node_id) const { return impl_->get(node_id); }

const Node* SceneGraph::findNode(NodeId node_id) const { return impl_->find(node_id); }

const Edge& SceneGraph::getEdge(NodeId source, NodeId target) const {
  return impl_->get(source, target);
}

const Edge* SceneGraph::findEdge(NodeId source, NodeId target) const {
  return impl_->find(source, target);
}

bool SceneGraph::removeNode(NodeId node) { return impl_->remove(node); }

bool SceneGraph::removeEdge(NodeId source, NodeId target) {
  return impl_->remove(source, target);
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

size_t SceneGraph::numNodes() const { return impl_->num_nodes(); }

size_t SceneGraph::numEdges() const { return impl_->num_edges(); }

bool SceneGraph::empty() const { return numNodes() == 0; }

bool SceneGraph::mergeNodes(NodeId from_id, NodeId to_id) {
  return impl_->contract(from_id, to_id);
}

bool SceneGraph::mergeGraph(const SceneGraph& other,
                            const GraphMergeConfig& config,
                            const Eigen::Isometry3d* transform_new_nodes) {
  metadata.add(other.metadata());
  impl_->merge(*other.impl_, config, transform_new_nodes);
  return true;
}

std::vector<NodeId> SceneGraph::getRemovedNodes(bool clear) {
  return impl_->removed_nodes(clear);
}

std::vector<NodeId> SceneGraph::getNewNodes(bool clear) {
  return impl_->new_nodes(clear);
}

std::vector<EdgeKey> SceneGraph::getRemovedEdges(bool clear) {
  return impl_->removed_edges(clear);
}

std::vector<EdgeKey> SceneGraph::getNewEdges(bool clear) {
  return impl_->new_edges(clear);
}

SceneGraph::Ptr SceneGraph::clone() const { return clone_unique(); }

UniqueGraph SceneGraph::clone_unique() const {
  auto to_return = empty_like();
  to_return->impl_ = impl_->clone();
  if (mesh_) {
    to_return->mesh_ = mesh_->clone();
  }

  return to_return;
}

UniqueGraph SceneGraph::empty_like() const {
  auto to_return = std::make_unique<SceneGraph>(layer_keys(), layer_names_);
  to_return->metadata = metadata;
  return to_return;
}

void SceneGraph::transform(const Eigen::Isometry3d& transform) {
  impl_->transform(transform);
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
  return io::loadDsgFromFile(filepath);
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

  for (const auto& [layer_id, partitions] : layer_partitions_) {
    total_memory +=
        sizeof(layer_id) + sizeof(partitions) +
        partitions.size() * (sizeof(PartitionId) + sizeof(SceneGraphLayer::Ptr));
  }

  total_memory += impl_->memory_usage();
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

  auto iter = layer_partitions_.find(key.layer);
  if (iter == layer_partitions_.end()) {
    iter = layer_partitions_.emplace(key.layer, Partitions()).first;
  }

  auto id_layer_pair =
      iter->second.emplace(key.partition, std::make_unique<Layer>(key)).first;
  return *id_layer_pair->second;
}

const Layer& SceneGraph::layerFromKey(const LayerKey& key) const {
  return const_cast<SceneGraph*>(this)->layerFromKey(key);
}

const Partitions& SceneGraph::layer_partition(LayerId layer_id) const {
  auto iter = layer_partitions_.find(layer_id);
  if (iter == layer_partitions_.end()) {
    static Partitions empty;  // avoid invalid reference
    return empty;
  }

  return iter->second;
}

std::optional<LayerKey> SceneGraph::getLayerKey(const std::string& name) const {
  auto iter = layer_names_.find(name);
  return iter == layer_names_.end() ? std::nullopt
                                    : std::optional<LayerKey>(iter->second);
}

LayerKeys SceneGraph::layer_keys() const {
  return LayerKeys(layer_keys_.begin(), layer_keys_.end());
}

const LayerNames SceneGraph::layer_names() const { return layer_names_; }

auto SceneGraph::layers() const -> const Layers& { return layers_; };

const std::map<LayerId, Partitions>& SceneGraph::layer_partitions() const {
  return layer_partitions_;
}

UniqueGraph SceneGraph::create_subgraph(const std::vector<NodeId>& nodes) const {
  auto graph = empty_like();
  for (const auto& node_id : nodes) {
    const auto node = findNode(node_id);
    if (!node) {
      continue;
    }

    const auto key = node->layer;
    graph->emplaceNode(key.layer, node_id, node->attributes().clone(), key.partition);
    for (const auto neighbor : node->connections()) {
      const auto& edge = getEdge(node_id, neighbor);
      graph->insertEdge(node_id, neighbor, edge.attributes().clone());
    }
  }

  return graph;
}

}  // namespace spark_dsg
