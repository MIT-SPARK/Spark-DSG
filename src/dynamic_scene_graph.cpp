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

#include <pcl/conversions.h>

#include <list>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;
using NodeRef = DynamicSceneGraph::NodeRef;
using DynamicNodeRef = DynamicSceneGraph::DynamicNodeRef;
using EdgeRef = DynamicSceneGraph::EdgeRef;
using MeshVertices = DynamicSceneGraph::MeshVertices;
using MeshFaces = DynamicSceneGraph::MeshFaces;
using MeshEdges = DynamicSceneGraph::MeshEdges;

namespace {

inline NodeId getMergedId(NodeId original,
                          const std::map<NodeId, NodeId>& previous_merges) {
  return previous_merges.count(original) ? previous_merges.at(original) : original;
}

}  // namespace

DynamicSceneGraph::LayerIds getDefaultLayerIds() {
  return {
      DsgLayers::OBJECTS, DsgLayers::PLACES, DsgLayers::ROOMS, DsgLayers::BUILDINGS};
}

DynamicSceneGraph::DynamicSceneGraph(LayerId mesh_layer_id)
    : DynamicSceneGraph(getDefaultLayerIds(), mesh_layer_id) {}

DynamicSceneGraph::DynamicSceneGraph(const LayerIds& layer_ids, LayerId mesh_layer_id)
    : mesh_layer_id(mesh_layer_id), layer_ids(layer_ids), next_mesh_edge_idx_(0) {
  if (layer_ids.empty()) {
    throw std::domain_error("scene graph cannot be initialized without layers");
  }

  if (std::find(layer_ids.begin(), layer_ids.end(), mesh_layer_id) != layer_ids.end()) {
    throw std::domain_error("mesh layer id must be unique");
  }

  clear();
}

void DynamicSceneGraph::clear() {
  layers_.clear();
  dynamic_layers_.clear();

  node_lookup_.clear();

  interlayer_edges_.reset();
  dynamic_interlayer_edges_.reset();

  mesh_vertices_.reset();
  mesh_faces_.reset();

  clearMeshEdges();

  for (const auto& id : layer_ids) {
    layers_[id] = std::make_unique<SceneGraphLayer>(id);
  }
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
                                        NodeAttributes::Ptr&& attrs) {
  if (!layers_.count(layer_id)) {
    SG_LOG(WARNING) << "Invalid layer: " << layer_id << std::endl;
    return false;
  }

  auto iter = node_lookup_.find(node_id);
  if (iter != node_lookup_.end()) {
    getNodePtr(node_id, iter->second)->attributes_ = std::move(attrs);
    return true;
  }

  const bool successful = layers_[layer_id]->emplaceNode(node_id, std::move(attrs));
  if (successful) {
    node_lookup_[node_id] = layer_id;
  }

  return successful;
}

bool DynamicSceneGraph::insertEdge(NodeId source,
                                   NodeId target,
                                   EdgeAttributes::Ptr&& edge_info,
                                   bool force_insert) {
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

  if (force_insert) {
    clearParentAncestry(source, target, source_key, target_key);
  }

  if (!addAncestry(source, target, source_key, target_key)) {
    return false;
  }

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
  if (!hasEdge(source, target)) {
    return false;
  }

  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  if (source_key == target_key) {
    layerFromKey(source_key).edgeContainer().get(source, target).info =
        std::move(attrs);
    return true;
  }

  if (source_key.dynamic || target_key.dynamic) {
    dynamic_interlayer_edges_.get(source, target).info = std::move(attrs);
    return true;
  } else {
    interlayer_edges_.get(source, target).info = std::move(attrs);
    return true;
  }
}

bool DynamicSceneGraph::hasLayer(LayerId layer_id) const {
  if (layer_id != mesh_layer_id) {
    return layers_.count(layer_id) != 0;
  }

  return hasMesh();
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

std::optional<NodeRef> DynamicSceneGraph::getNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  return std::cref(*getNodePtr(node_id, iter->second));
}

std::optional<LayerKey> DynamicSceneGraph::getLayerForNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  return iter->second;
}

std::optional<DynamicNodeRef> DynamicSceneGraph::getDynamicNode(NodeId node_id) const {
  auto iter = node_lookup_.find(node_id);
  if (iter == node_lookup_.end()) {
    return std::nullopt;
  }

  const auto& info = iter->second;
  if (!info.dynamic) {
    return std::nullopt;
  }

  return dynamic_layers_.at(info.layer).at(info.prefix)->getNode(node_id);
}

std::optional<EdgeRef> DynamicSceneGraph::getEdge(NodeId source, NodeId target) const {
  if (!hasEdge(source, target)) {
    return std::nullopt;
  }

  // defer to layers if it is a intralayer edge
  const auto& source_key = node_lookup_.at(source);
  const auto& target_key = node_lookup_.at(target);
  if (source_key == target_key) {
    return layerFromKey(source_key).getEdge(source, target);
  }

  if (source_key.dynamic || target_key.dynamic) {
    return std::cref(dynamic_interlayer_edges_.get(source, target));
  } else {
    return std::cref(interlayer_edges_.get(source, target));
  }
}

bool DynamicSceneGraph::removeNode(NodeId node_id) {
  if (!hasNode(node_id)) {
    return false;
  }

  const auto info = node_lookup_.at(node_id);
  clearMeshEdgesForNode(node_id);

  auto node = getNodePtr(node_id, info);
  if (node->hasParent()) {
    removeInterlayerEdge(node_id, node->parent_);
  }

  std::set<NodeId> targets_to_erase = node->children_;
  for (const auto& target : targets_to_erase) {
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
  const size_t static_size = layers_.size() + 1;  // for the mesh

  size_t unique_dynamic_layers = 0;
  for (const auto& id_layer_group_pair : dynamic_layers_) {
    if (!layers_.count(id_layer_group_pair.first) &&
        id_layer_group_pair.first != mesh_layer_id) {
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
         ((mesh_vertices_ == nullptr || !include_mesh) ? 0 : mesh_vertices_->size());
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

size_t DynamicSceneGraph::numEdges(bool include_mesh) const {
  return numStaticEdges() + numDynamicEdges() + (include_mesh ? mesh_edges_.size() : 0);
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

void DynamicSceneGraph::initMesh() {
  DynamicSceneGraph::MeshVertices fake_vertices;
  pcl::PolygonMesh fake_mesh;
  pcl::toPCLPointCloud2(fake_vertices, fake_mesh.cloud);

  setMeshDirectly(fake_mesh);
}

void DynamicSceneGraph::setMesh(const MeshVertices::Ptr& vertices,
                                const std::shared_ptr<MeshFaces>& faces,
                                bool invalidate_all_edges) {
  if (!vertices) {
    SG_LOG(INFO) << "received empty mesh. resetting all mesh edges" << std::endl;
    mesh_vertices_.reset();
    mesh_faces_.reset();
    clearMeshEdges();
    return;
  }

  mesh_faces_ = faces;
  mesh_vertices_ = vertices;

  if (invalidate_all_edges) {
    clearMeshEdges();
    return;
  }

  size_t max_vertex = mesh_vertices_->size();
  std::list<MeshEdge> invalid_edges;
  for (const auto& vertex_map_pair : mesh_edges_vertex_lookup_) {
    const size_t vertex_id = vertex_map_pair.first;
    if (vertex_id < max_vertex) {
      continue;
    }

    for (const auto& node_edge_pair : mesh_edges_vertex_lookup_.at(vertex_id)) {
      invalid_edges.push_back(mesh_edges_.at(node_edge_pair.second));
    }
  }

  for (const auto edge : invalid_edges) {
    removeMeshEdge(edge.source_node, edge.mesh_vertex);
  }
}

void DynamicSceneGraph::setMeshDirectly(const pcl::PolygonMesh& mesh) {
  mesh_vertices_.reset(new MeshVertices());
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices_);

  mesh_faces_.reset(new MeshFaces(mesh.polygons.begin(), mesh.polygons.end()));
}

bool DynamicSceneGraph::hasMesh() const {
  return mesh_vertices_ != nullptr && mesh_faces_ != nullptr;
}

bool DynamicSceneGraph::isMeshEmpty() const {
  if (!hasMesh()) {
    return true;
  }

  return mesh_vertices_->empty() && mesh_faces_->empty();
}

pcl::PolygonMesh DynamicSceneGraph::getMesh() const {
  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(*mesh_vertices_, mesh.cloud);
  mesh.polygons = *mesh_faces_;
  return mesh;
};

MeshVertices::Ptr DynamicSceneGraph::getMeshVertices() const { return mesh_vertices_; }

std::shared_ptr<MeshFaces> DynamicSceneGraph::getMeshFaces() const {
  return mesh_faces_;
}

std::optional<Eigen::Vector3d> DynamicSceneGraph::getMeshPosition(
    size_t idx, bool check_invalid) const {
  if (!mesh_vertices_) {
    return std::nullopt;
  }

  if (idx >= mesh_vertices_->size()) {
    return std::nullopt;
  }

  const pcl::PointXYZRGBA& point = mesh_vertices_->at(idx);

  // TODO(nathan) this is awkward, but probably fine in the short term
  if (check_invalid && point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
    return std::nullopt;
  }

  Eigen::Vector3d pos(point.x, point.y, point.z);
  return pos;
}

bool DynamicSceneGraph::insertMeshEdge(NodeId source,
                                       size_t mesh_vertex,
                                       bool allow_invalid_mesh) {
  if (!hasNode(source)) {
    return false;
  }

  if (!allow_invalid_mesh) {
    if (!mesh_vertices_ || mesh_vertex >= mesh_vertices_->size()) {
      return false;
    }
  }

  if (hasMeshEdge(source, mesh_vertex)) {
    return false;
  }

  mesh_edges_.emplace(std::piecewise_construct,
                      std::forward_as_tuple(next_mesh_edge_idx_),
                      std::forward_as_tuple(source, mesh_vertex));
  mesh_edges_node_lookup_[source][mesh_vertex] = next_mesh_edge_idx_;
  mesh_edges_vertex_lookup_[mesh_vertex][source] = next_mesh_edge_idx_;
  next_mesh_edge_idx_++;
  return true;
}

bool DynamicSceneGraph::hasMeshEdge(NodeId source, size_t mesh_vertex) const {
  if (!mesh_edges_node_lookup_.count(source)) {
    return false;
  }

  if (!mesh_edges_node_lookup_.at(source).count(mesh_vertex)) {
    return false;
  }

  return true;
}

const MeshEdges& DynamicSceneGraph::getMeshEdges() const { return mesh_edges_; }

std::vector<size_t> DynamicSceneGraph::getMeshConnectionIndices(NodeId node) const {
  std::vector<size_t> to_return;
  if (!mesh_edges_node_lookup_.count(node)) {
    return to_return;
  }

  for (const auto& id_edge_pair : mesh_edges_node_lookup_.at(node)) {
    to_return.push_back(id_edge_pair.first);
  }

  return to_return;
}

bool DynamicSceneGraph::removeMeshEdge(NodeId source, size_t mesh_vertex) {
  if (!hasMeshEdge(source, mesh_vertex)) {
    return false;
  }

  mesh_edges_.erase(mesh_edges_node_lookup_.at(source).at(mesh_vertex));

  mesh_edges_node_lookup_.at(source).erase(mesh_vertex);
  if (mesh_edges_node_lookup_.at(source).empty()) {
    mesh_edges_node_lookup_.erase(source);
  }

  mesh_edges_vertex_lookup_.at(mesh_vertex).erase(source);
  if (mesh_edges_vertex_lookup_.at(mesh_vertex).empty()) {
    mesh_edges_vertex_lookup_.erase(mesh_vertex);
  }

  next_mesh_edge_idx_++;
  return true;
}

void DynamicSceneGraph::invalidateMeshVertex(size_t index) {
  if (!mesh_edges_vertex_lookup_.count(index)) {
    return;
  }

  std::list<NodeId> nodes;
  for (const auto& node_edge_pair : mesh_edges_vertex_lookup_[index]) {
    nodes.push_back(node_edge_pair.first);
  }

  for (const auto& node : nodes) {
    removeMeshEdge(node, index);
  }
}

void DynamicSceneGraph::clearMeshEdges() {
  mesh_edges_.clear();
  mesh_edges_node_lookup_.clear();
  mesh_edges_vertex_lookup_.clear();
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
  if (node->hasParent()) {
    rewireInterlayerEdge(node_from, node_to, node->parent_);
  }

  // Reconnect children
  std::set<NodeId> targets_to_rewire = node->children_;
  for (const auto& target : targets_to_rewire) {
    rewireInterlayerEdge(node_from, node_to, target);
  }

  auto edge_iter = mesh_edges_node_lookup_.find(node_from);
  if (edge_iter != mesh_edges_node_lookup_.end()) {
    for (const auto& id_edge_pair : edge_iter->second) {
      // we always assume that a mesh edge can be invalid (as it was alread added)
      insertMeshEdge(node_to, id_edge_pair.first, true);
    }
  }

  clearMeshEdgesForNode(node_from);

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
                                   const std::map<NodeId, NodeId>& previous_merges,
                                   bool merge_mesh_edges,
                                   bool allow_invalid_mesh,
                                   bool clear_mesh_edges,
                                   std::map<LayerId, bool>* update_map,
                                   bool update_dynamic,
                                   bool clear_removed) {
  for (const auto& id_layers : other.dynamicLayers()) {
    const LayerId layer = id_layers.first;

    for (const auto& prefix_layer : id_layers.second) {
      const auto prefix = prefix_layer.first;
      if (!hasLayer(layer, prefix)) {
        createDynamicLayer(layer, prefix);
      }

      dynamic_layers_[layer][prefix]->mergeLayer(
          *prefix_layer.second, &node_lookup_, update_dynamic);
    }
  }

  for (const auto& id_layer : other.layers()) {
    const LayerId layer = id_layer.first;
    if (!hasLayer(layer)) {
      continue;
    }

    std::vector<NodeId> removed_nodes;
    id_layer.second->getRemovedNodes(removed_nodes, clear_removed);
    for (const auto& removed_id : removed_nodes) {
      removeNode(removed_id);
    }

    std::vector<EdgeKey> removed_edges;
    id_layer.second->edges_.getRemoved(removed_edges, clear_removed);
    for (const auto& removed_edge : removed_edges) {
      layers_[layer]->removeEdge(removed_edge.k1, removed_edge.k2);
    }

    const bool update =
        (update_map && update_map->count(layer)) ? update_map->at(layer) : true;
    layers_[layer]->mergeLayer(
        *id_layer.second, previous_merges, &node_lookup_, update);
  }

  for (const auto& id_edge_pair : other.interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    NodeId new_source = getMergedId(edge.source, previous_merges);
    NodeId new_target = getMergedId(edge.target, previous_merges);
    if (new_source == new_target) {
      continue;
    }

    insertEdge(new_source, new_target, edge.info->clone());
  }

  for (const auto& id_edge_pair : other.dynamic_interlayer_edges()) {
    const auto& edge = id_edge_pair.second;
    NodeId new_source = getMergedId(edge.source, previous_merges);
    NodeId new_target = getMergedId(edge.target, previous_merges);
    if (new_source == new_target) {
      continue;
    }

    insertEdge(new_source, new_target, edge.info->clone());
  }

  if (!merge_mesh_edges) {
    return true;
  }

  if (clear_mesh_edges) {
    clearMeshEdges();
  }

  // TODO(nathan) this doesn't handle merges
  for (const auto& id_mesh_edge : other.mesh_edges_) {
    insertMeshEdge(id_mesh_edge.second.source_node,
                   id_mesh_edge.second.mesh_vertex,
                   allow_invalid_mesh);
  }

  // TODO(Yun) check the other mesh info (faces, vertices etc. )
  return true;
}

bool DynamicSceneGraph::mergeGraph(const DynamicSceneGraph& other,
                                   bool merge_mesh_edges,
                                   bool allow_invalid_mesh,
                                   bool clear_mesh_edges,
                                   std::map<LayerId, bool>* attribute_update_map,
                                   bool update_dynamic_attributes,
                                   bool clear_removed) {
  return mergeGraph(other,
                    {},
                    merge_mesh_edges,
                    allow_invalid_mesh,
                    clear_mesh_edges,
                    attribute_update_map,
                    update_dynamic_attributes,
                    clear_removed);
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
  auto to_return = std::make_shared<DynamicSceneGraph>(layer_ids, mesh_layer_id);
  for (const auto id_layer_pair : node_lookup_) {
    auto node = getNodePtr(id_layer_pair.first, id_layer_pair.second);
    if (id_layer_pair.second.dynamic) {
      auto node_dyn = dynamic_cast<DynamicSceneGraphNode*>(node);
      if (!node_dyn) {
        // TODO(nathan) fix exception messages
        std::stringstream ss;
        ss << "node " << NodeSymbol(node->id).getLabel() << " is not dynamic!";
        throw std::runtime_error(ss.str());
      }

      to_return->emplacePrevDynamicNode(node_dyn->layer,
                                        node_dyn->id,
                                        node_dyn->timestamp,
                                        node_dyn->attributes_->clone());
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

  if (mesh_vertices_) {
    to_return->mesh_vertices_.reset(new MeshVertices(*mesh_vertices_));
  }
  if (mesh_faces_) {
    to_return->mesh_faces_ = std::make_shared<MeshFaces>(*mesh_faces_);
  }

  for (const auto& id_edge_pair : mesh_edges_) {
    const auto& edge = id_edge_pair.second;
    to_return->insertMeshEdge(edge.source_node, edge.mesh_vertex, true);
  }

  return to_return;
}

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

bool DynamicSceneGraph::addAncestry(NodeId source,
                                    NodeId target,
                                    const LayerKey& source_key,
                                    const LayerKey& target_key) {
  SceneGraphNode* source_node = getNodePtr(source, source_key);
  SceneGraphNode* target_node = getNodePtr(target, target_key);
  if (source_key.isParent(target_key)) {
    if (target_node->hasParent()) {
      return false;
    }
    source_node->children_.insert(target);
    target_node->setParent(source);
  } else if (target_key.isParent(source_key)) {
    if (source_node->hasParent()) {
      return false;
    }
    target_node->children_.insert(source);
    source_node->setParent(target);
  } else {
    source_node->siblings_.insert(target);
    target_node->siblings_.insert(source);
  }

  return true;
}

void DynamicSceneGraph::removeAncestry(NodeId source,
                                       NodeId target,
                                       const LayerKey& source_key,
                                       const LayerKey& target_key) {
  SceneGraphNode* source_node = getNodePtr(source, source_key);
  SceneGraphNode* target_node = getNodePtr(target, target_key);

  if (source_key.isParent(target_key)) {
    source_node->children_.erase(target);
    target_node->clearParent();
  } else if (target_key.isParent(source_key)) {
    target_node->children_.erase(source);
    source_node->clearParent();
  } else {
    source_node->siblings_.erase(target);
    target_node->siblings_.erase(source);
  }
}

void DynamicSceneGraph::clearParentAncestry(NodeId source,
                                            NodeId target,
                                            const LayerKey& source_key,
                                            const LayerKey& target_key) {
  SceneGraphNode* source_node = getNodePtr(source, source_key);
  SceneGraphNode* target_node = getNodePtr(target, target_key);
  const auto source_parent = source_node->getParent();
  const auto target_parent = target_node->getParent();

  if (source_key.isParent(target_key) && target_parent) {
    removeInterlayerEdge(
        target, *target_parent, target_key, node_lookup_.at(*target_parent));
    return;
  }

  if (target_key.isParent(source_key) && source_parent) {
    removeInterlayerEdge(
        source, *source_parent, source_key, node_lookup_.at(*source_parent));
    return;
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

  removeAncestry(source, target, source_key, target_key);
  bool new_source_has_parent =
      !addAncestry(new_source, target, new_source_key, target_key);

  // TODO(nathan) edges can technically jump from dynamic to static, so problems with
  // index not being available in other container
  EdgeAttributes::Ptr attrs;
  if (source_key.dynamic || target_key.dynamic) {
    attrs = dynamic_interlayer_edges_.get(source, target).info->clone();
    dynamic_interlayer_edges_.remove(source, target);
  } else {
    attrs = interlayer_edges_.get(source, target).info->clone();
    interlayer_edges_.remove(source, target);
  }

  if (new_source_has_parent) {
    // we silently drop edges when the new source node also has a parent
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

void DynamicSceneGraph::clearMeshEdgesForNode(NodeId node_id) {
  if (mesh_edges_node_lookup_.count(node_id)) {
    std::list<size_t> mesh_edge_targets_to_remove;
    for (const auto& vertex_edge_pair : mesh_edges_node_lookup_.at(node_id)) {
      mesh_edge_targets_to_remove.push_back(vertex_edge_pair.first);
    }

    for (const auto& vertex : mesh_edge_targets_to_remove) {
      removeMeshEdge(node_id, vertex);
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
