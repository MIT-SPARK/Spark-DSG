#include "kimera_dsg/dynamic_scene_graph.h"
#include "kimera_dsg/serialization_helpers.h"

#include <glog/logging.h>
#include <pcl/conversions.h>

#include <list>

namespace kimera {

using ColorPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

DynamicSceneGraph::DynamicSceneGraph(LayerId mesh_layer_id)
    : DynamicSceneGraph(getDefaultLayerIds(), mesh_layer_id) {}

DynamicSceneGraph::DynamicSceneGraph(const LayerIds& factory, LayerId mesh_layer_id)
    : SceneGraph(factory), mesh_layer_id_(mesh_layer_id), next_mesh_edge_idx_(0) {
  if (std::find(factory.begin(), factory.end(), mesh_layer_id) != factory.end()) {
    // TODO(nathan) custom exception
    // TODO(nathan) more informative error message
    throw std::runtime_error("mesh layer id must be unique");
  }
}

void DynamicSceneGraph::clear() {
  SceneGraph::clear();

  mesh_.reset();
  mesh_vertices_.reset();

  clearMeshEdges();
}

void DynamicSceneGraph::clearMeshEdges() {
  mesh_edges_.clear();
  mesh_edges_node_lookup_.clear();
  mesh_edges_vertex_lookup_.clear();
}

void DynamicSceneGraph::setMesh(const Mesh::ConstPtr& mesh, bool invalidate_all_edges) {
  if (!mesh) {
    VLOG(1) << "received empty mesh. resetting all mesh edges";
    mesh_vertices_.reset();
    clearMeshEdges();
    return;
  }

  mesh_ = mesh;
  mesh_vertices_.reset(new ColorPointCloud());
  pcl::fromPCLPointCloud2(mesh_->cloud, *mesh_vertices_);

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

bool DynamicSceneGraph::hasMeshEdge(NodeId source, size_t mesh_vertex) const {
  if (!mesh_edges_node_lookup_.count(source)) {
    return false;
  }

  if (!mesh_edges_node_lookup_.at(source).count(mesh_vertex)) {
    return false;
  }

  return true;
}

bool DynamicSceneGraph::insertMeshEdge(NodeId source, size_t mesh_vertex) {
  if (!mesh_vertices_) {
    return false;
  }

  if (!hasNode(source)) {
    return false;
  }

  if (mesh_vertex >= mesh_vertices_->size()) {
    return false;
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

bool DynamicSceneGraph::hasLayer(LayerId layer_id) const {
  if (layer_id != mesh_layer_id_) {
    return SceneGraph::hasLayer(layer_id);
  }

  return mesh_ != nullptr;
}

bool DynamicSceneGraph::removeNode(NodeId node) {
  if (mesh_edges_node_lookup_.count(node)) {
    std::list<size_t> mesh_edge_targets_to_remove;
    for (const auto& vertex_edge_pair : mesh_edges_node_lookup_.at(node)) {
      mesh_edge_targets_to_remove.push_back(vertex_edge_pair.first);
    }

    for (const auto& vertex : mesh_edge_targets_to_remove) {
      removeMeshEdge(node, vertex);
    }
  }

  return SceneGraph::removeNode(node);
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

size_t DynamicSceneGraph::numLayers() const {
  return layers_.size() + 1;  // for the mesh
}

size_t DynamicSceneGraph::numNodes() const {
  return SceneGraph::numNodes() +
         ((mesh_vertices_ == nullptr) ? 0 : mesh_vertices_->size());
}

size_t DynamicSceneGraph::numEdges() const {
  return SceneGraph::numEdges() + mesh_edges_.size();
}

ColorPointCloud::Ptr DynamicSceneGraph::getMeshCloudForNode(NodeId node) const {
  if (!mesh_ || !mesh_vertices_) {
    return nullptr;
  }

  if (!hasNode(node)) {
    return nullptr;
  }

  ColorPointCloud::Ptr cloud(new ColorPointCloud());
  if (!mesh_edges_node_lookup_.count(node)) {
    return cloud;
  }

  cloud->reserve(mesh_edges_node_lookup_.at(node).size());
  for (const auto& vertex_edge_pair : mesh_edges_node_lookup_.at(node)) {
    cloud->push_back(mesh_vertices_->at(vertex_edge_pair.first));
  }

  return cloud;
}

json DynamicSceneGraph::toJson(const JsonExportConfig& config) const {
  json to_return = SceneGraph::toJson(config);
  to_return["mesh_layer_id"] = mesh_layer_id_;
  if (!mesh_) {
    return to_return;
  }

  json mesh_json = *mesh_;
  to_return["mesh"] = mesh_json;

  to_return["mesh_edges"] = json::array();
  for (const auto& id_edge_pair : mesh_edges_) {
    to_return.at("mesh_edges")
        .push_back(json{{config.source_key, id_edge_pair.second.source_node},
                        {config.target_key, id_edge_pair.second.mesh_vertex}});
  }

  return to_return;
}

void DynamicSceneGraph::fillFromJson(const JsonExportConfig& config,
                                     const NodeAttributeFactory& node_attr_factory,
                                     const EdgeInfoFactory& edge_info_factory,
                                     const json& record) {
  SceneGraph::fillFromJson(config, node_attr_factory, edge_info_factory, record);
  mesh_layer_id_ = record.at("mesh_layer_id").get<LayerId>();

  if (record.contains("mesh")) {
    Mesh::Ptr mesh(new Mesh());
    *mesh = record.at("mesh").get<Mesh>();
    // clear all previous edges
    setMesh(mesh, true);

    for (const auto& edge : record.at("mesh_edges")) {
      auto source = edge.at(config.source_key).get<NodeId>();
      auto target = edge.at(config.target_key).get<size_t>();
      insertMeshEdge(source, target);
    }
  }
}

}  // namespace kimera
