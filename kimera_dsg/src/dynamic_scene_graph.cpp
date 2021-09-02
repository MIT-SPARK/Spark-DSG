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

  mesh_vertices_.reset();
  mesh_faces_.reset();

  clearMeshEdges();
}

void DynamicSceneGraph::clearMeshEdges() {
  mesh_edges_.clear();
  mesh_edges_node_lookup_.clear();
  mesh_edges_vertex_lookup_.clear();
}

void DynamicSceneGraph::setMeshDirectly(const pcl::PolygonMesh& mesh) {
  mesh_vertices_.reset(new MeshVertices());
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices_);

  mesh_faces_.reset(new MeshFaces(mesh.polygons.begin(), mesh.polygons.end()));
}

void DynamicSceneGraph::setMesh(const MeshVertices::Ptr& vertices,
                                const std::shared_ptr<MeshFaces>& faces,
                                bool invalidate_all_edges) {
  if (!vertices) {
    VLOG(1) << "received empty mesh. resetting all mesh edges";
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

bool DynamicSceneGraph::hasMeshEdge(NodeId source, size_t mesh_vertex) const {
  if (!mesh_edges_node_lookup_.count(source)) {
    return false;
  }

  if (!mesh_edges_node_lookup_.at(source).count(mesh_vertex)) {
    return false;
  }

  return true;
}

bool DynamicSceneGraph::insertMeshEdge(NodeId source,
                                       size_t mesh_vertex,
                                       bool allow_invalid_mesh) {
  if (!mesh_vertices_) {
    return false;
  }

  if (!hasNode(source)) {
    return false;
  }

  if (mesh_vertex >= mesh_vertices_->size() && !allow_invalid_mesh) {
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

  return hasMesh();
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

std::optional<Eigen::Vector3d> DynamicSceneGraph::getMeshPosition(size_t idx) const {
  if (!mesh_vertices_) {
    return std::nullopt;
  }

  if (idx >= mesh_vertices_->size()) {
    return std::nullopt;
  }

  const pcl::PointXYZRGBA& point = mesh_vertices_->at(idx);
  Eigen::Vector3d pos(point.x, point.y, point.z);
  return pos;
}

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

json DynamicSceneGraph::toJson(const JsonExportConfig& config) const {
  json to_return = SceneGraph::toJson(config);
  to_return["mesh_layer_id"] = mesh_layer_id_;
  if (!mesh_vertices_ || !mesh_faces_) {
    return to_return;
  }

  to_return["mesh"]["vertices"] = *mesh_vertices_;
  to_return["mesh"]["faces"] = *mesh_faces_;

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
    MeshVertices::Ptr new_vertices(new MeshVertices());
    *new_vertices =
        record.at("mesh").at("vertices").get<pcl::PointCloud<pcl::PointXYZRGBA>>();

    auto serialized_vertices =
        record.at("mesh").at("faces").get<std::vector<pcl::Vertices>>();
    std::shared_ptr<MeshFaces> new_faces(
        new MeshFaces(serialized_vertices.begin(), serialized_vertices.end()));

    // clear all previous edges
    setMesh(new_vertices, new_faces, true);

    for (const auto& edge : record.at("mesh_edges")) {
      auto source = edge.at(config.source_key).get<NodeId>();
      auto target = edge.at(config.target_key).get<size_t>();
      insertMeshEdge(source, target);
    }
  }
}

}  // namespace kimera
