#include "kimera_dsg/attribute_serialization.h"
#include "kimera_dsg/dynamic_scene_graph.h"
#include "kimera_dsg/scene_graph_layer.h"
#include "kimera_dsg/serialization_helpers.h"

#include <glog/logging.h>

#include <fstream>

namespace kimera {

using EdgesPtr = std::unique_ptr<SceneGraphLayer::Edges>;
using NodeSet = std::unordered_set<NodeId>;
using NodeCallback = std::function<void(NodeId, LayerId, NodeAttributes::Ptr&&)>;
using DynamicNodeCallback =
    std::function<void(LayerId, char, std::chrono::nanoseconds, NodeAttributes::Ptr&&)>;
using EdgeCallback = std::function<void(NodeId, NodeId, EdgeAttributes::Ptr&&)>;

void to_json(json& record, const MeshEdge& edge) {
  record = {{"source", edge.source_node}, {"target", edge.mesh_vertex}};
}

void from_json(const json& record, MeshEdge& edge) {
  edge.source_node = record.at("source").get<NodeId>();
  edge.mesh_vertex = record.at("target").get<size_t>();
}

template <typename T>
void to_json(json& record, const T& attributes) {
  AttributeSerializer serializer;
  attributes.serialize(serializer);
  record = serializer.record;
}

void to_json(json& record, const SceneGraphNode& node) {
  record = {{"id", node.id}, {"layer", node.layer}, {"attributes", node.attributes()}};
}

void to_json(json& record, const DynamicSceneGraphNode& node) {
  record = {{"id", node.id},
            {"layer", node.layer},
            {"prefix", NodeSymbol(node.id).category()},
            {"timestamp", node.timestamp.count()},
            {"attributes", node.attributes()}};
}

void to_json(json& record, const SceneGraphEdge& edge) {
  record = {{"source", edge.source}, {"target", edge.target}, {"info", *edge.info}};
}

void read_node_from_json(const json& record, NodeCallback callback) {
  auto node_id = record.at("id").get<NodeId>();
  auto layer = record.at("layer").get<LayerId>();
  auto attributes = NodeAttributeFactory::get_default().create(record.at("attributes"));
  callback(node_id, layer, std::move(attributes));
}

void read_node_from_json(const json& record, DynamicNodeCallback callback) {
  auto layer = record.at("layer").get<LayerId>();
  auto prefix = record.at("prefix").get<char>();
  auto timestamp = record.at("timestamp").get<uint64_t>();
  auto attributes = NodeAttributeFactory::get_default().create(record.at("attributes"));
  callback(layer, prefix, std::chrono::nanoseconds(timestamp), std::move(attributes));
}

void read_edge_from_json(const json& record, EdgeCallback callback) {
  auto source = record.at("source").get<NodeId>();
  auto target = record.at("target").get<NodeId>();
  auto info = EdgeAttributeFactory::get_default().create(record.at("info"));
  callback(source, target, std::move(info));
}

std::string SceneGraphLayer::serializeLayer(const NodeSet& nodes) const {
  json record;
  record["nodes"] = json::array();
  record["edges"] = json::array();

  for (const auto& node_id : nodes) {
    if (!hasNode(node_id)) {
      continue;
    }

    const auto& node = *nodes_.at(node_id);
    record["nodes"].push_back(node);

    if (node.siblings().empty()) {
      continue;
    }

    for (const auto& sibling : node.siblings()) {
      record["edges"].push_back(edges_.get(node_id, sibling));
    }
  }

  return record.dump();
}

EdgesPtr SceneGraphLayer::deserializeLayer(const std::string& info) {
  reset();

  auto record = json::parse(info);

  for (const auto& node : record.at("nodes")) {
    read_node_from_json(node,
                        [this](NodeId id, LayerId layer, NodeAttributes::Ptr&& attrs) {
                          if (layer != this->id) {
                            LOG(ERROR) << "invalid layer found, skipping!";
                          }
                          this->emplaceNode(id, std::move(attrs));
                        });
  }

  auto new_edges = std::make_unique<Edges>();
  for (const auto& edge : record.at("edges")) {
    read_edge_from_json(
        edge, [&](NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs) {
          new_edges->emplace(std::piecewise_construct,
                             std::forward_as_tuple(source, target),
                             std::forward_as_tuple(source, target, std::move(attrs)));
        });
  }

  return new_edges;
}

void DynamicSceneGraph::save(const std::string& filepath, bool include_mesh) const {
  std::ofstream outfile(filepath);
  outfile << this->serialize(include_mesh);
}

std::string DynamicSceneGraph::serialize(bool include_mesh) const {
  json record;
  record["directed"] = false;
  record["multigraph"] = false;
  record["nodes"] = json::array();
  record["edges"] = json::array();
  record["layer_ids"] = layer_ids;
  record["mesh_layer_id"] = mesh_layer_id;

  for (const auto& id_layer_pair : layers_) {
    for (const auto& id_node_pair : id_layer_pair.second->nodes_) {
      record["nodes"].push_back(*id_node_pair.second);
    }

    for (const auto& id_edge_pair : id_layer_pair.second->edges()) {
      record["edges"].push_back(id_edge_pair.second);
    }
  }

  for (const auto& id_edge_pair : interlayer_edges()) {
    record["edges"].push_back(id_edge_pair.second);
  }

  for (const auto& id_layer_group_pair : dynamic_layers_) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      const DynamicSceneGraphLayer& layer = *prefix_layer_pair.second;

      for (const auto& node : layer.nodes_) {
        record["nodes"].push_back(*node);
      }

      for (const auto& id_edge_pair : layer.edges_.edges) {
        record["edges"].push_back(id_edge_pair.second);
      }
    }
  }

  record["mesh_edges"] = json::array();
  for (const auto& id_edge_pair : mesh_edges_) {
    record.at("mesh_edges").push_back(id_edge_pair.second);
  }

  if (!mesh_vertices_ || !mesh_faces_ || !include_mesh) {
    return record.dump();
  }

  record["mesh"]["vertices"] = *mesh_vertices_;
  record["mesh"]["faces"] = *mesh_faces_;
  return record.dump();
}

DynamicSceneGraph::Ptr DynamicSceneGraph::load(const std::string& filepath) {
  std::stringstream ss;

  std::ifstream infile(filepath);
  ss << infile.rdbuf();

  return deserialize(ss.str());
}

DynamicSceneGraph::Ptr DynamicSceneGraph::deserialize(const std::string& contents) {
  const auto record = json::parse(contents);
  const auto mesh_layer_id = record.at("mesh_layer_id").get<LayerId>();
  const auto layer_ids = record.at("layer_ids").get<LayerIds>();

  auto graph = std::make_shared<DynamicSceneGraph>(layer_ids, mesh_layer_id);

  std::map<NodeId, json> dynamic_contents;
  for (const auto& node : record.at("nodes")) {
    if (node.contains("timestamp")) {
      dynamic_contents[node.at("id").get<NodeId>()] = node;
    } else {
      read_node_from_json(
          node, [graph](NodeId id, LayerId layer, NodeAttributes::Ptr&& attrs) {
            graph->emplaceNode(layer, id, std::move(attrs));
          });
    }
  }

  for (const auto& id_content_pair : dynamic_contents) {
    read_node_from_json(id_content_pair.second,
                        [graph](LayerId layer,
                                char prefix,
                                std::chrono::nanoseconds time,
                                NodeAttributes::Ptr&& attrs) {
                          graph->emplaceNode(
                              layer, prefix, time, std::move(attrs), false);
                        });
  }

  for (const auto& edge : record.at("edges")) {
    read_edge_from_json(
        edge, [graph](NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs) {
          graph->insertEdge(source, target, std::move(attrs));
        });
  }

  if (record.contains("mesh")) {
    MeshVertices::Ptr new_vertices(new MeshVertices());
    *new_vertices = record.at("mesh").at("vertices").get<MeshVertices>();

    auto faces = record.at("mesh").at("faces").get<MeshFaces>();
    auto new_faces = std::make_shared<MeshFaces>(faces.begin(), faces.end());

    // clear all previous edges
    graph->setMesh(new_vertices, new_faces, true);
  }

  if (record.contains("mesh_edges")) {
    size_t num_inserted = 0;
    for (const auto& edge : record.at("mesh_edges")) {
      auto source = edge.at("source").get<NodeId>();
      auto target = edge.at("target").get<size_t>();
      num_inserted += graph->insertMeshEdge(source, target, true) ? 1 : 0;
    }

    VLOG(1) << "Loaded " << num_inserted << " mesh edges";
  }

  return graph;
}

}  // namespace kimera
