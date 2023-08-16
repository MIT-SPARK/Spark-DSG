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
#include "graph_json_serialization.h"

#include <fstream>

#include "serialization_helpers.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/scene_graph_layer.h"

namespace spark_dsg {

template <>
std::unique_ptr<AttributeFactory<NodeAttributes, JsonConverter>>
    AttributeFactory<NodeAttributes, JsonConverter>::s_instance_ = nullptr;

template <>
std::unique_ptr<AttributeFactory<EdgeAttributes, JsonConverter>>
    AttributeFactory<EdgeAttributes, JsonConverter>::s_instance_ = nullptr;

using nlohmann::json;

using EdgesPtr = std::unique_ptr<SceneGraphLayer::Edges>;
using NodeSet = std::unordered_set<NodeId>;
using NodeCallback = std::function<void(NodeId, LayerId, NodeAttributes::Ptr&&)>;
using DynamicNodeCallback = std::function<void(
    LayerId, NodeId, std::chrono::nanoseconds, NodeAttributes::Ptr&&)>;
using EdgeCallback = std::function<void(NodeId, NodeId, EdgeAttributes::Ptr&&)>;

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

void to_json(json& record, const Color& c) {
  record["r"] = c.r;
  record["g"] = c.g;
  record["b"] = c.b;
  record["a"] = c.a;
}

void from_json(const json& record, Color& c) {
  c.r = record.at("r").get<uint8_t>();
  c.g = record.at("g").get<uint8_t>();
  c.b = record.at("b").get<uint8_t>();
  if (record.contains("a")) {
    c.a = record.at("a").get<uint8_t>();
  }
}

void to_json(json& record, const Mesh& mesh) {
  record["points"] = mesh.points;
  record["faces"] = mesh.faces;
  record["colors"] = mesh.colors;
  record["stamps"] = mesh.stamps;
  record["labels"] = mesh.labels;
}

void from_json(const json& record, Mesh& mesh) {
  // TODO(nathan) set mesh bool flags
  if (record.contains("vertices")) {
    for (const auto& vertex : record.at("vertices")) {
      Eigen::Vector3f pos(vertex.at("x").get<float>(),
                          vertex.at("y").get<float>(),
                          vertex.at("z").get<float>());
      mesh.points.push_back(pos);

      Color color{vertex.at("r").get<uint8_t>(),
                  vertex.at("g").get<uint8_t>(),
                  vertex.at("b").get<uint8_t>(),
                  255};
      mesh.colors.push_back(color);
    }

    for (const auto& face : record.at("faces")) {
      mesh.faces.push_back({{face.at(0).get<size_t>(),
                             face.at(1).get<size_t>(),
                             face.at(2).get<size_t>()}});
    }
  } else {
    mesh.points = record.at("points").get<decltype(mesh.points)>();
    mesh.faces = record.at("faces").get<decltype(mesh.faces)>();
    if (record.contains("colors")) {
      mesh.colors = record.at("colors").get<decltype(mesh.colors)>();
    }
  }

  if (record.contains("stamps")) {
    mesh.stamps = record.at("stamps").get<decltype(mesh.stamps)>();
  }

  if (record.contains("labels")) {
    mesh.labels = record.at("labels").get<decltype(mesh.labels)>();
  }
}

void read_node_from_json(const json& record, NodeCallback callback) {
  auto node_id = record.at("id").get<NodeId>();
  auto layer = record.at("layer").get<LayerId>();
  JsonConverter converter(&record.at("attributes"));
  auto attrs = JsonNodeFactory::get_default().create(converter);
  callback(node_id, layer, std::move(attrs));
}

void read_node_from_json(const json& record, DynamicNodeCallback callback) {
  auto layer = record.at("layer").get<LayerId>();
  // auto prefix = record.at("prefix").get<char>();
  auto node_id = record.at("id").get<NodeId>();
  auto timestamp = record.at("timestamp").get<uint64_t>();
  JsonConverter converter(&record.at("attributes"));
  auto attrs = JsonNodeFactory::get_default().create(converter);
  callback(layer, node_id, std::chrono::nanoseconds(timestamp), std::move(attrs));
}

void read_edge_from_json(const json& record, EdgeCallback callback) {
  auto source = record.at("source").get<NodeId>();
  auto target = record.at("target").get<NodeId>();
  JsonConverter converter(&record.at("info"));
  auto attrs = JsonEdgeFactory::get_default().create(converter);
  callback(source, target, std::move(attrs));
}

std::string SceneGraphLayer::serializeLayer(const NodeSet& nodes) const {
  json record;
  record["id"] = id;
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

std::string SceneGraphLayer::toBson() const {
  json record;
  record["id"] = id;
  record["nodes"] = json::array();
  record["edges"] = json::array();

  for (const auto& id_node_pair : nodes()) {
    record["nodes"].push_back(*id_node_pair.second);
  }

  for (const auto& id_edge_pair : edges()) {
    record["edges"].push_back(id_edge_pair.second);
  }

  std::string output;
  json::to_bson(record, output);
  return output;
}

using LayerPtr = std::shared_ptr<IsolatedSceneGraphLayer>;

LayerPtr layerFromJson(const json& record) {
  const auto layer_id = record.at("id").get<LayerId>();
  auto layer = std::make_shared<IsolatedSceneGraphLayer>(layer_id);

  for (const auto& node : record.at("nodes")) {
    read_node_from_json(
        node, [&layer](NodeId id, LayerId layer_id, NodeAttributes::Ptr&& attrs) {
          if (layer->id != layer_id) {
            SG_LOG(ERROR) << "invalid layer found, skipping!" << std::endl;
          }
          layer->emplaceNode(id, std::move(attrs));
        });
  }

  for (const auto& edge : record.at("edges")) {
    read_edge_from_json(
        edge, [&layer](NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs) {
          layer->insertEdge(source, target, std::move(attrs));
        });
  }

  return layer;
}

LayerPtr IsolatedSceneGraphLayer::readFromJson(const std::string& contents) {
  auto record = json::parse(contents);
  return layerFromJson(record);
}

LayerPtr IsolatedSceneGraphLayer::fromBson(const std::string& contents) {
  auto record = json::from_bson(contents);
  return layerFromJson(record);
}

EdgesPtr SceneGraphLayer::deserializeLayer(const std::string& info) {
  reset();

  auto record = json::parse(info);

  for (const auto& node : record.at("nodes")) {
    read_node_from_json(
        node, [this](NodeId id, LayerId layer, NodeAttributes::Ptr&& attrs) {
          if (layer != this->id) {
            SG_LOG(ERROR) << "invalid layer found, skipping!" << std::endl;
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

std::string DynamicSceneGraph::serializeToJson(bool include_mesh) const {
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

  for (const auto& id_edge_pair : dynamic_interlayer_edges()) {
    record["edges"].push_back(id_edge_pair.second);
  }

  for (const auto& id_layer_group_pair : dynamic_layers_) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      const DynamicSceneGraphLayer& layer = *prefix_layer_pair.second;

      for (size_t i = 0; i < layer.nodes_.size(); ++i) {
        if (!layer.node_status_.count(i)) {
          continue;
        }

        if (layer.node_status_.at(i) == NodeStatus::DELETED) {
          continue;
        }

        record["nodes"].push_back(*layer.nodes_.at(i));
      }

      for (const auto& id_edge_pair : layer.edges_.edges) {
        record["edges"].push_back(id_edge_pair.second);
      }
    }
  }

  if (!mesh_ || !include_mesh) {
    return record.dump();
  }

<<<<<<< HEAD
  record["mesh"] = *mesh_;
=======
  record["mesh"]["vertices"] = *mesh_vertices_;
  record["mesh"]["faces"] = *mesh_faces_;
  record["mesh"]["time_stamps"] = mesh_timestamps_;
  if (mesh_labels_) {
    record["mesh"]["labels"] = *mesh_labels_;
  }
>>>>>>> add timestamps to mesh json serialization
  return record.dump();
}

DynamicSceneGraph::Ptr DynamicSceneGraph::deserializeFromJson(
    const std::string& contents) {
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
    read_node_from_json(
        id_content_pair.second,
        [graph](LayerId layer,
                NodeId node,
                std::chrono::nanoseconds time,
                NodeAttributes::Ptr&& attrs) {
          if (!graph->emplacePrevDynamicNode(layer, node, time, std::move(attrs))) {
            std::stringstream ss;
            ss << "failed to add " << NodeSymbol(node).getLabel();
            throw std::runtime_error(ss.str());
          }
        });
  }

  for (const auto& edge : record.at("edges")) {
    read_edge_from_json(
        edge, [graph](NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs) {
          if (!graph->insertEdge(source, target, std::move(attrs))) {
            std::stringstream ss;
            ss << "failed to add " << NodeSymbol(source).getLabel() << " →  "
               << NodeSymbol(target).getLabel();
            throw std::runtime_error(ss.str());
          }
        });
  }

<<<<<<< HEAD
  if (!record.contains("mesh")) {
    return graph;
=======
  if (record.contains("mesh")) {
    MeshVertices::Ptr new_vertices(new MeshVertices());
    *new_vertices = record.at("mesh").at("vertices").get<MeshVertices>();

    auto faces = record.at("mesh").at("faces").get<MeshFaces>();
    auto new_faces = std::make_shared<MeshFaces>(faces.begin(), faces.end());

    auto labels = std::make_shared<std::vector<uint32_t>>();
    if (record.at("mesh").contains("labels")) {
      *labels = record.at("mesh").at("labels").get<std::vector<uint32_t>>();
    }
    auto mesh_timestamps =
        record.at("mesh").at("time_stamps").get<std::vector<uint64_t>>();

    // clear all previous edges
    graph->setMesh(new_vertices, new_faces, labels, mesh_timestamps);
>>>>>>> add timestamps to mesh json serialization
  }

  auto mesh = std::make_shared<Mesh>(record.at("mesh").get<Mesh>());
  graph->setMesh(mesh);
  return graph;
}

std::string Mesh::serializeToJson() const {
  json record = *this;
  return record.dump();
}

Mesh::Ptr Mesh::deserializeFromJson(const std::string& contents) {
  const auto record = json::parse(contents);
  auto mesh = std::make_shared<Mesh>(record.at("mesh").get<Mesh>());
  return mesh;
}

}  // namespace spark_dsg
