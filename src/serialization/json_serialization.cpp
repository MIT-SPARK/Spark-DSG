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
#include "spark_dsg/serialization/json_serialization.h"

#include <fstream>

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/scene_graph_layer.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using nlohmann::json;

using EdgesPtr = std::unique_ptr<SceneGraphLayer::Edges>;
using NodeSet = std::unordered_set<NodeId>;
using NodeCallback = std::function<void(NodeId, LayerId, NodeAttributes::Ptr&&)>;
using DynamicNodeCallback = std::function<void(
    LayerId, NodeId, std::chrono::nanoseconds, NodeAttributes::Ptr&&)>;
using EdgeCallback = std::function<void(NodeId, NodeId, EdgeAttributes::Ptr&&)>;

namespace io {

void to_json(json& record, const FileHeader& header) {
  record = {{"project_name", header.project_name},
            {"version",
             {{"major", header.version.major},
              {"minor", header.version.minor},
              {"patch", header.version.patch}}}};
}

void from_json(const json& record, FileHeader& header) {
  header.project_name = record.at("project_name").get<std::string>();
  header.version.major = record.at("version").at("major").get<uint8_t>();
  header.version.minor = record.at("version").at("minor").get<uint8_t>();
  header.version.patch = record.at("version").at("patch").get<uint8_t>();
}

}  // namespace io

void to_json(json& record, const NodeAttributes& attributes) {
  JsonConverter converter(&record);
  JsonNodeFactory::get_default().save(converter, attributes);
}

void to_json(json& record, const EdgeAttributes& attributes) {
  JsonConverter converter(&record);
  JsonEdgeFactory::get_default().save(converter, attributes);
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
  record[io::FileHeader::IDENTIFIER_STRING + "_header"] = io::FileHeader::current();

  record["directed"] = false;
  record["multigraph"] = false;
  record["nodes"] = json::array();
  record["edges"] = json::array();
  record["layer_ids"] = layer_ids;

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

  record["mesh"] = json::parse(mesh_->serializeToJson());

  return record.dump();
}

DynamicSceneGraph::Ptr DynamicSceneGraph::deserializeFromJson(
    const std::string& contents) {
  const auto record = json::parse(contents);

  // Parse header.
  const std::string header_field_name = io::FileHeader::IDENTIFIER_STRING + "_header";
  const auto header = record.contains(header_field_name)
                          ? record.at(header_field_name).get<io::FileHeader>()
                          : io::FileHeader::legacy();
  io::GlobalInfo::ScopedInfo info(header);

  const auto layer_ids = record.at("layer_ids").get<LayerIds>();
  auto graph = std::make_shared<DynamicSceneGraph>(layer_ids);

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

  if (!record.contains("mesh")) {
    return graph;
  }

  auto mesh = Mesh::deserializeFromJson(record.at("mesh").dump());
  graph->setMesh(mesh);
  return graph;
}

void to_json(json& j, const BoundingBox& b) {
  j = json{{"type", b.type},
           {"min", b.min},
           {"max", b.max},
           {"world_P_center", b.world_P_center},
           {"world_R_center", Eigen::Quaternionf(b.world_R_center)}};
}

void from_json(const json& j, BoundingBox& b) {
  if (j.at("type").is_null()) {
    b.type = BoundingBox::Type::RAABB;
  } else {
    b.type = j.at("type").get<BoundingBox::Type>();
  }

  if (b.type == BoundingBox::Type::INVALID) {
    return;
  }

  b.min = j.at("min").get<Eigen::Vector3f>();
  b.max = j.at("max").get<Eigen::Vector3f>();
  b.world_P_center = j.at("world_P_center").get<Eigen::Vector3f>();
  auto world_q_center = j.at("world_R_center").get<Eigen::Quaternionf>();
  b.world_R_center = world_q_center.toRotationMatrix();
}

void to_json(json& j, const NearestVertexInfo& info) {
  j = json{
      {"block", info.block}, {"voxel_pos", info.voxel_pos}, {"vertex", info.vertex}};

  if (info.label) {
    j["label"] = info.label.value();
  } else {
    j["label"] = nullptr;
  }
}

void from_json(const json& j, NearestVertexInfo& info) {
  info.block[0] = j.at("block").at(0).get<int32_t>();
  info.block[1] = j.at("block").at(1).get<int32_t>();
  info.block[2] = j.at("block").at(2).get<int32_t>();
  info.voxel_pos[0] = j.at("voxel_pos").at(0).get<double>();
  info.voxel_pos[1] = j.at("voxel_pos").at(1).get<double>();
  info.voxel_pos[2] = j.at("voxel_pos").at(2).get<double>();
  info.vertex = j.at("vertex");

  if (j.contains("label") && !j.at("label").is_null()) {
    info.label = j.at("label").get<uint32_t>();
  }
}

}  // namespace spark_dsg
