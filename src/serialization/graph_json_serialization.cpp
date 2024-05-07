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
#include "spark_dsg/serialization/graph_json_serialization.h"

#include <fstream>

#include "spark_dsg/logging.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using nlohmann::json;

void to_json(json& record, const SceneGraphNode& node) {
  record = {{"id", node.id},
            {"layer", node.layer},
            {"attributes", node.attributes()}};
  if (node.timestamp) {
    record["timestamp"] = node.timestamp->count();
  }
}

void to_json(json& record, const SceneGraphEdge& edge) {
  record = {
      {"source", edge.source}, {"target", edge.target}, {"info", edge.attributes()}};
}

void read_node_from_json(const serialization::AttributeFactory<NodeAttributes>& factory,
                         const json& record,
                         DynamicSceneGraph& graph) {
  auto node_id = record.at("id").get<NodeId>();
  auto layer = record.at("layer").get<LayerId>();
  auto attrs = serialization::Visitor::from(factory, record.at("attributes"));
  if (!attrs) {
    std::stringstream ss;
    ss << "invalid attributes for " << NodeSymbol(node_id).getLabel();
    throw std::runtime_error(ss.str());
  }

  bool added = false;
  if (record.contains("timestamp")) {
    auto time = record.at("timestamp").get<uint64_t>();
    added = graph.emplacePrevDynamicNode(
        layer, node_id, std::chrono::nanoseconds(time), std::move(attrs));
  } else {
    added = graph.emplaceNode(layer, node_id, std::move(attrs));
  }

  if (!added) {
    std::stringstream ss;
    ss << "failed to add " << NodeSymbol(node_id).getLabel();
    throw std::runtime_error(ss.str());
  }
}

void read_edge_from_json(const serialization::AttributeFactory<EdgeAttributes>& factory,
                         const json& record,
                         DynamicSceneGraph& graph) {
  auto source = record.at("source").get<NodeId>();
  auto target = record.at("target").get<NodeId>();
  auto attrs = serialization::Visitor::from(factory, record.at("info"));

  if (!graph.insertEdge(source, target, std::move(attrs))) {
    std::stringstream ss;
    ss << "failed to add " << NodeSymbol(source).getLabel() << " →  "
       << NodeSymbol(target).getLabel();
    throw std::runtime_error(ss.str());
  }
}

namespace io::json {

std::string writeGraph(const DynamicSceneGraph& graph, bool include_mesh) {
  nlohmann::json record;
  record[io::FileHeader::IDENTIFIER_STRING + "_header"] = io::FileHeader::current();

  record["directed"] = false;
  record["multigraph"] = false;
  record["nodes"] = nlohmann::json::array();
  record["edges"] = nlohmann::json::array();
  record["layer_ids"] = graph.layer_ids;

  for (const auto& id_layer_pair : graph.layers()) {
    for (const auto& id_node_pair : id_layer_pair.second->nodes()) {
      record["nodes"].push_back(*id_node_pair.second);
    }

    for (const auto& id_edge_pair : id_layer_pair.second->edges()) {
      record["edges"].push_back(id_edge_pair.second);
    }
  }

  for (const auto& id_edge_pair : graph.interlayer_edges()) {
    record["edges"].push_back(id_edge_pair.second);
  }

  for (const auto& id_edge_pair : graph.dynamic_interlayer_edges()) {
    record["edges"].push_back(id_edge_pair.second);
  }

  for (const auto& id_layer_group_pair : graph.dynamicLayers()) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      const auto& layer = *prefix_layer_pair.second;

      for (size_t i = 0; i < layer.nodes().size(); ++i) {
        if (!layer.hasNodeByIndex(i)) {
          continue;
        }

        record["nodes"].push_back(layer.getNodeByIndex(i));
      }

      for (const auto& id_edge_pair : layer.edges()) {
        record["edges"].push_back(id_edge_pair.second);
      }
    }
  }

  auto mesh = graph.mesh();
  if (!mesh || !include_mesh) {
    return record.dump();
  }

  // TODO(nathan) push header serialization to to/from json and reuse
  record["mesh"] = nlohmann::json::parse(mesh->serializeToJson());
  return record.dump();
}

DynamicSceneGraph::Ptr readGraph(const std::string& contents) {
  const auto record = nlohmann::json::parse(contents);

  // Parse header.
  const std::string header_field_name = FileHeader::IDENTIFIER_STRING + "_header";
  const auto header = record.contains(header_field_name)
                          ? record.at(header_field_name).get<io::FileHeader>()
                          : io::FileHeader::legacy();
  io::GlobalInfo::ScopedInfo info(header);
  const auto node_factory = serialization::AttributeRegistry<NodeAttributes>::current();
  const auto edge_factory = serialization::AttributeRegistry<EdgeAttributes>::current();

  const auto layer_ids = record.at("layer_ids").get<DynamicSceneGraph::LayerIds>();
  auto graph = std::make_shared<DynamicSceneGraph>(layer_ids);

  for (const auto& node : record.at("nodes")) {
    read_node_from_json(node_factory, node, *graph);
  }

  for (const auto& edge : record.at("edges")) {
    read_edge_from_json(edge_factory, edge, *graph);
  }

  if (!record.contains("mesh")) {
    return graph;
  }

  // TODO(nathan) push header serialization to to/from json and reuse
  auto mesh = Mesh::deserializeFromJson(record.at("mesh").dump());
  graph->setMesh(mesh);
  return graph;
}

}  // namespace io::json
}  // namespace spark_dsg
