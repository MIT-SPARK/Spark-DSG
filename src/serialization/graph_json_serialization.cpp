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

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using nlohmann::json;

void to_json(json& record, const SceneGraphNode& node) {
  record = {{"id", node.id},
            {"layer", node.layer.layer},
            {"partition", node.layer.partition},
            {"attributes", node.attributes()}};
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

  PartitionId partition = 0;
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 1, 0)) {
    io::warnOutdatedHeader(header);

    if (record.contains("timestamp")) {
      partition = NodeSymbol(node_id).category();
    }
  } else {
    partition = record.at("partition").get<PartitionId>();
  }

  auto attrs = serialization::Visitor::from(factory, record.at("attributes"));
  if (!attrs) {
    std::stringstream ss;
    ss << "invalid attributes for " << NodeSymbol(node_id).str();
    throw std::runtime_error(ss.str());
  }

  if (!graph.emplaceNode(layer, node_id, std::move(attrs), partition)) {
    std::stringstream ss;
    ss << "failed to add " << NodeSymbol(node_id).str();
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
    ss << "failed to add " << NodeSymbol(source).str() << " →  "
       << NodeSymbol(target).str();
    throw std::runtime_error(ss.str());
  }
}

namespace io::json {

std::string writeGraph(const DynamicSceneGraph& graph, bool include_mesh) {
  nlohmann::json record;
  record[io::FileHeader::header_json_key()] = io::FileHeader::current();
  record["directed"] = false;
  record["multigraph"] = false;
  record["nodes"] = nlohmann::json::array();
  record["edges"] = nlohmann::json::array();
  record["layer_keys"] = graph.layer_keys();
  record["layer_names"] = graph.layer_names();
  record["metadata"] = graph.metadata();

  for (const auto& [layer_id, layer] : graph.layers()) {
    for (const auto& [node_id, node] : layer->nodes()) {
      record["nodes"].push_back(*node);
    }

    for (const auto& [edge_id, edge] : layer->edges()) {
      record["edges"].push_back(edge);
    }
  }

  for (const auto& [edge_id, edge] : graph.interlayer_edges()) {
    record["edges"].push_back(edge);
  }

  for (const auto& [layer_id, partitions] : graph.layer_partitions()) {
    for (const auto& [partition_id, partition] : partitions) {
      for (const auto& [node_id, node] : partition->nodes()) {
        record["nodes"].push_back(*node);
      }

      for (const auto& [edge_id, edge] : partition->edges()) {
        record["edges"].push_back(edge);
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
  const auto header_field_name = FileHeader::header_json_key();
  const auto header = record.contains(header_field_name)
                          ? record.at(header_field_name).get<io::FileHeader>()
                          : io::FileHeader::legacy();
  io::GlobalInfo::ScopedInfo info(header);
  const auto node_factory = serialization::AttributeRegistry<NodeAttributes>::current();
  const auto edge_factory = serialization::AttributeRegistry<EdgeAttributes>::current();

  DynamicSceneGraph::LayerKeys layer_keys;
  if (header.version < io::Version(1, 1, 2)) {
    io::warnOutdatedHeader(header);

    const auto layer_ids = record.at("layer_ids").get<std::vector<LayerId>>();
    layer_keys = DynamicSceneGraph::LayerKeys(layer_ids.begin(), layer_ids.end());
  } else {
    record.at("layer_keys").get_to(layer_keys);
  }

  DynamicSceneGraph::LayerNames layer_names;
  if (header.version < io::Version(1, 1, 0)) {
    io::warnOutdatedHeader(header);

    layer_names = {{DsgLayers::OBJECTS, 2},
                   {DsgLayers::AGENTS, 2},
                   {DsgLayers::PLACES, 3},
                   {DsgLayers::ROOMS, 4},
                   {DsgLayers::BUILDINGS, 5}};
  } else if (header.version < io::Version(1, 1, 1)) {
    io::warnOutdatedHeader(header);

    const auto names = record.at("layer_names").get<std::map<std::string, LayerId>>();
    layer_names = DynamicSceneGraph::LayerNames(names.begin(), names.end());
  } else {
    layer_names = record.at("layer_names").get<DynamicSceneGraph::LayerNames>();
  }

  auto graph = std::make_shared<DynamicSceneGraph>(layer_keys, layer_names);

  if (record.contains("metadata")) {
    graph->metadata = record["metadata"];
  }

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
