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
#include "spark_dsg/serialization/graph_binary_serialization.h"

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"

namespace spark_dsg {

void write_binary(serialization::BinarySerializer& s, const SceneGraphNode& node) {
  s.startFixedArray(4);
  s.write(node.layer.layer);
  s.write(node.id);
  // for parsing reasons for old files, this needs to be in roughly the same order as
  // the timestamp field
  s.write(node.layer.partition);
  s.write(node.attributes());
}

void write_binary(serialization::BinarySerializer& s, const SceneGraphEdge& edge) {
  s.startFixedArray(3);
  s.write(edge.source);
  s.write(edge.target);
  s.write(*edge.info);
}

namespace io::binary {

using spark_dsg::serialization::AttributeFactory;
using spark_dsg::serialization::BinaryDeserializer;
using spark_dsg::serialization::BinarySerializer;

using NodeCallback =
    std::function<void(LayerKey, NodeId, std::unique_ptr<NodeAttributes>&&)>;
using EdgeCallback =
    std::function<void(NodeId, NodeId, std::unique_ptr<EdgeAttributes>&&)>;

NodeId parseNode(const AttributeFactory<NodeAttributes>& factory,
                 const BinaryDeserializer& deserializer,
                 const NodeCallback& callback) {
  deserializer.checkFixedArrayLength(4);
  LayerId layer;
  deserializer.read(layer);
  NodeId node;
  deserializer.read(node);

  PartitionId partition = 0;
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 1, 0)) {
    io::warnOutdatedHeader(header);

    std::optional<std::chrono::nanoseconds> stamp;
    deserializer.read(stamp);
    // guess at interlayer IDs based on node symbol prefix
    if (stamp) {
      partition = NodeSymbol(node).category();
    }
  } else {
    deserializer.read(partition);
  }

  auto attrs = serialization::Visitor::from(factory, deserializer);
  if (!attrs) {
    return node;
  }

  callback({layer, partition}, node, std::move(attrs));
  return node;
}

void parseEdge(const AttributeFactory<EdgeAttributes>& factory,
               const BinaryDeserializer& deserializer,
               const EdgeCallback& callback) {
  deserializer.checkFixedArrayLength(3);
  NodeId source;
  deserializer.read(source);
  NodeId target;
  deserializer.read(target);

  // last argument always forces parents to rewire
  auto attrs = serialization::Visitor::from(factory, deserializer);
  callback(source, target, std::move(attrs));
}

void writeLayer(const SceneGraphLayer& graph, std::vector<uint8_t>& buffer) {
  BinarySerializer serializer(&buffer);
  serializer.write(graph.id.layer);  // we don't save partition IDs

  // saves names to type index mapping
  serializer.write(serialization::AttributeRegistry<NodeAttributes>::names());
  serializer.write(serialization::AttributeRegistry<EdgeAttributes>::names());

  serializer.startDynamicArray();
  for (const auto& id_node_pair : graph.nodes()) {
    serializer.write(*id_node_pair.second);
  }
  serializer.endDynamicArray();

  serializer.startDynamicArray();
  for (const auto& id_edge_pair : graph.edges()) {
    serializer.write(id_edge_pair.second);
  }
  serializer.endDynamicArray();
}

void writeGraph(const DynamicSceneGraph& graph,
                std::vector<uint8_t>& buffer,
                bool include_mesh) {
  BinarySerializer serializer(&buffer);
  serializer.write(graph.layer_keys());

  // saves names to type index mapping
  serializer.write(serialization::AttributeRegistry<NodeAttributes>::names());
  serializer.write(serialization::AttributeRegistry<EdgeAttributes>::names());

  serializer.write(graph.layer_names());

  // dump metadata to serialized json and write
  serializer.write(graph.metadata().dump());

  serializer.startDynamicArray();
  for (const auto& [layer_id, layer] : graph.layers()) {
    for (const auto& [node_id, node] : layer->nodes()) {
      serializer.write(*node);
    }
  }

  for (const auto& [layer_id, partitions] : graph.layer_partitions()) {
    for (const auto& [partition_id, partition] : partitions) {
      for (const auto& [node_id, node] : partition->nodes()) {
        serializer.write(*node);
      }
    }
  }
  serializer.endDynamicArray();

  serializer.startDynamicArray();
  for (const auto& [layer_id, layer] : graph.layers()) {
    for (const auto& [edge_id, edge] : layer->edges()) {
      serializer.write(edge);
    }
  }

  for (const auto& [layer_id, partitions] : graph.layer_partitions()) {
    for (const auto& [partition_id, partition] : partitions) {
      for (const auto& [edge_id, edge] : partition->edges()) {
        serializer.write(edge);
      }
    }
  }

  for (const auto& [edge_id, edge] : graph.interlayer_edges()) {
    serializer.write(edge);
  }
  serializer.endDynamicArray();

  auto mesh = graph.mesh();
  if (!include_mesh || !mesh) {
    serializer.write(false);
    return;
  }

  serializer.write(true);
  mesh->serializeToBinary(buffer);
}

template <typename Attrs>
AttributeFactory<Attrs> loadFactory(const io::FileHeader& header,
                                    const BinaryDeserializer& deserializer) {
  if (header.version < io::Version(1, 0, 2)) {
    io::warnOutdatedHeader(header);

    return serialization::AttributeRegistry<Attrs>::current();
  }

  std::vector<std::string> names;
  deserializer.read(names);
  return serialization::AttributeRegistry<Attrs>::fromNames(names);
}

bool updateGraph(DynamicSceneGraph& graph, const BinaryDeserializer& deserializer) {
  const auto& header = io::GlobalInfo::loadedHeader();

  if (header.version < io::Version(1, 1, 2)) {
    io::warnOutdatedHeader(header);

    // NOTE(nathan) we intentionally don't try to use the layer IDs to populate anything
    // because they will not include partitions and cause lots of serialization churn
    std::vector<LayerId> layer_ids;
    deserializer.read(layer_ids);
  } else {
    DynamicSceneGraph::LayerKeys layer_keys;
    deserializer.read(layer_keys);
    for (const auto& key : layer_keys) {
      graph.addLayer(key.layer, key.partition);
    }

    std::set<LayerKey> valid(layer_keys.begin(), layer_keys.end());
    const auto curr_keys = graph.layer_keys();
    for (const auto& key : curr_keys) {
      if (!valid.count(key)) {
        graph.removeLayer(key.layer, key.partition);
      }
    }
  }

  if (header.version < io::Version(1, 0, 2)) {
    io::warnOutdatedHeader(header);

    LayerId mesh_layer_id;
    deserializer.read(mesh_layer_id);
  }

  // load name to type index mapping if present
  const auto node_factory = loadFactory<NodeAttributes>(header, deserializer);
  const auto edge_factory = loadFactory<EdgeAttributes>(header, deserializer);

  std::map<std::string, LayerKey> layer_names;
  if (header.version < io::Version(1, 1, 0)) {
    io::warnOutdatedHeader(header);

    layer_names = {{DsgLayers::OBJECTS, 2},
                   {DsgLayers::AGENTS, 2},
                   {DsgLayers::PLACES, 3},
                   {DsgLayers::ROOMS, 4},
                   {DsgLayers::BUILDINGS, 5}};
  } else if (header.version < io::Version(1, 1, 1)) {
    io::warnOutdatedHeader(header);

    std::map<std::string, LayerId> names;
    deserializer.read(names);
    layer_names = DynamicSceneGraph::LayerNames(names.begin(), names.end());
  } else {
    deserializer.read(layer_names);
  }

  for (const auto& [name, key] : layer_names) {
    graph.addLayer(key.layer, key.partition, name);
  }

  if (header.version < io::Version(1, 0, 6)) {
    io::warnOutdatedHeader(header);
  } else {
    std::string metadata_json;
    deserializer.read(metadata_json);
    try {
      graph.metadata = nlohmann::json::parse(metadata_json);
    } catch (const std::exception& e) {
      SG_LOG(WARNING) << "Invalid json metadata: " << e.what();
    }
  }

  std::unordered_set<NodeId> stale_nodes;
  for (const auto& id_key_pair : graph.node_lookup()) {
    stale_nodes.insert(id_key_pair.first);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    stale_nodes.erase(parseNode(
        node_factory,
        deserializer,
        [&graph](const auto& key, const auto& node, auto&& attrs) {
          graph.addOrUpdateNode(key.layer, node, std::move(attrs), key.partition);
        }));
  }

  if (header.version < io::Version(1, 1, 0)) {
    io::warnOutdatedHeader(header);

    deserializer.checkDynamicArray();
    while (!deserializer.isDynamicArrayEnd()) {
      stale_nodes.erase(parseNode(
          node_factory,
          deserializer,
          [&graph](const auto& key, const auto& node, auto&& attrs) {
            graph.addOrUpdateNode(key.layer, node, std::move(attrs), key.partition);
          }));
    }
  }

  for (const auto& node_id : stale_nodes) {
    graph.removeNode(node_id);
  }

  graph.markEdgesAsStale();
  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    parseEdge(edge_factory,
              deserializer,
              [&graph](const auto& source, const auto& target, auto&& attrs) {
                graph.addOrUpdateEdge(source, target, std::move(attrs));
              });
  }
  graph.removeAllStaleEdges();

  if (!deserializer.checkIfTrue()) {
    return true;
  }

  auto mesh = std::make_shared<Mesh>();
  deserializer.read(*mesh);
  graph.setMesh(mesh);
  return true;
}

DynamicSceneGraph::Ptr readGraph(const uint8_t* const buffer, size_t length) {
  BinaryDeserializer deserializer(buffer, length);

  // make an empty graph
  auto graph = std::make_shared<DynamicSceneGraph>(true);
  if (!updateGraph(*graph, deserializer)) {
    return nullptr;
  }

  return graph;
}

std::shared_ptr<SceneGraphLayer> readLayer(const uint8_t* const buffer, size_t length) {
  const auto& header = io::GlobalInfo::loadedHeader();

  BinaryDeserializer deserializer(buffer, length);
  LayerId layer_id;
  deserializer.read(layer_id);

  auto graph = std::make_shared<SceneGraphLayer>(layer_id);

  // load name to type index mapping if present
  const auto node_factory = loadFactory<NodeAttributes>(header, deserializer);
  const auto edge_factory = loadFactory<EdgeAttributes>(header, deserializer);

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    parseNode(node_factory,
              deserializer,
              [&graph](const auto&, const auto& node, auto&& attrs) {
                graph->emplaceNode(node, std::move(attrs));
              });
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    parseEdge(edge_factory,
              deserializer,
              [&](const auto& source, const auto& target, auto&& attrs) {
                graph->insertEdge(source, target, std::move(attrs));
              });
  }

  return graph;
}

bool updateGraph(DynamicSceneGraph& graph, const uint8_t* const buffer, size_t length) {
  BinaryDeserializer deserializer(buffer, length);
  return updateGraph(graph, deserializer);
}

}  // namespace io::binary
}  // namespace spark_dsg
