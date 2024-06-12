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

#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"

namespace spark_dsg {

void write_binary(serialization::BinarySerializer& s, const SceneGraphNode& node) {
  s.startFixedArray(4);
  s.write(node.layer);
  s.write(node.id);
  s.write(node.timestamp);
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

NodeId parseNode(const AttributeFactory<NodeAttributes>& factory,
                 const BinaryDeserializer& deserializer,
                 DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(4);
  LayerId layer;
  deserializer.read(layer);
  NodeId node;
  deserializer.read(node);
  std::optional<std::chrono::nanoseconds> stamp;
  deserializer.read(stamp);

  auto attrs = serialization::Visitor::from(factory, deserializer);
  if (!attrs) {
    return node;
  }

  graph.addOrUpdateNode(layer, node, std::move(attrs), stamp);
  return node;
}

void parseEdge(const AttributeFactory<EdgeAttributes>& factory,
               const BinaryDeserializer& deserializer,
               DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(3);
  NodeId source;
  deserializer.read(source);
  NodeId target;
  deserializer.read(target);

  // last argument always forces parents to rewire
  auto attrs = serialization::Visitor::from(factory, deserializer);
  graph.addOrUpdateEdge(source, target, std::move(attrs));
}

void writeGraph(const DynamicSceneGraph& graph,
                std::vector<uint8_t>& buffer,
                bool include_mesh) {
  BinarySerializer serializer(&buffer);
  serializer.write(graph.layer_ids);

  // saves names to type index mapping
  serializer.write(serialization::AttributeRegistry<NodeAttributes>::names());
  serializer.write(serialization::AttributeRegistry<EdgeAttributes>::names());

  serializer.startDynamicArray();
  for (const auto& id_layer_pair : graph.layers()) {
    for (const auto& id_node_pair : id_layer_pair.second->nodes()) {
      serializer.write(*id_node_pair.second);
    }
  }
  serializer.endDynamicArray();

  serializer.startDynamicArray();
  for (const auto& id_layer_group_pair : graph.dynamicLayers()) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      for (const auto& node : prefix_layer_pair.second->nodes()) {
        serializer.write(*node);
      }
    }
  }
  serializer.endDynamicArray();

  serializer.startDynamicArray();
  for (const auto& id_layer_pair : graph.layers()) {
    for (const auto& id_edge_pair : id_layer_pair.second->edges()) {
      serializer.write(id_edge_pair.second);
    }
  }

  for (const auto& id_layer_group_pair : graph.dynamicLayers()) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      for (const auto& id_edge_pair : prefix_layer_pair.second->edges()) {
        serializer.write(id_edge_pair.second);
      }
    }
  }

  for (const auto& id_edge_pair : graph.interlayer_edges()) {
    serializer.write(id_edge_pair.second);
  }

  for (const auto& id_edge_pair : graph.dynamic_interlayer_edges()) {
    serializer.write(id_edge_pair.second);
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
    return serialization::AttributeRegistry<Attrs>::current();
  }

  std::vector<std::string> names;
  deserializer.read(names);
  return serialization::AttributeRegistry<Attrs>::fromNames(names);
}

bool updateGraph(DynamicSceneGraph& graph, const BinaryDeserializer& deserializer) {
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 0, 2)) {
    LayerId mesh_layer_id;
    deserializer.read(mesh_layer_id);
  }

  // load name to type index mapping if present
  const auto node_factory = loadFactory<NodeAttributes>(header, deserializer);
  const auto edge_factory = loadFactory<EdgeAttributes>(header, deserializer);

  std::unordered_set<NodeId> stale_nodes;
  for (const auto& id_key_pair : graph.node_lookup()) {
    stale_nodes.insert(id_key_pair.first);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    stale_nodes.erase(parseNode(node_factory, deserializer, graph));
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    stale_nodes.erase(parseNode(node_factory, deserializer, graph));
  }

  for (const auto& node_id : stale_nodes) {
    graph.removeNode(node_id);
  }

  graph.markEdgesAsStale();
  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    parseEdge(edge_factory, deserializer, graph);
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

  std::vector<LayerId> layer_ids;
  deserializer.read(layer_ids);

  auto graph = std::make_shared<DynamicSceneGraph>(layer_ids);
  if (!updateGraph(*graph, deserializer)) {
    return nullptr;
  }

  return graph;
}

bool updateGraph(DynamicSceneGraph& graph, const uint8_t* const buffer, size_t length) {
  BinaryDeserializer deserializer(buffer, length);

  std::vector<LayerId> layer_ids;
  deserializer.read(layer_ids);

  if (graph.layer_ids != layer_ids) {
    // TODO(nathan) maybe warn about mismatch
    graph.reset(layer_ids);
  }

  return updateGraph(graph, deserializer);
}

}  // namespace io::binary
}  // namespace spark_dsg
