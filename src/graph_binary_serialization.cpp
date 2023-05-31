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
#include "spark_dsg/graph_binary_serialization.h"

#include "spark_dsg/binary_serializer.h"

namespace spark_dsg {

using serialization::BinaryConverter;
using serialization::BinaryDeserializer;
using serialization::BinaryEdgeFactory;
using serialization::BinaryNodeFactory;
using serialization::BinarySerializer;

template <>
std::unique_ptr<AttributeFactory<NodeAttributes, BinaryConverter>>
    AttributeFactory<NodeAttributes, BinaryConverter>::s_instance_ = nullptr;

template <>
std::unique_ptr<AttributeFactory<EdgeAttributes, BinaryConverter>>
    AttributeFactory<EdgeAttributes, BinaryConverter>::s_instance_ = nullptr;

void insertNode(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(3);
  LayerId layer;
  deserializer.read(layer);
  NodeId node;
  deserializer.read(node);

  BinaryConverter converter(&deserializer);
  graph.emplaceNode(layer, node, BinaryNodeFactory::get_default().create(converter));
  converter.finalize();
}

std::optional<NodeId> updateNode(const BinaryDeserializer& deserializer,
                                 DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(3);
  LayerId layer;
  deserializer.read(layer);
  NodeId node;
  deserializer.read(node);

  BinaryConverter conv(&deserializer);
  const auto node_opt = graph.getNode(node);
  if (node_opt) {
    BinaryNodeFactory::get_default().update(conv, node_opt->get().attributes());
  } else {
    graph.emplaceNode(layer, node, BinaryNodeFactory::get_default().create(conv));
  }
  conv.finalize();
  return node_opt ? std::optional<NodeId>(node_opt->get().id) : std::nullopt;
}

void insertDynamicNode(const BinaryDeserializer& deserializer,
                       DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(4);
  LayerId layer_id;
  deserializer.read(layer_id);
  NodeId node_id;
  deserializer.read(node_id);
  std::chrono::nanoseconds::rep timestamp_ns;
  deserializer.read(timestamp_ns);

  BinaryConverter converter(&deserializer);
  graph.emplacePrevDynamicNode(layer_id,
                               node_id,
                               std::chrono::nanoseconds(timestamp_ns),
                               BinaryNodeFactory::get_default().create(converter));
  converter.finalize();
}

std::optional<NodeId> updateDynamicNode(const BinaryDeserializer& deserializer,
                                        DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(4);
  LayerId layer;
  deserializer.read(layer);
  NodeId node;
  deserializer.read(node);
  std::chrono::nanoseconds::rep timestamp_ns_value;
  deserializer.read(timestamp_ns_value);
  std::chrono::nanoseconds timestamp_ns(timestamp_ns_value);

  BinaryConverter conv(&deserializer);
  const auto node_opt = graph.getNode(node);
  if (node_opt) {
    BinaryNodeFactory::get_default().update(conv, node_opt->get().attributes());
  } else {
    graph.emplacePrevDynamicNode(
        layer, node, timestamp_ns, BinaryNodeFactory::get_default().create(conv));
  }
  conv.finalize();
  return node_opt ? std::optional<NodeId>(node_opt->get().id) : std::nullopt;
}

void insertEdge(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(3);
  NodeId source;
  deserializer.read(source);
  NodeId target;
  deserializer.read(target);

  BinaryConverter conv(&deserializer);
  graph.insertEdge(source, target, BinaryEdgeFactory::get_default().create(conv));
  conv.finalize();
}

void updateEdge(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(3);
  NodeId source;
  deserializer.read(source);
  NodeId target;
  deserializer.read(target);

  BinaryConverter conv(&deserializer);
  const auto edge_opt = graph.getEdge(source, target);
  if (edge_opt) {
    BinaryEdgeFactory::get_default().update(conv, edge_opt->get().attributes());
  } else {
    // always force parents to switch
    graph.insertEdge(
        source, target, BinaryEdgeFactory::get_default().create(conv), true);
  }
  conv.finalize();
}

void insertMeshEdge(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
  deserializer.checkFixedArrayLength(2);
  NodeId source;
  deserializer.read(source);
  size_t target;
  deserializer.read(target);
  graph.insertMeshEdge(source, target, true);
}

void insertMesh(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
  size_t num_vertices = deserializer.readFixedArrayLength() / 6;
  MeshVertices::Ptr vertices(new MeshVertices());
  vertices->resize(num_vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    auto& point = vertices->at(i);
    deserializer.read(point.x);
    deserializer.read(point.y);
    deserializer.read(point.z);
    float r;
    deserializer.read(r);
    float g;
    deserializer.read(g);
    float b;
    deserializer.read(b);

    point.r = static_cast<uint8_t>(255.0f * r);
    point.g = static_cast<uint8_t>(255.0f * g);
    point.b = static_cast<uint8_t>(255.0f * b);
  }

  size_t num_faces = deserializer.readFixedArrayLength() / 3;
  std::shared_ptr<MeshFaces> faces(new MeshFaces(num_faces));
  for (size_t i = 0; i < num_faces; ++i) {
    auto& face = faces->at(i);
    face.vertices.resize(3);
    deserializer.read(face.vertices[0]);
    deserializer.read(face.vertices[1]);
    deserializer.read(face.vertices[2]);
  }

  graph.setMesh(vertices, faces, false);
}

void writeGraph(const DynamicSceneGraph& graph,
                std::vector<uint8_t>& buffer,
                bool include_mesh) {
  BinarySerializer serializer(&buffer);
  serializer.write(graph.layer_ids);
  serializer.write(graph.mesh_layer_id);

  serializer.writeArrayStart();
  for (const auto& id_layer_pair : graph.layers()) {
    for (const auto& id_node_pair : id_layer_pair.second->nodes()) {
      serializer.write(*id_node_pair.second);
    }
  }
  serializer.writeArrayEnd();

  serializer.writeArrayStart();
  for (const auto& id_layer_group_pair : graph.dynamicLayers()) {
    for (const auto& prefix_layer_pair : id_layer_group_pair.second) {
      for (const auto& node : prefix_layer_pair.second->nodes()) {
        serializer.write(*node);
      }
    }
  }
  serializer.writeArrayEnd();

  serializer.writeArrayStart();
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
  serializer.writeArrayEnd();

  serializer.writeArrayStart();
  for (const auto& edge : graph.getMeshEdges()) {
    serializer.write(edge.second);
  }
  serializer.writeArrayEnd();

  if (!include_mesh || !graph.hasMesh()) {
    serializer.write(false);
    return;
  }

  serializer.write(true);
  serializer.write(*graph.getMeshVertices());
  serializer.write(*graph.getMeshFaces());
}

DynamicSceneGraph::Ptr readGraph(const uint8_t* const buffer, size_t length) {
  BinaryDeserializer deserializer(buffer, length);

  std::vector<LayerId> layer_ids;
  LayerId mesh_layer_id;
  deserializer.read(layer_ids);
  deserializer.read(mesh_layer_id);

  auto graph = std::make_shared<DynamicSceneGraph>(layer_ids, mesh_layer_id);

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    insertNode(deserializer, *graph);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    insertDynamicNode(deserializer, *graph);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    insertEdge(deserializer, *graph);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    insertMeshEdge(deserializer, *graph);
  }

  serialization::PackType type;
  try {
    // TODO(nathan) this is ugly
    type = deserializer.getCurrType();
    deserializer.checkType(type);
  } catch (const std::out_of_range&) {
    std::cerr << "serialized data does not contain mesh flag!" << std::endl;
    return graph;
  }

  if (type != serialization::PackType::TRUE) {
    return graph;
  }

  insertMesh(deserializer, *graph);
  return graph;
}

bool updateGraphNormal(DynamicSceneGraph& graph,
                       const BinaryDeserializer& deserializer) {
  std::vector<LayerId> layer_ids;
  LayerId mesh_layer_id;
  deserializer.read(layer_ids);
  deserializer.read(mesh_layer_id);

  if (graph.layer_ids != layer_ids) {
    // TODO(nathan) maybe throw exception
    return false;
  }

  if (graph.mesh_layer_id != mesh_layer_id) {
    // TODO(nathan) maybe throw exception
    return false;
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    updateNode(deserializer, graph);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    updateDynamicNode(deserializer, graph);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    updateEdge(deserializer, graph);
  }

  // TODO(nathan) we might want to not do this
  graph.clearMeshEdges();
  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    // okay to directly insert, internal checks will prevent duplicates
    insertMeshEdge(deserializer, graph);
  }

  return true;
}

bool updateGraphRemoveStale(DynamicSceneGraph& graph,
                            const BinaryDeserializer& deserializer) {
  std::vector<LayerId> layer_ids;
  LayerId mesh_layer_id;
  deserializer.read(layer_ids);
  deserializer.read(mesh_layer_id);

  if (graph.layer_ids != layer_ids) {
    // TODO(nathan) maybe throw exception
    return false;
  }

  if (graph.mesh_layer_id != mesh_layer_id) {
    // TODO(nathan) maybe throw exception
    return false;
  }

  std::unordered_set<NodeId> stale_nodes;
  for (const auto& id_key_pair : graph.node_lookup()) {
    stale_nodes.insert(id_key_pair.first);
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    const auto node_id = updateNode(deserializer, graph);
    if (node_id) {
      stale_nodes.erase(*node_id);
    }
  }

  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    const auto node_id = updateDynamicNode(deserializer, graph);
    if (node_id) {
      stale_nodes.erase(*node_id);
    }
  }

  for (const auto& node_id : stale_nodes) {
    graph.removeNode(node_id);
  }

  graph.markEdgesAsStale();
  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    updateEdge(deserializer, graph);
  }
  graph.removeAllStaleEdges();

  // TODO(nathan) we might want to not do this
  graph.clearMeshEdges();
  deserializer.checkDynamicArray();
  while (!deserializer.isDynamicArrayEnd()) {
    // okay to directly insert, internal checks will prevent duplicates
    insertMeshEdge(deserializer, graph);
  }

  return true;
}

bool updateGraph(DynamicSceneGraph& graph,
                 const uint8_t* const buffer,
                 size_t length,
                 bool remove_stale) {
  BinaryDeserializer deserializer(buffer, length);
  if (remove_stale) {
    return updateGraphRemoveStale(graph, deserializer);
  }

  graph.clear();
  return updateGraphNormal(graph, deserializer);
}

}  // namespace spark_dsg
