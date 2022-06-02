#include "kimera_dsg/graph_binary_serialization.h"
#include "kimera_dsg/binary_serializer.h"

#include <glog/logging.h>

namespace kimera {

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

void updateNode(const BinaryDeserializer& deserializer, DynamicSceneGraph& graph) {
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

void updateDynamicNode(const BinaryDeserializer& deserializer,
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
    graph.insertEdge(source, target, BinaryEdgeFactory::get_default().create(conv));
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

void writeGraph(const DynamicSceneGraph& graph, std::vector<uint8_t>& buffer) {
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
  serializer.writeArrayEnd();

  serializer.writeArrayStart();
  for (const auto& edge : graph.getMeshEdges()) {
    serializer.write(edge.second);
  }
  serializer.writeArrayEnd();
}

DynamicSceneGraph::Ptr readGraph(const std::vector<uint8_t>& buffer) {
  BinaryDeserializer deserializer(&buffer);

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

  return graph;
}

bool updateGraph(DynamicSceneGraph& graph, const std::vector<uint8_t>& buffer) {
  BinaryDeserializer deserializer(&buffer);

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

}  // namespace kimera
