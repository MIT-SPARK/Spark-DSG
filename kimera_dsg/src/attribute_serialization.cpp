#include "kimera_dsg/attribute_serialization.h"
#include "kimera_dsg/attribute_serializer.h"
#include "kimera_dsg/serialization_helpers.h"

namespace kimera {

using NodeFactoryBase = AttributeFactory<NodeAttributes>;

template <>
std::unique_ptr<NodeFactoryBase> NodeFactoryBase::s_instance_ = nullptr;

using EdgeFactoryBase = AttributeFactory<EdgeAttributes>;

template <>
std::unique_ptr<EdgeFactoryBase> EdgeFactoryBase::s_instance_ = nullptr;

NodeFactoryBase& NodeAttributeFactory::get_default() {
  auto& factory = NodeAttributeFactory::instance();
  if (factory.default_set) {
    return factory;
  }

  factory.add("NodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<NodeAttributes>();
  });
  factory.add("SemanticNodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<SemanticNodeAttributes>();
  });
  factory.add("ObjectNodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<ObjectNodeAttributes>();
  });
  factory.add("RoomNodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<RoomNodeAttributes>();
  });
  factory.add("PlaceNodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<PlaceNodeAttributes>();
  });
  factory.add("AgentNodeAttributes", []() -> NodeAttributes::Ptr {
    return std::make_unique<AgentNodeAttributes>();
  });
  factory.default_set = true;
  return factory;
}

EdgeFactoryBase& EdgeAttributeFactory::get_default() {
  auto& factory = EdgeAttributeFactory::instance();
  if (factory.default_set) {
    return factory;
  }

  factory.add("EdgeAttributes", []() -> EdgeAttributes::Ptr {
    return std::make_unique<EdgeAttributes>();
  });
  // backwards compatability
  factory.add("SceneGraphEdgeInfo", []() -> EdgeAttributes::Ptr {
    return std::make_unique<EdgeAttributes>();
  });
  return factory;
}

void NodeAttributes::serialize(AttributeSerializer& serializer) const {
  serializer.mark_type("NodeAttributes");
  serializer.write("position", position);
}

void NodeAttributes::deserialize(const AttributeSerializer& serializer) {
  serializer.read("position", position);
}

void SemanticNodeAttributes::serialize(AttributeSerializer& serializer) const {
  NodeAttributes::serialize(serializer);
  serializer.mark_type("SemanticNodeAttributes");

  serializer.write("name", name);
  serializer.write("color", color);
  serializer.write("bounding_box", bounding_box);
  serializer.write("semantic_label", semantic_label);
}

void SemanticNodeAttributes::deserialize(const AttributeSerializer& serializer) {
  NodeAttributes::deserialize(serializer);
  serializer.read("name", name);
  serializer.read("color", color);
  serializer.read("bounding_box", bounding_box);
  serializer.read("semantic_label", semantic_label);
}

void ObjectNodeAttributes::serialize(AttributeSerializer& serializer) const {
  SemanticNodeAttributes::serialize(serializer);

  serializer.mark_type("ObjectNodeAttributes");
  serializer.write("registered", registered);
  serializer.write("world_R_object", world_R_object);
}

void ObjectNodeAttributes::deserialize(const AttributeSerializer& serializer) {
  SemanticNodeAttributes::deserialize(serializer);
  serializer.read("registered", registered);
  serializer.read("world_R_object", world_R_object);
}

void RoomNodeAttributes::serialize(AttributeSerializer& serializer) const {
  SemanticNodeAttributes::serialize(serializer);

  serializer.mark_type("RoomNodeAttributes");
}

void RoomNodeAttributes::deserialize(const AttributeSerializer& serializer) {
  SemanticNodeAttributes::deserialize(serializer);
}

void PlaceNodeAttributes::serialize(AttributeSerializer& serializer) const {
  SemanticNodeAttributes::serialize(serializer);

  serializer.mark_type("PlaceNodeAttributes");
  serializer.write("distance", distance);
  serializer.write("num_basis_points", num_basis_points);
  serializer.write("voxblox_mesh_connections", voxblox_mesh_connections);
  serializer.write("pcl_mesh_connections", pcl_mesh_connections);
  serializer.write("is_active", is_active);
}

void PlaceNodeAttributes::deserialize(const AttributeSerializer& serializer) {
  SemanticNodeAttributes::deserialize(serializer);
  serializer.read("distance", distance);
  serializer.read("num_basis_points", num_basis_points);
  serializer.read("voxblox_mesh_connections", voxblox_mesh_connections);
  serializer.read("pcl_mesh_connections", pcl_mesh_connections);
  serializer.read("is_active", is_active);
}

void AgentNodeAttributes::serialize(AttributeSerializer& serializer) const {
  NodeAttributes::serialize(serializer);

  serializer.mark_type("AgentNodeAttributes");
  serializer.write("world_R_body", world_R_body);
  serializer.write("external_key", external_key);
  serializer.write("dbow_ids", dbow_ids);
  serializer.write("dbow_values", dbow_values);
}

void AgentNodeAttributes::deserialize(const AttributeSerializer& serializer) {
  NodeAttributes::deserialize(serializer);
  serializer.read("world_R_body", world_R_body);
  serializer.read("external_key", external_key);
  serializer.read("dbow_ids", dbow_ids);
  serializer.read("dbow_values", dbow_values);
}

void EdgeAttributes::serialize(AttributeSerializer& serializer) const {
  serializer.mark_type("EdgeAttributes");
  serializer.write("weighted", weighted);
  serializer.write("weight", weight);
}

void EdgeAttributes::deserialize(const AttributeSerializer& serializer) {
  serializer.read("weighted", weighted);
  serializer.read("weight", weight);
}

}  // namespace kimera
