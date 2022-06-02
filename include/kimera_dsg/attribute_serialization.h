#pragma once
#include "kimera_dsg/edge_attributes.h"
#include "kimera_dsg/node_attributes.h"

namespace kimera {
namespace attributes {

template <typename Converter>
void serialize(Converter& converter, const NodeAttributes& attrs) {
  converter.write("position", attrs.position);
  converter.write("last_update_time_ns", attrs.last_update_time_ns);
}

template <typename Converter>
void deserialize(const Converter& converter, NodeAttributes& attrs) {
  converter.read("position", attrs.position);
  converter.read("last_update_time_ns", attrs.last_update_time_ns);
}

template <typename Converter>
void serialize(Converter& converter, const SemanticNodeAttributes& attrs) {
  serialize(converter, static_cast<const NodeAttributes&>(attrs));
  converter.write("name", attrs.name);
  converter.write("color", attrs.color);
  converter.write("bounding_box", attrs.bounding_box);
  converter.write("semantic_label", attrs.semantic_label);
}

template <typename Converter>
void deserialize(const Converter& converter, SemanticNodeAttributes& attrs) {
  deserialize(converter, static_cast<NodeAttributes&>(attrs));
  converter.read("name", attrs.name);
  converter.read("color", attrs.color);
  converter.read("bounding_box", attrs.bounding_box);
  converter.read("semantic_label", attrs.semantic_label);
}

template <typename Converter>
void serialize(Converter& converter, const ObjectNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
  converter.write("registered", attrs.registered);
  converter.write("world_R_object", attrs.world_R_object);
}

template <typename Converter>
void deserialize(const Converter& converter, ObjectNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("registered", attrs.registered);
  converter.read("world_R_object", attrs.world_R_object);
}

template <typename Converter>
void serialize(Converter& converter, const RoomNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
}

template <typename Converter>
void deserialize(const Converter& converter, RoomNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
}

template <typename Converter>
void serialize(Converter& converter, const PlaceNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
  converter.write("distance", attrs.distance);
  converter.write("num_basis_points", attrs.num_basis_points);
  converter.write("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.write("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.write("is_active", attrs.is_active);
}

template <typename Converter>
void deserialize(const Converter& converter, PlaceNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("distance", attrs.distance);
  converter.read("num_basis_points", attrs.num_basis_points);
  converter.read("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.read("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.read("is_active", attrs.is_active);
}

template <typename Converter>
void serialize(Converter& converter, const AgentNodeAttributes& attrs) {
  serialize(converter, static_cast<const NodeAttributes&>(attrs));
  converter.write("world_R_body", attrs.world_R_body);
  converter.write("external_key", attrs.external_key);
  converter.write("dbow_ids", attrs.dbow_ids);
  converter.write("dbow_values", attrs.dbow_values);
}

template <typename Converter>
void deserialize(const Converter& converter, AgentNodeAttributes& attrs) {
  deserialize(converter, static_cast<NodeAttributes&>(attrs));
  converter.read("world_R_body", attrs.world_R_body);
  converter.read("external_key", attrs.external_key);
  converter.read("dbow_ids", attrs.dbow_ids);
  converter.read("dbow_values", attrs.dbow_values);
}

template <typename Converter>
void serialize(Converter& converter, const EdgeAttributes& attrs) {
  converter.write("weighted", attrs.weighted);
  converter.write("weight", attrs.weight);
}

template <typename Converter>
void deserialize(const Converter& converter, EdgeAttributes& attrs) {
  converter.read("weighted", attrs.weighted);
  converter.read("weight", attrs.weight);
}

}  // namespace attributes
}  // namespace kimera
