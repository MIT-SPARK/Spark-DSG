#include "kimera_dsg/node_attributes.h"
#include "kimera_dsg/serialization_helpers.h"

using nlohmann::json;

namespace kimera {

// TODO(nathan) output stream operators are ugly

#define WRITE_FIELD_TO_JSON(record, field) record[#field] = field

#define READ_FIELD_FROM_JSON(record, field) \
  field = record.at(#field).get<decltype(field)>();

#define READ_FIELD_FROM_JSON_SAFE(record, field)      \
  if (record.contains(#field)) {                      \
    field = record.at(#field).get<decltype(field)>(); \
  }                                                   \
  static_assert(true, "")

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(), name(""), color(ColorVector::Zero()), semantic_label(0u) {}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << "  - Color : " << color.cast<float>().transpose() << std::endl
      << "  - Name: " << name << std::endl
      << "  - Bounding Box: " << bounding_box << std::endl
      << "  - Semantic Label: " << std::to_string(semantic_label) << std::endl;
  return out;
}

json SemanticNodeAttributes::toJson() const {
  json to_return = NodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(SemanticNodeAttributes, to_return);
  WRITE_FIELD_TO_JSON(to_return, name);
  WRITE_FIELD_TO_JSON(to_return, color);
  WRITE_FIELD_TO_JSON(to_return, bounding_box);
  WRITE_FIELD_TO_JSON(to_return, semantic_label);
  return to_return;
}

void SemanticNodeAttributes::fillFromJson(const json& record) {
  NodeAttributes::fillFromJson(record);
  READ_FIELD_FROM_JSON(record, name);
  READ_FIELD_FROM_JSON(record, color);
  READ_FIELD_FROM_JSON(record, bounding_box);
  READ_FIELD_FROM_JSON(record, semantic_label);
}

ObjectNodeAttributes::ObjectNodeAttributes()
    : SemanticNodeAttributes(), world_R_object(Eigen::Quaterniond::Identity()) {}

std::ostream& ObjectNodeAttributes::fill_ostream(std::ostream& out) const {
  // TODO(nathan) think about printing out rotation here
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - Is Registered?: " << (registered ? "yes" : "no") << std::endl;
  return out;
}

json ObjectNodeAttributes::toJson() const {
  json to_return = SemanticNodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(ObjectNodeAttributes, to_return);
  WRITE_FIELD_TO_JSON(to_return, registered);
  WRITE_FIELD_TO_JSON(to_return, world_R_object);
  return to_return;
}

void ObjectNodeAttributes::fillFromJson(const json& record) {
  SemanticNodeAttributes::fillFromJson(record);
  READ_FIELD_FROM_JSON(record, registered);
  READ_FIELD_FROM_JSON(record, world_R_object);
}

RoomNodeAttributes::RoomNodeAttributes() : SemanticNodeAttributes() {}

std::ostream& RoomNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  return out;
}

json RoomNodeAttributes::toJson() const {
  json to_return = SemanticNodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(RoomNodeAttributes, to_return);
  return to_return;
}

void RoomNodeAttributes::fillFromJson(const json& record) {
  SemanticNodeAttributes::fillFromJson(record);
}

PlaceNodeAttributes::PlaceNodeAttributes()
    : SemanticNodeAttributes(), distance(0.0), num_basis_points(0) {}

PlaceNodeAttributes::PlaceNodeAttributes(double distance, unsigned int num_basis_points)
    : SemanticNodeAttributes(),
      distance(distance),
      num_basis_points(num_basis_points) {}

std::ostream& PlaceNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << " - distance: " << distance;
  out << " - num_basis_points: " << num_basis_points;
  out << " - is_active: " << is_active;
  return out;
}

json PlaceNodeAttributes::toJson() const {
  json to_return = SemanticNodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(PlaceNodeAttributes, to_return);
  WRITE_FIELD_TO_JSON(to_return, distance);
  WRITE_FIELD_TO_JSON(to_return, num_basis_points);
  WRITE_FIELD_TO_JSON(to_return, voxblox_mesh_connections);
  WRITE_FIELD_TO_JSON(to_return, pcl_mesh_connections);
  WRITE_FIELD_TO_JSON(to_return, is_active);
  return to_return;
}

void PlaceNodeAttributes::fillFromJson(const json& record) {
  SemanticNodeAttributes::fillFromJson(record);
  READ_FIELD_FROM_JSON(record, distance);
  READ_FIELD_FROM_JSON(record, num_basis_points);
  READ_FIELD_FROM_JSON_SAFE(record, voxblox_mesh_connections);
  READ_FIELD_FROM_JSON_SAFE(record, pcl_mesh_connections);
  READ_FIELD_FROM_JSON_SAFE(record, is_active);
}

std::ostream& operator<<(std::ostream& out, const Eigen::Quaterniond& q) {
  return out << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
}

AgentNodeAttributes::AgentNodeAttributes() : NodeAttributes() {}

AgentNodeAttributes::AgentNodeAttributes(const Eigen::Quaterniond& world_R_body,
                                         const Eigen::Vector3d& world_P_body,
                                         NodeId external_key)
    : NodeAttributes(world_P_body),
      world_R_body(world_R_body),
      external_key(external_key) {}

std::ostream& AgentNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << " - orientation: " << world_R_body;
  return out;
}

json AgentNodeAttributes::toJson() const {
  json to_return = NodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(AgentNodeAttributes, to_return);
  WRITE_FIELD_TO_JSON(to_return, world_R_body);
  WRITE_FIELD_TO_JSON(to_return, external_key);
  WRITE_FIELD_TO_JSON(to_return, dbow_ids);
  WRITE_FIELD_TO_JSON(to_return, dbow_values);
  return to_return;
}

void AgentNodeAttributes::fillFromJson(const json& record) {
  NodeAttributes::fillFromJson(record);
  READ_FIELD_FROM_JSON(record, world_R_body);
  READ_FIELD_FROM_JSON(record, external_key);
  READ_FIELD_FROM_JSON(record, dbow_ids);
  READ_FIELD_FROM_JSON(record, dbow_values);
}

}  // namespace kimera
