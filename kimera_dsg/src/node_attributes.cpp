#include "kimera_dsg/node_attributes.h"
#include "kimera_dsg/serialization_helpers.h"

using nlohmann::json;

namespace kimera {

#define WRITE_FIELD_TO_JSON(record, field) record[#field] = field

#define READ_FIELD_FROM_JSON(record, field) \
  field = record.at(#field).get<decltype(field)>();

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(), name(""), color(ColorVector::Zero()), semantic_label(0u) {}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  // TODO(nathan) think about printing out rotation here
  NodeAttributes::fill_ostream(out);
  out << "  - Color : " << color << std::endl
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
    : world_R_object(Eigen::Quaterniond::Identity()), SemanticNodeAttributes() {}

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
  PlaceNodeAttributes::fill_ostream(out);
  out << " - distance: " << distance;
  out << " - num_basis_points: " << num_basis_points;
  return out;
}

json PlaceNodeAttributes::toJson() const {
  json to_return = SemanticNodeAttributes::toJson();
  REGISTER_JSON_ATTR_TYPE(PlaceNodeAttributes, to_return);
  WRITE_FIELD_TO_JSON(to_return, distance);
  WRITE_FIELD_TO_JSON(to_return, num_basis_points);
  return to_return;
}

void PlaceNodeAttributes::fillFromJson(const json& record) {
  SemanticNodeAttributes::fillFromJson(record);
  READ_FIELD_FROM_JSON(record, distance);
  READ_FIELD_FROM_JSON(record, num_basis_points);
}

}  // namespace kimera
