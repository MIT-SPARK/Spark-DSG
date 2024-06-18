#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/node_attributes.h"

namespace spark_dsg {

inline NodeAttributes getNodeAttributes() {
  NodeAttributes expected;
  expected.position << 1.0, 2.0, 3.0;
  return expected;
}

inline SemanticNodeAttributes getSemanticNodeAttributes() {
  SemanticNodeAttributes expected;
  expected.position << 1.0, 2.0, 3.0;
  expected.name = "semantic_attributes";
  expected.color = Color(4, 5, 6);
  expected.bounding_box.type = BoundingBox::Type::AABB;
  expected.bounding_box.world_P_center << 7.0f, 8.0f, 9.0f;
  expected.bounding_box.dimensions << 10.0f, 11.0f, 12.0f;
  expected.semantic_label = 13;
  return expected;
}

inline ObjectNodeAttributes getObjectNodeAttributes() {
  ObjectNodeAttributes expected;
  expected.position << 1.0, 2.0, 3.0;
  expected.name = "object_attributes";
  expected.color = Color(4, 5, 6);
  expected.bounding_box.type = BoundingBox::Type::AABB;
  expected.bounding_box.world_P_center << 7.0f, 8.0f, 9.0f;
  expected.bounding_box.dimensions << 10.0f, 11.0f, 12.0f;
  expected.semantic_label = 13;
  expected.registered = true;
  expected.world_R_object = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);
  return expected;
}

inline RoomNodeAttributes getRoomNodeAttributes() {
  RoomNodeAttributes expected;
  expected.position << 1.0, 2.0, 3.0;
  expected.name = "room_attributes";
  expected.color = Color(4, 5, 6);
  expected.bounding_box.type = BoundingBox::Type::AABB;
  expected.bounding_box.world_P_center << 7.0f, 8.0f, 9.0f;
  expected.bounding_box.dimensions << 10.0f, 11.0f, 12.0f;
  expected.semantic_label = 13;
  return expected;
}

inline PlaceNodeAttributes getPlaceNodeAttributes() {
  PlaceNodeAttributes expected;
  expected.position << 1.0, 2.0, 3.0;
  expected.name = "place_attributes";
  expected.color = Color(4, 5, 6);
  expected.bounding_box.type = BoundingBox::Type::AABB;
  expected.bounding_box.world_P_center << 7.0f, 8.0f, 9.0f;
  expected.bounding_box.dimensions << 10.0f, 11.0f, 12.0f;
  expected.semantic_label = 13;
  expected.distance = 14.0;
  expected.num_basis_points = 15;
  return expected;
}

inline KhronosObjectAttributes getKhronosObjectAttributes() {
  KhronosObjectAttributes expected;
  expected.first_observed_ns = {0, 1, 2};
  expected.last_observed_ns = {3, 4, 5};
  expected.mesh.resizeVertices(6);
  expected.mesh.resizeFaces(7);
  expected.trajectory_timestamps = {8, 9, 10};
  expected.trajectory_positions.resize(11, Eigen::Vector3f::UnitX());
  expected.dynamic_object_points.emplace_back(12, Eigen::Vector3f::UnitY());
  expected.dynamic_object_points.emplace_back(13, Eigen::Vector3f::UnitZ());
  expected.details["test"] = {14, 15, 16};
  expected.details["test2"] = {17, 18, 19};
  return expected;
}

inline EdgeAttributes getEdgeAttributes() {
  EdgeAttributes expected;
  expected.weighted = true;
  expected.weight = 5.0;
  return expected;
}

}  // namespace spark_dsg
