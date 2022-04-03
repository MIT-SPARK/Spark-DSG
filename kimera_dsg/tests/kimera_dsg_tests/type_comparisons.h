#pragma once
#include <kimera_dsg/edge_attributes.h>
#include <kimera_dsg/node_attributes.h>

namespace kimera {

template <typename Scalar>
bool quaternionsEqual(const Eigen::Quaternion<Scalar>& lhs,
                      const Eigen::Quaternion<Scalar>& rhs) {
  return lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() &&
         lhs.z() == rhs.z();
}

inline bool operator==(const BoundingBox& lhs, const BoundingBox& rhs) {
  if (lhs.type != rhs.type) {
    return false;
  }

  switch (lhs.type) {
    case BoundingBox::Type::INVALID:
      return true;
    case BoundingBox::Type::AABB:
      return lhs.min == rhs.min && lhs.max == rhs.max;
    case BoundingBox::Type::OBB:
      return lhs.min == rhs.min && lhs.max == rhs.max &&
             lhs.world_P_center == rhs.world_P_center &&
             quaternionsEqual(Eigen::Quaternionf(lhs.world_R_center),
                              Eigen::Quaternionf(rhs.world_R_center));
    default:
      return false;
  }
}

inline bool operator==(const NodeAttributes& lhs, const NodeAttributes& rhs) {
  return lhs.position == rhs.position;
}

inline bool operator==(const SemanticNodeAttributes& lhs, const SemanticNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label;
}

inline bool operator==(const ObjectNodeAttributes& lhs, const ObjectNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label && lhs.registered == rhs.registered &&
         quaternionsEqual(lhs.world_R_object, rhs.world_R_object);
}

inline bool operator==(const RoomNodeAttributes& lhs, const RoomNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label;
}

inline bool operator==(const PlaceNodeAttributes& lhs, const PlaceNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label && lhs.distance == rhs.distance &&
         lhs.num_basis_points == rhs.num_basis_points;
}

inline bool operator==(const EdgeAttributes& lhs, const EdgeAttributes& rhs) {
  return lhs.weighted == rhs.weighted && lhs.weight == rhs.weight;
}

}  // namespace kimera
