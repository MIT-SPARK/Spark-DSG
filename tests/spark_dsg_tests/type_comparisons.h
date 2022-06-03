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
#pragma once
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>

namespace spark_dsg {

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

inline bool operator==(const SemanticNodeAttributes& lhs,
                       const SemanticNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label;
}

inline bool operator==(const ObjectNodeAttributes& lhs,
                       const ObjectNodeAttributes& rhs) {
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

}  // namespace spark_dsg
