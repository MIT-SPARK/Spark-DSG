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
#include "spark_dsg/bounding_box.h"

#include "spark_dsg/bounding_box_extraction.h"
#include "spark_dsg/scene_graph_types.h"

namespace spark_dsg {

BoundingBox::BoundingBox(const Eigen::Vector3f& dimensions)
    : type(Type::AABB), dimensions(dimensions), world_P_center(dimensions / 2) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& dimensions,
                         const Eigen::Vector3f& world_P_center)
    : type(Type::AABB), dimensions(dimensions), world_P_center(world_P_center) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& dimensions,
                         const Eigen::Vector3f& world_P_center,
                         float yaw)
    : type(Type::RAABB), dimensions(dimensions), world_P_center(world_P_center) {
  world_R_center = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
};

BoundingBox::BoundingBox(const Eigen::Vector3f& dimensions,
                         const Eigen::Vector3f& world_P_center,
                         const Eigen::Quaternionf& world_R_center)
    : type(Type::OBB),
      dimensions(dimensions),
      world_P_center(world_P_center),
      world_R_center(world_R_center.toRotationMatrix()) {}

BoundingBox::BoundingBox(BoundingBox::Type type,
                         const Eigen::Vector3f& dimensions,
                         const Eigen::Vector3f& world_P_center,
                         const Eigen::Matrix3f& world_R_center)
    : type(type),
      dimensions(dimensions),
      world_P_center(world_P_center),
      world_R_center(world_R_center) {}

BoundingBox::BoundingBox(const PointAdaptor& points, BoundingBox::Type type) {
  *this = bounding_box::extract(points, type);
}

BoundingBox::BoundingBox(const std::vector<Eigen::Vector3f>& points,
                         BoundingBox::Type type) {
  *this = bounding_box::extract(PointVectorAdaptor(points), type);
}

BoundingBox::BoundingBox(const Mesh& mesh, BoundingBox::Type type) {
  *this = bounding_box::extract(MeshAdaptor(mesh), type);
}

bool BoundingBox::isValid() const {
  if (type == Type::INVALID) {
    return false;
  }
  return volume() > 0.0f;
}

float BoundingBox::volume() const { return std::max(dimensions.prod(), 0.0f); }

bool BoundingBox::hasRotation() const {
  if (type == Type::INVALID || type == Type::AABB) {
    return false;
  }
  return world_R_center != Eigen::Matrix3f::Identity();
}

Eigen::Vector3f BoundingBox::minCorner() const {
  return world_P_center - dimensions / 2;
}

Eigen::Vector3f BoundingBox::maxCorner() const {
  return world_P_center + dimensions / 2;
}

std::array<Eigen::Vector3f, 8> BoundingBox::corners() const {
  const Eigen::Vector3f min = -dimensions / 2;
  const Eigen::Vector3f max = dimensions / 2;
  std::array<Eigen::Vector3f, 8> corners = {
      min,
      {max(0), min(1), min(2)},
      {max(0), max(1), min(2)},
      {min(0), max(1), min(2)},

      {min(0), min(1), max(2)},
      {max(0), min(1), max(2)},
      max,
      {min(0), max(1), max(2)},
  };
  if (hasRotation()) {
    for (Eigen::Vector3f& corner : corners) {
      corner = world_R_center * corner;
    }
  }
  for (Eigen::Vector3f& corner : corners) {
    corner += world_P_center;
  }
  return corners;
}

void BoundingBox::merge(const BoundingBox& other) {
  if (!other.isValid()) {
    return;
  }
  if (!isValid()) {
    *this = other;
    return;
  }

  if (type == Type::AABB && other.type == Type::AABB) {
    mergeAABB(other);
    return;
  }

  mergeGeneral(other);
}

void BoundingBox::mergeAABB(const BoundingBox& other) {
  const Eigen::Vector3f min = minCorner().cwiseMin(other.minCorner());
  const Eigen::Vector3f max = maxCorner().cwiseMax(other.maxCorner());
  dimensions = max - min;
  world_P_center = (min + max) / 2;
}

void BoundingBox::mergeGeneral(const BoundingBox& other) {
  // Compute all corner points.
  std::vector<Eigen::Vector3f> points;
  points.reserve(16);
  for (const Eigen::Vector3f& corner : corners()) {
    points.push_back(corner);
  }
  for (const Eigen::Vector3f& corner : other.corners()) {
    points.push_back(corner);
  }

  // Compute the new bounding box of the same type.
  PointVectorAdaptor adaptor(points);
  *this = bounding_box::extract(adaptor, type);
}

Eigen::Vector3f BoundingBox::pointToWorldFrame(const Eigen::Vector3f& point_B) const {
  if (hasRotation()) {
    return world_R_center * point_B + world_P_center;
  } else {
    return point_B + world_P_center;
  }
}

Eigen::Vector3f BoundingBox::pointToBoxFrame(const Eigen::Vector3f& point_W) const {
  if (hasRotation()) {
    return world_R_center.transpose() * (point_W - world_P_center);
  } else {
    return point_W - world_P_center;
  }
}

bool BoundingBox::contains(const Eigen::Vector3f& point) const {
  if (!isValid()) {
    return false;
  }
  return isInside(pointToBoxFrame(point));
}

bool BoundingBox::contains(const Eigen::Vector3d& point) const {
  return contains(static_cast<Eigen::Vector3f>(point.cast<float>()));
}

bool BoundingBox::intersects(const BoundingBox& other) const {
  if (!isValid() || !other.isValid()) {
    return false;
  }

  // Currently only supports AABB-AABB intersection.
  if (type != Type::AABB || other.type != Type::AABB) {
    return false;
  }

  return (minCorner().array() < other.maxCorner().array()).all() &&
         (maxCorner().array() > other.minCorner().array()).all();
}

float BoundingBox::computeIoU(const BoundingBox& other) const {
  if (!isValid() || !other.isValid()) {
    return 0;
  }

  // Currently only supports AABB-AABB intersection.
  if (type != Type::AABB || other.type != Type::AABB) {
    return 0;
  }

  const Eigen::Vector3f intersection_min = minCorner().cwiseMax(other.minCorner());
  const Eigen::Vector3f intersection_max = maxCorner().cwiseMin(other.maxCorner());
  const Eigen::Vector3f intersection_size =
      (intersection_max - intersection_min).cwiseMax(0);
  const float intersection_volume = intersection_size.prod();
  const float union_volume = volume() + other.volume() - intersection_volume;
  return intersection_volume / union_volume;
}

bool BoundingBox::operator==(const BoundingBox& other) const {
  if (type != other.type) {
    return false;
  }
  if (type == BoundingBox::Type::INVALID) {
    return true;
  }
  if (dimensions != other.dimensions || world_P_center != other.world_P_center) {
    return false;
  }
  if (type == BoundingBox::Type::AABB) {
    return true;
  }
  return world_R_center == other.world_R_center;
}

std::ostream& operator<<(std::ostream& out, const Eigen::Quaternionf& q) {
  out << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
  return out;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& box) {
  if (box.type == BoundingBox::Type::INVALID) {
    os << "invalid";
    return os;
  }

  auto format = getDefaultVectorFormat();
  os << "{pos: " << box.world_P_center.transpose().format(format)
     << ", dim: " << box.dimensions.transpose().format(format);
  if (box.type == BoundingBox::Type::RAABB || box.type == BoundingBox::Type::OBB) {
    os << ", rot: " << Eigen::Quaternionf(box.world_R_center);
  }
  os << "}";
  return os;
}

bool BoundingBox::isInside(const Eigen::Vector3f& point_B) const {
  return (point_B.cwiseAbs() - dimensions / 2).maxCoeff() <= 0;
}

size_t BoundingBox::MeshAdaptor::size() const {
  return indices ? indices->size() : mesh.numVertices();
}

Eigen::Vector3f BoundingBox::MeshAdaptor::get(size_t index) const {
  const size_t global_idx = indices ? indices->at(index) : index;
  return mesh.pos(global_idx);
}

size_t BoundingBox::PointVectorAdaptor::size() const { return points.size(); }

Eigen::Vector3f BoundingBox::PointVectorAdaptor::get(size_t index) const {
  return points[index];
}

}  // namespace spark_dsg
