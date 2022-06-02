#include "kimera_dsg/bounding_box.h"
#include "kimera_dsg/scene_graph_types.h"

namespace kimera {

std::ostream& operator<<(std::ostream& out, const Eigen::Quaternionf& q) {
  out << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
  return out;
}

BoundingBox::BoundingBox()
    : type(Type::INVALID),
      min(Eigen::Vector3f::Zero()),
      max(Eigen::Vector3f::Zero()),
      world_P_center(Eigen::Vector3f::Zero()),
      world_R_center(Eigen::Matrix3f::Identity()) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max)
    : type(Type::AABB),
      min(min),
      max(max),
      world_P_center(Eigen::Vector3f::Zero()),
      world_R_center(Eigen::Matrix3f::Identity()) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& min,
                         const Eigen::Vector3f& max,
                         const Eigen::Vector3f& world_P_center,
                         const Eigen::Quaternionf& world_R_center)
    : type(Type::OBB),
      min(min),
      max(max),
      world_P_center(world_P_center),
      world_R_center(world_R_center.toRotationMatrix()) {}

BoundingBox::BoundingBox(BoundingBox::Type type,
                         const Eigen::Vector3f& min,
                         const Eigen::Vector3f& max,
                         const Eigen::Vector3f& world_P_center,
                         const Eigen::Matrix3f& world_R_center)
    : type(type),
      min(min),
      max(max),
      world_P_center(world_P_center),
      world_R_center(world_R_center) {}

std::ostream& operator<<(std::ostream& os, const BoundingBox& box) {
  if (box.type == BoundingBox::Type::INVALID) {
    os << "invalid";
    return os;
  }

  auto format = getDefaultVectorFormat();
  os << "{min: " << box.min.transpose().format(format)
     << ", max: " << box.max.transpose().format(format);
  if (box.type == BoundingBox::Type::RAABB || box.type == BoundingBox::Type::OBB) {
    os << ", rot: " << Eigen::Quaternionf(box.world_R_center);
  }
  if (box.type == BoundingBox::Type::OBB) {
    os << ", pos: " << box.world_P_center.transpose().format(format);
  }
  os << "}";
  return os;
};

// TODO(nathan) consider template instead
bool BoundingBox::isInside(const Eigen::Vector3d& point) const {
  const Eigen::Vector3f point_float = point.cast<float>();
  return isInside(point_float);
}

bool BoundingBox::isInsideOBB(const Eigen::Vector3f& point_world) const {
  Eigen::Vector3f point = world_R_center.transpose() * (point_world - world_P_center);
  return min(0) <= point(0) && point(0) <= max(0) && min(1) <= point(1) &&
         point(1) <= max(1) && min(2) <= point(2) && point(2) <= max(2);
}

bool BoundingBox::isInside(const Eigen::Vector3f& point) const {
  switch (type) {
    case Type::AABB:
      return min(0) <= point(0) && point(0) <= max(0) && min(1) <= point(1) &&
             point(1) <= max(1) && min(2) <= point(2) && point(2) <= max(2);
    case Type::OBB:
    case Type::RAABB:  // RAABB is the same as OBB (despite the function name)
      return isInsideOBB(point);
    case Type::INVALID:
    default:
      throw std::runtime_error("not implemented for current bounding box type");
  }
}

float BoundingBox::volume() const {
  switch (type) {
    case Type::AABB:
    case Type::OBB:
    case Type::RAABB:
      return std::abs((max - min).prod());
    default:
    case Type::INVALID:
      return std::numeric_limits<float>::quiet_NaN();
  }
}

}  // namespace kimera
