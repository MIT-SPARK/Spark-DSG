#include "kimera_dsg/bounding_box.h"

namespace kimera {

std::ostream& operator<<(std::ostream& out, const Eigen::Quaternionf& q) {
  out << q.w() << " + " << q.x() << "i + " << q.y() << "j + " << q.z() << "k";
  return out;
}

BoundingBox::BoundingBox() : type(Type::INVALID) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max)
    : type(Type::AABB), min(min), max(max) {}

BoundingBox::BoundingBox(const Eigen::Vector3f& min,
                         const Eigen::Vector3f& max,
                         const Eigen::Vector3f world_P_center,
                         const Eigen::Quaternionf& world_R_center)
    : type(Type::OBB),
      min(min),
      max(max),
      world_P_center(world_P_center),
      world_R_center(world_R_center) {}

std::ostream& operator<<(std::ostream& os, const BoundingBox& box) {
  os << "Bounding box: " << std::endl
     << " - Max: " << box.max.transpose() << std::endl
     << " - Min: " << box.min.transpose() << std::endl;
  if (box.type == BoundingBox::Type::OBB) {
    os << " - Position: " << box.world_P_center.transpose() << std::endl
       << " - Orientation: " << box.world_R_center << std::endl;
  }
  return os;
};

}  // namespace kimera
