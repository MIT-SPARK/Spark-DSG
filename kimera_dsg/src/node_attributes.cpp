#include "kimera_dsg/node_attributes.h"

namespace kimera {

// TODO(nathan) output stream operators are ugly

NodeAttributes::NodeAttributes() : position(Eigen::Vector3d::Zero()) {}

NodeAttributes::NodeAttributes(const Eigen::Vector3d& pos) : position(pos) {}

std::ostream& NodeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  out << "  - position: " << position.transpose().format(format) << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs) {
  return attrs.fill_ostream(out);
}

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(), name(""), color(ColorVector::Zero()), semantic_label(0u) {}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  NodeAttributes::fill_ostream(out);
  out << "  - color: " << color.cast<int>().transpose().format(format) << std::endl
      << "  - name: " << name << std::endl
      << "  - bounding box: " << bounding_box << std::endl
      << "  - label: " << std::to_string(semantic_label) << std::endl;
  return out;
}

ObjectNodeAttributes::ObjectNodeAttributes()
    : SemanticNodeAttributes(), world_R_object(Eigen::Quaterniond::Identity()) {}

std::ostream& ObjectNodeAttributes::fill_ostream(std::ostream& out) const {
  // TODO(nathan) think about printing out rotation here
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - registered?: " << (registered ? "yes" : "no") << std::endl;
  return out;
}

RoomNodeAttributes::RoomNodeAttributes() : SemanticNodeAttributes() {}

std::ostream& RoomNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  return out;
}

PlaceNodeAttributes::PlaceNodeAttributes()
    : SemanticNodeAttributes(), distance(0.0), num_basis_points(0) {}

PlaceNodeAttributes::PlaceNodeAttributes(double distance, unsigned int num_basis_points)
    : SemanticNodeAttributes(),
      distance(distance),
      num_basis_points(num_basis_points) {}

std::ostream& PlaceNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - distance: " << distance << std::endl;
  out << "  - num_basis_points: " << num_basis_points << std::endl;
  out << "  - is_active: " << is_active << std::endl;
  return out;
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
  out << "  - orientation: " << world_R_body << std::endl;
  return out;
}

}  // namespace kimera
