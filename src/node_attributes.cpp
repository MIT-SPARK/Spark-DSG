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
#include "spark_dsg/node_attributes.h"

namespace spark_dsg {

NodeAttributes::NodeAttributes() : NodeAttributes(Eigen::Vector3d::Zero()) {}

NodeAttributes::NodeAttributes(const Eigen::Vector3d& pos)
    : position(pos), last_update_time_ns(0), is_active(false) {}

std::ostream& NodeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  out << "  - position: " << position.transpose().format(format) << std::endl;
  if (last_update_time_ns != 0) {
    out << "  - last update time: " << last_update_time_ns << " [ns]" << std::endl;
  }
  out << std::boolalpha << " - is_active: " << is_active;
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
    : SemanticNodeAttributes(),
      registered(false),
      world_R_object(Eigen::Quaterniond::Identity()) {}

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

}  // namespace spark_dsg
