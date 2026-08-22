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
#include "spark_dsg/scene_graph_node.h"

#include "spark_dsg/printing.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"

namespace spark_dsg {
namespace {

template <typename Derived>
bool matricesEqual(const Eigen::DenseBase<Derived>& lhs,
                   const Eigen::DenseBase<Derived>& rhs) {
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
    return false;
  }

  bool same = true;
  for (int r = 0; r < lhs.rows(); ++r) {
    for (int c = 0; c < lhs.cols(); ++c) {
      const auto lhs_nan = std::isnan(lhs(r, c));
      const auto rhs_nan = std::isnan(rhs(r, c));
      // if one value is nan, this still works
      same &= (lhs_nan && rhs_nan) || lhs(r, c) == rhs(r, c);
    }
  }

  return same;
}

}  // namespace

decltype(NodeAttributes::registration_) NodeAttributes::registration_ =
    NodeAttributeRegistration<NodeAttributes>("NodeAttributes");

std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs) {
  return attrs.fill_ostream(out);
}

NodeAttributes::NodeAttributes() : NodeAttributes(Eigen::Vector3d::Zero()) {}

NodeAttributes::NodeAttributes(const Eigen::Vector3d& pos)
    : position(pos), last_update_time_ns(0), is_active(false), is_predicted(false) {}

NodeAttributes::Ptr NodeAttributes::clone() const {
  return std::make_unique<NodeAttributes>(*this);
}

size_t NodeAttributes::memoryUsage() const {
  // By default simply dispatch serialization to estimate the attributes size. Not
  // perfect but should be ok.
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(*this);
  return buffer.size();
}

void NodeAttributes::transform(const Eigen::Isometry3d& transform) {
  position = transform * position;
}

bool NodeAttributes::operator==(const NodeAttributes& other) const {
  return is_equal(other);
}

const serialization::RegistrationInfo& NodeAttributes::registration() const {
  return registrationImpl();
}

std::ostream& NodeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  out << "  - position: " << position.transpose().format(format) << "\n";
  out << "  - last update time: "
      << (last_update_time_ns == 0 ? "n/a" : std::to_string(last_update_time_ns))
      << "\n";
  out << std::boolalpha << "  - is_active: " << is_active << "\n";
  out << std::boolalpha << "  - is_predicted: " << is_predicted;
  return out;
}

void NodeAttributes::serialization_info() {
  serialization::field("position", position);
  serialization::field("last_update_time_ns", last_update_time_ns);
  serialization::field("is_active", is_active);
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 0, 4)) {
    io::warnOutdatedHeader(header);
  } else {
    serialization::field("is_predicted", is_predicted);
  }
}

void NodeAttributes::serialization_info() const {
  const_cast<NodeAttributes*>(this)->serialization_info();
}

bool NodeAttributes::is_equal(const NodeAttributes& other) const {
  return matricesEqual(position, other.position) &&
         last_update_time_ns == other.last_update_time_ns &&
         is_active == other.is_active && is_predicted == other.is_predicted;
}

const serialization::RegistrationInfo& NodeAttributes::registrationImpl() const {
  return registration_.info;
}

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerKey layer_id,
                               std::unique_ptr<NodeAttributes>&& attrs)
    : id(node_id), layer(layer_id), attributes_(std::move(attrs)) {}

SceneGraphNode::~SceneGraphNode() = default;

bool SceneGraphNode::hasParent() const { return parents_.size() == 1; }

bool SceneGraphNode::hasSiblings() const { return !siblings_.empty(); }

bool SceneGraphNode::hasChildren() const { return !children_.empty(); }

std::optional<NodeId> SceneGraphNode::getParent() const {
  if (parents_.size() != 1) {
    return std::nullopt;
  }

  return *parents_.begin();
}

const std::set<NodeId>& SceneGraphNode::siblings() const { return siblings_; };

const std::set<NodeId>& SceneGraphNode::children() const { return children_; };

const std::set<NodeId>& SceneGraphNode::parents() const { return parents_; };

std::vector<NodeId> SceneGraphNode::connections() const {
  // TODO(nathan) this would be better as a custom iterator
  std::vector<NodeId> to_return;
  to_return.insert(to_return.end(), siblings_.begin(), siblings_.end());
  to_return.insert(to_return.end(), children_.begin(), children_.end());
  to_return.insert(to_return.end(), parents_.begin(), parents_.end());
  return to_return;
}

void SceneGraphNode::setAttributes(std::unique_ptr<NodeAttributes>&& attrs) {
  attributes_ = std::move(attrs);
}

size_t SceneGraphNode::memoryUsage() const {
  size_t total_size = sizeof(SceneGraphNode);
  // Attributes size.
  if (attributes_) {
    total_size += attributes_->memoryUsage();
  }

  // Connected nodes size.
  total_size += siblings_.size() * sizeof(NodeId);
  total_size += children_.size() * sizeof(NodeId);
  total_size += parents_.size() * sizeof(NodeId);

  return total_size;
}

}  // namespace spark_dsg
