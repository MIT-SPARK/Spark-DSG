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
#include "spark_dsg/attributes/agent_node_attributes.h"

#include "attribute_utilities.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"

namespace spark_dsg {

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;
using attributes_detail::quaternionsEqual;
using attributes_detail::quatToString;

const RegistrationInfo& AgentNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("AgentNodeAttributes");
  return info;
}

AgentNodeAttributes::AgentNodeAttributes() : NodeAttributes(), timestamp(0) {}

AgentNodeAttributes::AgentNodeAttributes(std::chrono::nanoseconds timestamp,
                                         const Eigen::Quaterniond& world_R_body,
                                         const Eigen::Vector3d& world_P_body,
                                         NodeId external_key)
    : NodeAttributes(world_P_body),
      timestamp(timestamp),
      world_R_body(world_R_body),
      external_key(external_key) {}

NodeAttributes::Ptr AgentNodeAttributes::clone() const {
  return std::make_unique<AgentNodeAttributes>(*this);
}

void AgentNodeAttributes::transform(const Eigen::Isometry3d& transform) {
  NodeAttributes::transform(transform);
  world_R_body = transform.linear() * world_R_body;
}

std::ostream& AgentNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << "\n  - orientation: " << quatToString(world_R_body);
  out << "\n  - observed_semantic_labels.size(): " << observed_semantic_labels.size();
  return out;
}

void AgentNodeAttributes::serialization_info() {
  NodeAttributes::serialization_info();
  serialization::field("timestamp", timestamp);
  serialization::field("world_R_body", world_R_body);
  serialization::field("external_key", external_key);
  serialization::field("dbow_ids", dbow_ids);
  serialization::field("dbow_values", dbow_values);
  serialization::field("observed_semantic_labels", observed_semantic_labels);
}

bool AgentNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const AgentNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!NodeAttributes::is_equal(other)) {
    return false;
  }

  return timestamp == derived->timestamp &&
         quaternionsEqual(world_R_body, derived->world_R_body) &&
         external_key == derived->external_key && dbow_ids == derived->dbow_ids &&
         dbow_values == derived->dbow_values &&
         observed_semantic_labels == derived->observed_semantic_labels;
}

}  // namespace spark_dsg
