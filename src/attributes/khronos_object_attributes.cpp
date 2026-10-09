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
#include "spark_dsg/attributes/khronos_object_attributes.h"

#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"

namespace spark_dsg {

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;

const RegistrationInfo& KhronosObjectAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("KhronosObjectAttributes");
  return info;
}

KhronosObjectAttributes::KhronosObjectAttributes() : mesh(true, false, false) {}

NodeAttributes::Ptr KhronosObjectAttributes::clone() const {
  return std::make_unique<KhronosObjectAttributes>(*this);
}

std::ostream& KhronosObjectAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - first_observed_ns: ";
  for (uint64_t t : first_observed_ns) {
    out << t << " ";
  }

  out << "\n  - last_observed_ns: ";
  for (uint64_t t : last_observed_ns) {
    out << t << " ";
  }

  out << "\n  - mesh: " << mesh.numVertices() << " vertices, " << mesh.numFaces()
      << " faces";
  return out;
}

void KhronosObjectAttributes::serialization_info() {
  ObjectNodeAttributes::serialization_info();
  serialization::field("first_observed_ns", first_observed_ns);
  serialization::field("last_observed_ns", last_observed_ns);
  serialization::field("trajectory_positions", trajectory_positions);
  serialization::field("trajectory_timestamps", trajectory_timestamps);
  serialization::field("dynamic_object_points", dynamic_object_points);
  serialization::field("details", details);
  serialization::field("mesh", mesh);
}

bool KhronosObjectAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const KhronosObjectAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!ObjectNodeAttributes::is_equal(other)) {
    return false;
  }

  return first_observed_ns == derived->first_observed_ns &&
         last_observed_ns == derived->last_observed_ns && mesh == derived->mesh &&
         trajectory_positions == derived->trajectory_positions &&
         dynamic_object_points == derived->dynamic_object_points &&
         details == derived->details;
}

}  // namespace spark_dsg
