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
#include "spark_dsg/attributes/object_node_attributes.h"

#include "attribute_utilities.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"

namespace spark_dsg {

namespace {

template <typename T>
std::string showIterable(const T& iterable, size_t max_length = 80) {
  std::stringstream ss;
  ss << "[";
  auto iter = iterable.begin();
  while (iter != iterable.end()) {
    ss << *iter;

    ++iter;
    if (iter != iterable.end()) {
      ss << ", ";
    }

    if (max_length && ss.str().size() >= max_length) {
      ss << "...";
      break;
    }
  }

  ss << "]";

  return ss.str();
}

}  // namespace

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;
using attributes_detail::quaternionsEqual;
using attributes_detail::quatToString;

const RegistrationInfo& ObjectNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("ObjectNodeAttributes");
  return info;
}

ObjectNodeAttributes::ObjectNodeAttributes()
    : SemanticNodeAttributes(),
      registered(false),
      world_R_object(Eigen::Quaterniond::Identity()) {}

NodeAttributes::Ptr ObjectNodeAttributes::clone() const {
  return std::make_unique<ObjectNodeAttributes>(*this);
}

void ObjectNodeAttributes::transform(const Eigen::Isometry3d& transform) {
  SemanticNodeAttributes::transform(transform);
  world_R_object =
      Eigen::Quaterniond(transform.linear() * world_R_object.toRotationMatrix());
}

std::ostream& ObjectNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - mesh_connections: " << showIterable(mesh_connections);
  out << "\n  - registered?: " << (registered ? "yes" : "no");
  out << "\n  - world_R_object: " << quatToString(world_R_object);
  return out;
}

void ObjectNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("mesh_connections", mesh_connections);
  serialization::field("registered", registered);
  serialization::field("world_R_object", world_R_object);
}

bool ObjectNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const ObjectNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return mesh_connections == derived->mesh_connections &&
         registered == derived->registered &&
         quaternionsEqual(world_R_object, derived->world_R_object);
}

}  // namespace spark_dsg
