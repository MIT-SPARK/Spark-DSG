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
#include "spark_dsg/attributes/semantic_node_attributes.h"

#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;

const RegistrationInfo& SemanticNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("SemanticNodeAttributes");
  return info;
}

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(), name(""), semantic_label(NO_SEMANTIC_LABEL) {}

NodeAttributes::Ptr SemanticNodeAttributes::clone() const {
  return std::make_unique<SemanticNodeAttributes>(*this);
}

void SemanticNodeAttributes::transform(const Eigen::Isometry3d& transform) {
  NodeAttributes::transform(transform);
  bounding_box.transform(transform);
}

bool SemanticNodeAttributes::hasLabel() const {
  return semantic_label != NO_SEMANTIC_LABEL;
}

bool SemanticNodeAttributes::hasFeature() const {
  return semantic_feature.rows() * semantic_feature.cols() != 0;
}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << "\n  - color: " << color << "\n"
      << "  - name: '" << name << "'\n"
      << "  - bounding box: " << bounding_box << "\n"
      << "  - label: " << std::to_string(semantic_label) << "\n"
      << "  - feature: [" << semantic_feature.rows() << " x " << semantic_feature.cols()
      << "]\n"
      << "  - concentration: [" << feature_concentration.rows() << " x "
      << feature_concentration.cols() << "]";
  return out;
}

void SemanticNodeAttributes::serialization_info() {
  NodeAttributes::serialization_info();
  serialization::field("name", name);
  serialization::field("color", color);

  const auto& version = io::GlobalInfo::loadedVersion();
  serialization::field("bounding_box", bounding_box);
  serialization::field("semantic_label", semantic_label);
  if (version <= io::Version(1, 1, 6)) {
    io::GlobalInfo::warnOutdated();

    Eigen::MatrixXf feature;
    serialization::field("semantic_feature", feature);
    if (feature.size()) {
      // this will lose information technically
      semantic_feature = feature.col(0);
    }
  } else {
    serialization::field("semantic_feature", semantic_feature);
  }

  if (version >= io::Version(1, 1, 7)) {
    serialization::field("feature_concentration", feature_concentration);
  }

  if (version >= io::Version(1, 1, 4)) {
    serialization::field("label_weights", label_weights);
  }
}

bool SemanticNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const SemanticNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!NodeAttributes::is_equal(other)) {
    return false;
  }

  return name == derived->name && color == derived->color &&
         bounding_box == derived->bounding_box &&
         semantic_label == derived->semantic_label &&
         semantic_feature == derived->semantic_feature &&
         feature_concentration == derived->feature_concentration;
}

}  // namespace spark_dsg
