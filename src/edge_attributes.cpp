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
#include "spark_dsg/edge_attributes.h"

#include <iomanip>

#include "spark_dsg/printing.h"
#include "spark_dsg/serialization/attribute_serialization.h"

namespace spark_dsg {

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

EdgeAttributes::EdgeAttributes() : weighted(false), weight(1.0) {}

EdgeAttributes::EdgeAttributes(double weight) : weighted(true), weight(weight) {}

EdgeAttributes::~EdgeAttributes() = default;

EdgeAttributes::Ptr EdgeAttributes::clone() const {
  return std::make_unique<EdgeAttributes>(*this);
}

std::ostream& operator<<(std::ostream& out, const EdgeAttributes& attrs) {
  out << "{";
  attrs.fill_ostream(out);
  out << "}";
  return out;
}

bool EdgeAttributes::operator==(const EdgeAttributes& other) const {
  return is_equal(other);
}

void EdgeAttributes::fill_ostream(std::ostream& out) const {
  out << std::boolalpha << "weighted: " << weighted << ", weight: " << weight;
}

void EdgeAttributes::serialization_info() {
  serialization::field("weighted", weighted);
  serialization::field("weight", weight);
}

void EdgeAttributes::serialization_info() const {
  const_cast<EdgeAttributes*>(this)->serialization_info();
}

bool EdgeAttributes::is_equal(const EdgeAttributes& other) const {
  return weighted == other.weighted && weight == other.weight;
}

SpatialEdgeAttributes::SpatialEdgeAttributes()
    : EdgeAttributes(), type(SpatialEdgeAttributes::Type::UNKNOWN) {}

EdgeAttributes::Ptr SpatialEdgeAttributes::clone() const {
  return std::make_unique<SpatialEdgeAttributes>(*this);
}

void SpatialEdgeAttributes::fill_ostream(std::ostream& out) const {
  EdgeAttributes::fill_ostream(out);
  out << "\n  - type: " << static_cast<int>(type) << "\n"
      << "]";
}

void SpatialEdgeAttributes::serialization_info() {
  EdgeAttributes::serialization_info();
  int type_val = static_cast<int>(type);
  serialization::field("type", type_val);
}

bool SpatialEdgeAttributes::is_equal(const EdgeAttributes& other) const {
  const auto derived = dynamic_cast<const SpatialEdgeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!EdgeAttributes::is_equal(other)) {
    return false;
  }

  return type == derived->type;
}

ArticulateEdgeAttributes::ArticulateEdgeAttributes()
    : EdgeAttributes(), type(ArticulateEdgeAttributes::Type::UNKNOWN) {}

EdgeAttributes::Ptr ArticulateEdgeAttributes::clone() const {
  return std::make_unique<ArticulateEdgeAttributes>(*this);
}

void ArticulateEdgeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  EdgeAttributes::fill_ostream(out);
  out << "\n  - type: " << static_cast<int>(type) << "\n"
      << "\n  - axis: " << axis.transpose().format(format) << "\n"
      << "]";
}

void ArticulateEdgeAttributes::serialization_info() {
  EdgeAttributes::serialization_info();
  int type_val = static_cast<int>(type);
  serialization::field("type", type_val);
}

bool ArticulateEdgeAttributes::is_equal(const EdgeAttributes& other) const {
  const auto derived = dynamic_cast<const ArticulateEdgeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!EdgeAttributes::is_equal(other)) {
    return false;
  }

  return type == derived->type && matricesEqual(axis, derived->axis);
}
}  // namespace spark_dsg
