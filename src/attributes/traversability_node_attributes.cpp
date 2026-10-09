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
#include "spark_dsg/attributes/traversability_node_attributes.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;

const RegistrationInfo& TraversabilityNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("TraversabilityNodeAttributes");
  return info;
}

const RegistrationInfo& TravNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("TravNodeAttributes");
  return info;
}

bool BoundaryInfo::operator==(const BoundaryInfo& other) const {
  return min == other.min && max == other.max && states == other.states;
}

NodeAttributes::Ptr TraversabilityNodeAttributes::clone() const {
  return std::make_unique<TraversabilityNodeAttributes>(*this);
}

std::ostream& TraversabilityNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - min: " << boundary.min.transpose() << "\n"
      << "  - max: " << boundary.max.transpose() << "\n"
      << "  - first_observed_ns: " << first_observed_ns << "\n"
      << "  - last_observed_ns: " << last_observed_ns << "\n"
      << "  - distance: " << distance << "\n";
  return out;
}

void TraversabilityNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("first_observed_ns", first_observed_ns);
  serialization::field("last_observed_ns", last_observed_ns);
  serialization::field("distance", distance);
  serialization::field("min", boundary.min);
  serialization::field("max", boundary.max);

  // Workaround for state serialization.
  for (size_t i = 0; i < 4; ++i) {
    std::vector<uint8_t> s;
    s.reserve(boundary.states[i].size());
    for (const auto& state : boundary.states[i]) {
      s.push_back(static_cast<uint8_t>(state));
    }

    serialization::field("states_" + std::to_string(i), s);
    boundary.states[i].clear();
    boundary.states[i].reserve(s.size());
    for (const auto& state : s) {
      boundary.states[i].push_back(static_cast<TraversabilityState>(state));
    }
  }

  const auto& version = io::GlobalInfo::loadedVersion();
  if (version < io::Version(1, 1, 4)) {
    io::GlobalInfo::warnOutdated();
    if (version == io::Version(1, 1, 3)) {
      // Backwards compatibility for cognition labels.
      std::map<int, float> temp;
      serialization::field("cognition_labels", temp);
      label_weights.clear();
      for (const auto& [label, weight] : temp) {
        label_weights[static_cast<Label>(label)] = weight;
      }
    }
  }
}

bool TraversabilityNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const TraversabilityNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!NodeAttributes::is_equal(other)) {
    return false;
  }

  return boundary == derived->boundary && distance == derived->distance &&
         first_observed_ns == derived->first_observed_ns &&
         last_observed_ns == derived->last_observed_ns;
}

NodeAttributes::Ptr TravNodeAttributes::clone() const {
  return std::make_unique<TravNodeAttributes>(*this);
}

std::ostream& TravNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - first_observed_ns: " << first_observed_ns << "\n"
      << "  - last_observed_ns: " << last_observed_ns << "\n"
      << "  - num states: " << states.size() << "\n"
      << "  - num radii: " << radii.size() << "\n"
      << "  - min radius: " << min_radius << "\n"
      << "  - max radius: " << max_radius;
  return out;
}

void TravNodeAttributes::serialization_info() {
  const auto& version = io::GlobalInfo::loadedVersion();
  if (version <= io::Version(1, 1, 5)) {
    io::GlobalInfo::warnOutdated();
    NodeAttributes::serialization_info();
  } else {
    SemanticNodeAttributes::serialization_info();
  }

  serialization::field("first_observed_ns", first_observed_ns);
  serialization::field("last_observed_ns", last_observed_ns);
  serialization::field("radii", radii);
  serialization::field("min_radius", min_radius);
  serialization::field("max_radius", max_radius);

  // Workaround for state serialization.
  std::vector<uint8_t> s;
  s.reserve(states.size());
  for (const auto& state : states) {
    s.push_back(static_cast<uint8_t>(state));
  }

  serialization::field("states", s);
  states.clear();
  states.reserve(s.size());
  for (const auto& state : s) {
    states.push_back(static_cast<TraversabilityState>(state));
  }
}

bool TravNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const TravNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return states == derived->states && radii == derived->radii &&
         min_radius == derived->min_radius && max_radius == derived->max_radius &&
         first_observed_ns == derived->first_observed_ns &&
         last_observed_ns == derived->last_observed_ns;
}

void TravNodeAttributes::fromExteriorPoints(
    const std::vector<Eigen::Vector3d>& points_W,
    const TraversabilityStates& states_in) {
  radii = std::vector<double>(radii.size(), std::numeric_limits<double>::max());
  states = TraversabilityStates(radii.size(), TraversabilityState::INTRAVERSABLE);
  min_radius = std::numeric_limits<double>::max();
  max_radius = 0.0;

  // Update all bins with points.
  for (size_t i = 0; i < points_W.size(); ++i) {
    const Eigen::Vector3d point_L = points_W[i] - position;
    const double distance = point_L.norm();
    min_radius = std::min(min_radius, distance);
    max_radius = std::max(max_radius, distance);
    const size_t bin = getBin(point_L);

    radii[bin] = std::min(radii[bin], distance);
    if (i < states_in.size()) {
      // Need separate implementation of fusion for agglomeration of states.
      if (states_in[i] == TraversabilityState::TRAVERSABLE) {
        states[bin] = TraversabilityState::TRAVERSABLE;
      } else if (states_in[i] == TraversabilityState::UNKNOWN &&
                 states[bin] != TraversabilityState::UNKNOWN) {
        states[bin] = TraversabilityState::UNKNOWN;
      }
    }
  }

  // Fill in empty bins.
  for (size_t i = 0; i < radii.size(); ++i) {
    if (radii[i] == std::numeric_limits<double>::max()) {
      radii[i] = min_radius;
      states[i] = TraversabilityState::UNKNOWN;
    }
  }
}

void TravNodeAttributes::clear() {
  radii.clear();
  states.clear();
  min_radius = 0.0;
  max_radius = 0.0;
}

double TravNodeAttributes::getBinPercentage(const Eigen::Vector3d& point_L) const {
  const double angle = std::atan2(point_L.y(), point_L.x()) / (2.0 * std::numbers::pi);
  return angle >= 0.0 ? angle : angle + 1.0;
}

size_t TravNodeAttributes::getBin(const Eigen::Vector3d& point_L) const {
  return static_cast<size_t>(getBinPercentage(point_L) * radii.size());
}

bool TravNodeAttributes::contains(const Eigen::Vector3d& point_W) const {
  const Eigen::Vector3d p_L = point_W - position;  // local frame
  const double distance = p_L.norm();
  if (distance < min_radius) {
    return true;
  }

  if (distance > max_radius) {
    return false;
  }

  // Check detailed by interpolating the bin.
  const double bin = getBinPercentage(p_L);
  size_t bin_left = static_cast<size_t>(std::floor(bin * radii.size()));
  size_t bin_right = (bin_left + 1) % radii.size();
  double distance_max =
      radii[bin_left] + (radii[bin_right] - radii[bin_left]) *
                            (bin * radii.size() - static_cast<double>(bin_left));
  return distance <= distance_max;
}

Eigen::Vector3d TravNodeAttributes::getBoundaryPoint(size_t bin,
                                                     bool in_world_frame) const {
  const double angle = (static_cast<double>(bin) / static_cast<double>(radii.size())) *
                       2.0 * std::numbers::pi;
  const Eigen::Vector3d point_L(
      radii[bin] * std::cos(angle), radii[bin] * std::sin(angle), 0.0);
  if (in_world_frame) {
    return position + point_L;
  } else {
    return point_L;
  }
}

bool TravNodeAttributes::intersects(const TravNodeAttributes& other) const {
  const double distance = (other.position - position).norm();
  if (distance > (max_radius + other.max_radius)) {
    return false;
  }

  if (distance < (min_radius + other.min_radius)) {
    return true;
  }

  // Check detailed intersection.
  // TODO(lschmid): For now a simple approximation by checking the corner points only.
  for (size_t i = 0; i < other.radii.size(); ++i) {
    if (contains(other.getBoundaryPoint(i, true))) {
      return true;
    }
  }

  return false;
}

double TravNodeAttributes::area() const {
  double area = 0.0;
  const size_t N = radii.size();
  const double angle_increment =
      std::sin((2.0 * std::numbers::pi) / static_cast<double>(N));
  for (size_t i = 0; i < N; ++i) {
    const double r1 = radii[i];
    const double r2 = radii[(i + 1) % N];
    area += 0.5 * r1 * r2 * angle_increment;
  }

  return area;
}

}  // namespace spark_dsg
