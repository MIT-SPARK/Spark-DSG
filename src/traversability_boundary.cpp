#include "spark_dsg/traversability_boundary.h"

namespace spark_dsg {

bool traversable(TraversabilityState state, bool optimistic) {
  return state == TraversabilityState::TRAVERSABLE ||
         state == TraversabilityState::TRAVERSED ||
         (optimistic && state == TraversabilityState::UNKNOWN);
}

bool intraversable(TraversabilityState state, bool optimistic) {
  return state == TraversabilityState::INTRAVERSABLE ||
         (optimistic && state == TraversabilityState::UNKNOWN);
}

void fuseStates(const TraversabilityState& from, TraversabilityState& to) {
  if (from == TraversabilityState::TRAVERSED) {
    to = TraversabilityState::TRAVERSED;
    return;
  }
  if (from == TraversabilityState::INTRAVERSABLE &&
      to != TraversabilityState::TRAVERSED) {
    to = TraversabilityState::INTRAVERSABLE;
    return;
  }
  if (from == TraversabilityState::UNKNOWN && to != TraversabilityState::TRAVERSED &&
      to != TraversabilityState::INTRAVERSABLE) {
    to = TraversabilityState::UNKNOWN;
  }
}

void fuseStates(const std::vector<TraversabilityState>& from,
                std::vector<TraversabilityState>& to) {
  if (to.size() < from.size()) {
    to.resize(from.size());
  }
  for (size_t i = 0; i < from.size(); ++i) {
    fuseStates(from[i], to[i]);
  }
}

void fuseStates(const TraversabilityState& from, std::vector<TraversabilityState>& to) {
  for (auto& state : to) {
    fuseStates(from, state);
  }
}

std::pair<size_t, size_t> computeMinMaxTraversability(
    const std::vector<TraversabilityState>& states) {
  // Find the minimum and maximum traversable width.
  size_t min_traversable = 0;
  size_t max_traversable = 0;
  size_t current_traversable = 0;
  size_t current_unknown = 0;
  for (const auto& state : states) {
    if (state == TraversabilityState::INTRAVERSABLE) {
      current_traversable = 0;
      current_unknown = 0;
    } else if (state == TraversabilityState::UNKNOWN) {
      current_unknown++;
      current_traversable = 0;
    } else {
      current_traversable++;
      current_unknown++;
    }
    min_traversable = std::max(min_traversable, current_traversable);
    max_traversable = std::max(max_traversable, current_unknown);
  }
  return {min_traversable, max_traversable};
}

const std::array<Side, 4> Side::ALL = {
    Side(Side::BOTTOM), Side(Side::LEFT), Side(Side::TOP), Side(Side::RIGHT)};

Boundary::Boundary(const TraversabilityNodeAttributes& attrs)
    : Boundary(attrs.boundary, attrs.position) {}

Boundary::Boundary(const BoundaryInfo& info, const Eigen::Vector3d& position)
    : min(info.min + position.head<2>()),
      max(info.max + position.head<2>()),
      states(info.states) {}

Boundary::Boundary(const Eigen::Vector2d& min,
                   const Eigen::Vector2d& max,
                   const std::array<std::vector<TraversabilityState>, 4>& states)
    : min(min), max(max), states(states) {}

bool Boundary::contains(const Eigen::Vector2d& point) const {
  return (point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() &&
          point.y() <= max.y());
}

bool Boundary::intersects(const Boundary& other) const {
  return (min.x() < other.max.x() && max.x() > other.min.x() &&
          min.y() < other.max.y() && max.y() > other.min.y());
}

bool Boundary::valid() const { return (min.x() < max.x() && min.y() < max.y()); }

double Boundary::width() const { return max.x() - min.x(); }

double Boundary::height() const { return max.y() - min.y(); }

double Boundary::area() const { return width() * height(); }

Eigen::Vector2d Boundary::center() const { return (min + max) / 2.0; }

void Boundary::toAttributes(TraversabilityNodeAttributes& attrs) const {
  const Eigen::Vector2d c = center();
  attrs.position.head(2) = c;
  attrs.boundary.min = min - c;
  attrs.boundary.max = max - c;
}

double Boundary::getCoordinate(Side side) const {
  switch (side) {
    case Side::BOTTOM:
      return min.y();
    case Side::LEFT:
      return min.x();
    case Side::TOP:
      return max.y();
    case Side::RIGHT:
      return max.x();
    default:
      throw std::out_of_range("Invalid side index for Boundary.");
  }
}

void Boundary::setCoordinate(Side side, double coordinate) {
  switch (side) {
    case Side::BOTTOM:
      min.y() = coordinate;
      break;
    case Side::LEFT:
      min.x() = coordinate;
      break;
    case Side::TOP:
      max.y() = coordinate;
      break;
    case Side::RIGHT:
      max.x() = coordinate;
      break;
    default:
      throw std::out_of_range("Invalid side index for Boundary.");
  }
}

bool Boundary::containsOtherBoxWidth(const Boundary& other, bool vertical) const {
  if (vertical) {
    return (min.y() <= other.min.y() && max.y() >= other.max.y());
  } else {
    return (min.x() <= other.min.x() && max.x() >= other.max.x());
  }
}

Boundary Boundary::intersection(const Boundary& other) const {
  return Boundary(min.cwiseMax(other.min), max.cwiseMin(other.max));
}

double Boundary::xIntersection(const Boundary& other) const {
  if (min.x() > other.max.x() || max.x() < other.min.x()) {
    return 0.0;  // No intersection
  }
  return std::min(max.x(), other.max.x()) - std::max(min.x(), other.min.x());
}

double Boundary::yIntersection(const Boundary& other) const {
  if (min.y() > other.max.y() || max.y() < other.min.y()) {
    return 0.0;  // No intersection
  }
  return std::min(max.y(), other.max.y()) - std::max(min.y(), other.min.y());
}

double Boundary::xDistance(const Boundary& other) const {
  if (max.x() < other.min.x()) {
    return other.min.x() - max.x();  // Distance to the right
  } else if (min.x() > other.max.x()) {
    return other.max.x() - min.x();  // Distance to the left is negative.
  }
  return 0.0;  // Overlapping in x-axis
}

double Boundary::yDistance(const Boundary& other) const {
  if (max.y() < other.min.y()) {
    return other.min.y() - max.y();  // Distance to the top
  } else if (min.y() > other.max.y()) {
    return other.max.y() - min.y();  // Distance to the bottom is negative.
  }
  return 0.0;  // Overlapping in y-axis
}

double Boundary::distanceToSide(const Side side, const Eigen::Vector2d& point) const {
  // Considers the side as finite line.
  switch (side) {
    case Side::BOTTOM:
      if (point.x() < min.x()) return (point - min).norm();
      if (point.x() > max.x()) return (point - max).norm();
      return std::abs(point.y() - min.y());
    case Side::LEFT:
      if (point.y() < min.y()) return (point - min).norm();
      if (point.y() > max.y()) return (point - max).norm();
      return std::abs(point.x() - min.x());
    case Side::TOP:
      if (point.x() < min.x()) return (point - min).norm();
      if (point.x() > max.x()) return (point - max).norm();
      return std::abs(point.y() - max.y());
    case Side::RIGHT:
      if (point.y() < min.y()) return (point - min).norm();
      if (point.y() > max.y()) return (point - max).norm();
      return std::abs(point.x() - max.x());
    default:
      throw std::out_of_range("Invalid side index for Boundary.");
  }
}

Side Boundary::lineIntersectsSide(const Eigen::Vector2d& source) const {
  if (contains(source)) {
    return Side::INVALID;
  }
  const Eigen::Vector2d query = source - center();            // Local coordinates
  const Eigen::Vector2d corner = max - center();              // Local coordinates
  const double theta_q = std::atan2(query.y(), query.x());    // [-PI, PI]
  const double theta_0 = std::atan2(corner.y(), corner.x());  // Top-right
  if (std::abs(theta_q) <= theta_0) {
    return Side::RIGHT;
  }
  if (std::abs(theta_q) >= M_PI - theta_0) {
    return Side::LEFT;
  }
  if (theta_q > 0) {
    return Side::TOP;
  } else {
    return Side::BOTTOM;
  }
}

Side Boundary::lineIntersectsSide(const Eigen::Vector3d& source) const {
  return lineIntersectsSide(source.head<2>().eval());
}

Boundary::BoundarySide Boundary::side(Side side) {
  if (side == Side::INVALID) {
    throw std::out_of_range("Invalid side index for BoundarySide.");
  }
  if (side.horizontal()) {
    return BoundarySide(min.x(), max.x(), states[side]);
  }
  return BoundarySide(min.y(), max.y(), states[side]);
}

const Boundary::BoundarySide Boundary::side(Side side) const {
  return const_cast<Boundary*>(this)->side(side);
}

double Boundary::BoundarySide::maxTraversableDistance(const BoundarySide& other) const {
  if (min > other.max || max < other.min || states.empty() || other.states.empty()) {
    return 0.0;  // No overlap
  }

  // Single boundary cases.
  if (states.size() == 1 && (states[0] == TraversabilityState::UNKNOWN ||
                             states[0] == TraversabilityState::INTRAVERSABLE)) {
    return 0.0;
  }
  if (other.states.size() == 1 &&
      (other.states[0] == TraversabilityState::UNKNOWN ||
       other.states[0] == TraversabilityState::INTRAVERSABLE)) {
    return 0.0;
  }
  const double start = std::max(min, other.min);
  const double end = std::min(max, other.max);
  if (states.size() == 1 && other.states.size() == 1) {
    return end - start;
  }

  // Multiple states: Interpolate and count the voxels.
  std::vector<TraversabilityState> states_in_range;
  if (states.size() == 1) {
    states_in_range = other.getStates(start, end);
  } else if (other.states.size() == 1) {
    states_in_range = getStates(start, end);
  } else {
    // NOTE(lschmid): This assumes equal voxel spacing.
    states_in_range = getStates(start, end);
    const auto other_states = other.getStates(start, end);
    fuseStates(other_states, states_in_range);
  }

  return computeMinMaxTraversability(states_in_range).first * voxelSize();
}

TraversabilityState Boundary::BoundarySide::getState(double coordinate) const {
  if (states.size() == 0) {
    return TraversabilityState::UNKNOWN;
  }
  size_t idx = static_cast<size_t>((coordinate - min) / (max - min) * states.size());
  return states[idx];
}

std::vector<TraversabilityState> Boundary::BoundarySide::getStates(double from,
                                                                   double to) const {
  const size_t start = std::max(
      0, static_cast<int>(std::floor((from - min) / (max - min) * states.size())));
  const size_t end = std::min(
      states.size(),
      static_cast<size_t>(std::ceil((to - min) / (max - min) * states.size())));
  std::vector<TraversabilityState> result(end - start);
  for (size_t i = start; i < end; ++i) {
    result[i - start] = states[i];
  }
  return result;
}

}  // namespace spark_dsg
