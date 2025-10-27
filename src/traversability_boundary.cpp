#include "spark_dsg/traversability_boundary.h"

#include <cmath>
#include <numbers>

namespace spark_dsg {

bool isTraversable(TraversabilityState state, bool optimistic) {
  return state == TraversabilityState::TRAVERSABLE ||
         state == TraversabilityState::TRAVERSED ||
         (optimistic && state == TraversabilityState::UNKNOWN);
}

bool areAllTraversable(const TraversabilityStates& states, bool optimistic) {
  for (const auto& state : states) {
    if (!isTraversable(state, optimistic)) {
      return false;
    }
  }
  return true;
}

void fuseStates(const TraversabilityState from,
                TraversabilityState& to,
                bool optimistic) {
  if (from == TraversabilityState::TRAVERSED || to == TraversabilityState::TRAVERSED) {
    to = TraversabilityState::TRAVERSED;
    return;
  }
  if (from == TraversabilityState::INTRAVERSABLE ||
      to == TraversabilityState::INTRAVERSABLE) {
    to = TraversabilityState::INTRAVERSABLE;
    return;
  }
  if (optimistic) {
    if (from == TraversabilityState::TRAVERSABLE) {
      to = TraversabilityState::TRAVERSABLE;
    }
  } else if (from == TraversabilityState::UNKNOWN) {
    to = TraversabilityState::UNKNOWN;
  }
}

void fuseStates(const TraversabilityStates& from,
                TraversabilityStates& to,
                bool optimistic) {
  if (to.size() < from.size()) {
    to.resize(from.size());
  }
  for (size_t i = 0; i < from.size(); ++i) {
    fuseStates(from[i], to[i], optimistic);
  }
}

void fuseStates(const TraversabilityState from,
                TraversabilityStates& to,
                bool optimistic) {
  for (auto& state : to) {
    fuseStates(from, state, optimistic);
  }
}

std::pair<size_t, size_t> computeMinMaxTraversability(
    const TraversabilityStates& states) {
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

bool simplifyTraversabilityStates(TraversabilityStates& states) {
  if (states.size() <= 1) {
    return false;
  }

  for (size_t i = 1; i < states.size(); ++i) {
    if (states[i] != states[0]) {
      return false;
    }
  }
  states.resize(1);
  return true;
}

bool filterTraversabilityStates(TraversabilityStates& states,
                                size_t width,
                                bool optimistic) {
  if (width <= 1 || states == TraversabilityStates(
                                  states.size(), TraversabilityState::INTRAVERSABLE)) {
    return false;
  }

  if (states.size() < width) {
    states = TraversabilityStates(states.size(), TraversabilityState::INTRAVERSABLE);
    return true;
  }

  size_t steps_since_intraversable = 0;
  size_t steps_since_unknown = 0;
  TraversabilityStates new_states = states;
  for (size_t i = 0; i < new_states.size(); ++i) {
    if (new_states[i] == TraversabilityState::INTRAVERSABLE) {
      if (steps_since_intraversable < width && steps_since_intraversable > 0) {
        // Change the previous states to INTRAVERSABLE.
        for (size_t j = i - steps_since_intraversable; j < i; ++j) {
          fuseStates(TraversabilityState::INTRAVERSABLE, new_states[j]);
        }
      }
      steps_since_intraversable = 0;
      steps_since_unknown = 0;
    } else if (!optimistic && new_states[i] == TraversabilityState::UNKNOWN) {
      if (steps_since_unknown < width && steps_since_unknown > 0) {
        // Change the previous states to UNKNOWN.
        for (size_t j = i - steps_since_unknown; j < i; ++j) {
          fuseStates(TraversabilityState::UNKNOWN, new_states[j], false);
        }
      }
      steps_since_intraversable += 1;
      steps_since_unknown = 0;
    } else {
      steps_since_intraversable += 1;
      steps_since_unknown += 1;
    }
  }

  // Handle the last segment.
  if (steps_since_intraversable < width && steps_since_intraversable > 0) {
    // Change the previous states to INTRAVERSABLE.
    for (size_t j = states.size() - steps_since_intraversable; j < states.size(); ++j) {
      fuseStates(TraversabilityState::INTRAVERSABLE, new_states[j]);
    }
  } else if (steps_since_unknown < width && steps_since_unknown > 0) {
    // Change the previous states to UNKNOWN.
    for (size_t j = states.size() - steps_since_unknown; j < states.size(); ++j) {
      fuseStates(TraversabilityState::UNKNOWN, new_states[j], false);
    }
  }

  if (new_states != states) {
    states = new_states;
    return true;
  }
  return false;
}

const std::array<Side, 4> Side::ALL = {
    Side(Side::BOTTOM), Side(Side::LEFT), Side(Side::TOP), Side(Side::RIGHT)};

std::string Side::str() const {
  switch (index) {
    case BOTTOM:
      return "BOTTOM";
    case LEFT:
      return "LEFT";
    case TOP:
      return "TOP";
    case RIGHT:
      return "RIGHT";
    default:
      return "INVALID";
  }
}

Boundary::Boundary(const TraversabilityNodeAttributes& attrs)
    : Boundary(attrs.boundary, attrs.position) {}

Boundary::Boundary(const BoundaryInfo& info, const Eigen::Vector3d& position)
    : min(info.min + position.head<2>()),
      max(info.max + position.head<2>()),
      states(info.states) {}

Boundary::Boundary(const Eigen::Vector2d& min,
                   const Eigen::Vector2d& max,
                   const std::array<TraversabilityStates, 4>& states)
    : min(min), max(max), states(states) {}

bool Boundary::valid() const { return (min.x() < max.x() && min.y() < max.y()); }

double Boundary::width() const { return max.x() - min.x(); }

double Boundary::height() const { return max.y() - min.y(); }

double Boundary::area() const { return width() * height(); }

Eigen::Vector2d Boundary::center() const { return (min + max) / 2.0; }

std::string Boundary::str() const {
  std::stringstream ss;
  ss << "[" << min.x() << ", " << max.x() << "]x[" << min.y() << ", " << max.y() << "]";
  return ss.str();
}

double Boundary::voxelSize(Side side) const { return this->side(side).voxelSize(); }

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

bool Boundary::contains(const Eigen::Vector2d& point) const {
  return (point.x() >= min.x() && point.x() <= max.x() && point.y() >= min.y() &&
          point.y() <= max.y());
}

bool Boundary::contains(const Eigen::Vector3d& point) const {
  return contains(point.head<2>().eval());
}

bool Boundary::contains(const Boundary& other) const {
  return (min.x() <= other.min.x() && max.x() >= other.max.x() &&
          min.y() <= other.min.y() && max.y() >= other.max.y());
}

bool Boundary::intersects(const Boundary& other) const {
  return (min.x() < other.max.x() && max.x() > other.min.x() &&
          min.y() < other.max.y() && max.y() > other.min.y());
}

void Boundary::setCoordinate(Side side, double coordinate, bool preserve_voxel_size) {
  auto n1 = this->side(side.next());
  auto n2 = this->side(side.previous());

  // For single or unknown state there are no voxels.
  if (n1.states.size() <= 1 && n2.states.size() <= 1) {
    coord(side) = coordinate;
    return;
  }

  // Compute the new states for the new coordinate.
  const double voxel_size = std::min(n1.voxelSize(), n2.voxelSize());
  size_t num_states =
      std::round(std::abs(coordinate - coord(side.opposite())) / voxel_size);
  const bool is_upper = side.isUpper();
  if (n1.states.size() > 1) {
    n1.resizeStates(num_states, is_upper);
  }
  if (n2.states.size() > 1) {
    n2.resizeStates(num_states, is_upper);
  }

  // Set the new coordinate.
  if (preserve_voxel_size) {
    if (side.isLower()) {
      coord(side) = coord(side.opposite()) - num_states * voxel_size;
    } else {
      coord(side) = coord(side.opposite()) + num_states * voxel_size;
    }
  } else {
    coord(side) = coordinate;
  }
}

Boundary Boundary::intersection(const Boundary& other) const {
  return Boundary(min.cwiseMax(other.min), max.cwiseMin(other.max));
}

double Boundary::overlap(const Boundary& other) const {
  const auto intersection = this->intersection(other);
  if (!intersection.valid()) {
    return 0.0;
  }
  return intersection.area() / area();
}

double Boundary::computeIoU(const Boundary& other) const {
  const auto intersection = this->intersection(other);
  if (!intersection.valid()) {
    return 0.0;
  }
  const double int_area = intersection.area();
  return int_area / (area() + other.area() - int_area);
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

double Boundary::distanceToSide1D(const Side side, double coordinate) const {
  return side.isLower() ? getCoordinate(side) - coordinate
                        : coordinate - getCoordinate(side);
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
  if (std::abs(theta_q) >= std::numbers::pi - theta_0) {
    return Side::LEFT;
  }
  if (theta_q > 0) {
    return Side::TOP;
  }
  return Side::BOTTOM;
}

Side Boundary::lineIntersectsSide(const Eigen::Vector3d& source) const {
  return lineIntersectsSide(source.head<2>().eval());
}

Side Boundary::isOnSide(const Boundary& other) const {
  // x intersection
  if (other.max.x() >= min.x() && other.min.x() <= max.x()) {
    if (other.max.y() <= min.y()) {
      return Side::BOTTOM;
    } else if (other.min.y() >= max.y()) {
      return Side::TOP;
    }
  }

  // y intersection
  if (other.max.y() >= min.y() && other.min.y() <= max.y()) {
    if (other.max.x() <= min.x()) {
      return Side::LEFT;
    } else if (other.min.x() >= max.x()) {
      return Side::RIGHT;
    }
  }
  return Side::INVALID;
}

double Boundary::maxTraversableDistance(const Boundary& other, Side side) const {
  if (side == Side::INVALID) {
    return 0.0;
  }
  return this->side(side).maxTraversableDistance(other.side(side.opposite()));
}

void Boundary::mergeTraversabilityStates(const Boundary& other,
                                         double margin,
                                         bool optimistic) {
  for (const auto s : Side::ALL) {
    if (distanceToSide1D(s, other.getCoordinate(s.opposite())) > 0.0) {
      continue;  // No overlap on orthogonal axis
    }
    // Orthogonal distance between the sides, where positive distances mean this is
    // inside other.
    const double distance = distanceToSide1D(s, other.getCoordinate(s));
    if (distance < -margin) {
      continue;  // Too far inside to be relevant.
    }
    auto this_side = this->side(s);
    const auto other_side = other.side(s);
    const double start = std::max(this_side.min, other_side.min);
    const double end = std::min(this_side.max, other_side.max);

    if (end < start) {
      continue;
    }

    if (distance > 0.0) {
      // This area is inside the other boundary, thus traversable.
      for (size_t i = this_side.index(start); i <= this_side.index(end); ++i) {
        fuseStates(TraversabilityState::TRAVERSABLE, this_side.states[i], optimistic);
      }
    }
    if (distance <= margin) {
      // Within the margin area fuse other states.
      for (size_t i = this_side.index(start); i <= this_side.index(end); ++i) {
        fuseStates(other_side.getState(this_side.coordFromIndex(i)),
                   this_side.states[i],
                   optimistic);
      }
    }
  }
}

void Boundary::toAttributes(TraversabilityNodeAttributes& attrs) const {
  const Eigen::Vector2d c = center();
  attrs.position.head(2) = c;
  attrs.boundary.min = min - c;
  attrs.boundary.max = max - c;
  attrs.boundary.states = states;
}

Boundary::BoundarySide Boundary::side(Side side) {
  if (side == Side::INVALID) {
    throw std::out_of_range("Invalid side index for BoundarySide.");
  }
  if (side.horizontal()) {
    return BoundarySide(min.x(), max.x(), states[side], side);
  }
  return BoundarySide(min.y(), max.y(), states[side], side);
}

const Boundary::BoundarySide Boundary::side(Side side) const {
  return const_cast<Boundary*>(this)->side(side);
}

double Boundary::BoundarySide::voxelSize() const {
  if (states.size() <= 1) {
    return max - min;
  }
  return (max - min) / states.size();
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
  TraversabilityStates states_in_range;
  if (states.size() == 1) {
    states_in_range = other.getStates(start, end);
  } else if (other.states.size() == 1) {
    states_in_range = getStates(start, end);
  } else {
    // NOTE(lschmid): This assumes equal voxel spacing.
    states_in_range = getStates(start, end);
    const auto other_states = other.getStates(start, end);
    fuseStates(other_states, states_in_range, false);
  }

  return computeMinMaxTraversability(states_in_range).first * voxelSize();
}

TraversabilityState Boundary::BoundarySide::getState(double coordinate) const {
  if (states.size() == 0) {
    return TraversabilityState::UNKNOWN;
  }
  return states[index(coordinate)];
}

TraversabilityStates Boundary::BoundarySide::getStates(double from, double to) const {
  if (from > max || to < from || states.empty()) {
    return {};
  }
  const size_t start = std::max(
      0, static_cast<int>(std::floor((from - min) / (max - min) * states.size())));
  const size_t end = std::min(
      states.size(),
      static_cast<size_t>(std::ceil((to - min) / (max - min) * states.size())));
  TraversabilityStates result(end - start);
  for (size_t i = start; i < end; ++i) {
    result[i - start] = states[i];
  }
  return result;
}

void Boundary::BoundarySide::resizeStates(size_t new_size, bool on_upper_side) {
  if (states.size() == new_size) {
    return;
  }
  if (on_upper_side) {
    // Pad the ends.
    states.resize(new_size, TraversabilityState::UNKNOWN);
    return;
  }
  // Pad the fronts.
  if (new_size > states.size()) {
    const size_t diff = new_size - states.size();
    states.insert(states.begin(), diff, TraversabilityState::UNKNOWN);
  } else {
    const size_t diff = states.size() - new_size;
    states.erase(states.begin(), states.begin() + diff);
  }
}

void Boundary::BoundarySide::fuseBoundaryStates(
    const BoundarySide& other,
    std::function<void(TraversabilityState, TraversabilityState&)> fuse_fn) {
  const double start = std::max(min, other.min);
  const double end = std::min(max, other.max);
  if (start > end) {
    return;
  }

  if (states.empty()) {
    states = {TraversabilityState::UNKNOWN};
  }

  if (states.size() == 1) {
    fuse_fn(other.getState(start), states[0]);
    return;
  }

  // Interpolate all voxels in the range.
  size_t idx = index(start);
  const double voxel_size = voxelSize();
  for (double coordinate = start + 0.5 * voxel_size; coordinate < end;
       coordinate += voxel_size, ++idx) {
    fuse_fn(other.getState(coordinate), states[idx]);
  }
}

void Boundary::BoundarySide::fuseBoundaryStates(const BoundarySide& other,
                                                bool optimistic) {
  fuseBoundaryStates(other,
                     [optimistic](TraversabilityState from, TraversabilityState& to) {
                       fuseStates(from, to, optimistic);
                     });
}

double& Boundary::coord(Side side) {
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

size_t Boundary::BoundarySide::index(double coordinate) const {
  if (states.size() <= 1) {
    return 0;
  }
  if (coordinate >= max) {
    return states.size() - 1;
  }
  return static_cast<size_t>((coordinate - min) / voxelSize());
}

double Boundary::BoundarySide::coordFromIndex(size_t index) const {
  if (states.size() <= 1) {
    return (max + min) / 2.0;
  }
  return min + voxelSize() * (index + 0.5);
}

}  // namespace spark_dsg
