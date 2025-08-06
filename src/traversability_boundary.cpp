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

void fuseStates(TraversabilityState from, TraversabilityState& to) {
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

void fuseStates(const TraversabilityStates& from, TraversabilityStates& to) {
  if (to.size() < from.size()) {
    to.resize(from.size());
  }
  for (size_t i = 0; i < from.size(); ++i) {
    fuseStates(from[i], to[i]);
  }
}

void fuseStates(TraversabilityState from, TraversabilityStates& to) {
  for (auto& state : to) {
    fuseStates(from, state);
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

bool filterTraversabilityStates(TraversabilityStates& states, size_t width) {
  // TODO(lschmid): Implement.
  if (states.size() < width || width == 0) {
    return false;
  }

  TraversabilityStates filtered(states.size(), TraversabilityState::INTRAVERSABLE);
  bool changed = false;

  for (size_t i = 0; i <= states.size() - width; ++i) {
    bool has_intraversable = false;
    bool has_unknown = false;
    for (size_t j = 0; j < width; ++j) {
      if (states[i + j] == TraversabilityState::INTRAVERSABLE) {
        has_intraversable = true;
        break;
      }
      if (states[i + j] == TraversabilityState::UNKNOWN) {
        has_unknown = true;
      }
    }
    TraversabilityState new_state;
    if (has_intraversable) {
      new_state = TraversabilityState::INTRAVERSABLE;
    } else if (has_unknown) {
      new_state = TraversabilityState::UNKNOWN;
    } else {
      new_state = TraversabilityState::TRAVERSABLE;
    }
    if (filtered[i + width / 2] != new_state) {
      filtered[i + width / 2] = new_state;
      changed = true;
    }
  }

  // Copy filtered states back, keeping only the filtered region
  states = filtered;
  return changed;
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

bool Boundary::intersects(const Boundary& other) const {
  return (min.x() < other.max.x() && max.x() > other.min.x() &&
          min.y() < other.max.y() && max.y() > other.min.y());
}

void Boundary::setCoordinate(Side side, double coordinate, bool preserve_voxel_size) {
  // For single or unknown state there are no voxels.
  const Side n_side = side.next();
  auto& n_states_1 = states[n_side];
  if (n_states_1.size() <= 1) {
    coord(side) = coordinate;
    return;
  }

  // Compute the new states for the new coordinate.
  const double voxel_size = voxelSize(n_side);
  size_t num_states =
      std::round(std::abs(coordinate - coord(side.opposite())) / voxel_size);
  auto& n_states_2 = states[n_side.opposite()];
  if (side.isUpper()) {
    // Pad the ends.
    n_states_1.resize(num_states, TraversabilityState::UNKNOWN);
    n_states_2.resize(num_states, TraversabilityState::UNKNOWN);
  } else {
    // Pad the fronts.
    if (num_states > n_states_1.size()) {
      const size_t diff = num_states - n_states_1.size();
      for (size_t i = 0; i < diff; ++i) {
        n_states_1.insert(n_states_1.begin(), TraversabilityState::UNKNOWN);
        n_states_2.insert(n_states_2.begin(), TraversabilityState::UNKNOWN);
      }
    } else if (num_states < n_states_1.size()) {
      const size_t diff = n_states_1.size() - num_states;
      n_states_1.erase(n_states_1.begin(), n_states_1.begin() + diff);
      n_states_2.erase(n_states_2.begin(), n_states_2.begin() + diff);
    }
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
  TraversabilityStates states_in_range;
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

}  // namespace spark_dsg
