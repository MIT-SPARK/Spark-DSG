#include "spark_dsg/traversability_boundary.h"

namespace spark_dsg {

Boundary::Boundary(const TraversabilityNodeAttributes& attrs)
    : min(attrs.boundary.min + attrs.position.head<2>()),
      max(attrs.boundary.max + attrs.position.head<2>()) {}

Boundary::Boundary(const Eigen::Vector2d& min, const Eigen::Vector2d& max)
    : min(min), max(max) {}

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

double Boundary::getCoordinate(const Side side) const {
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

void Boundary::setCoordinate(const Side side, const double coordinate) {
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

// Side intersectsSide(const Eigen::Vector3d& point,
//                     const TraversabilityNodeAttributes& attrs) {
//   const Eigen::Vector3d query = point - attrs.position;
//   const double theta_q = std::atan2(query.y(), query.x());  // [-PI, PI]
//   const double theta_0 = std::atan2(attrs.boundary[3].point.y(),
//                                     attrs.boundary[3].point.x());  // Top-right
//   if (std::abs(theta_q) <= theta_0) {
//     return Side::RIGHT;
//   }
//   if (std::abs(theta_q) >= M_PI - theta_0) {
//     return Side::LEFT;
//   }
//   if (theta_q > 0) {
//     return Side::TOP;
//   } else {
//     return Side::BOTTOM;
//   }
// }

}  // namespace spark_dsg
