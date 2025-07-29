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
#pragma once

#include <spark_dsg/node_attributes.h>

#include <Eigen/Geometry>

namespace spark_dsg {

/**
 * @brief Check whether a state is considered traversable, i.e., it is TRAVERSABLE or
 * TRAVERSED. If optimistic is true, UNKNOWN is also considered traversable.
 */
bool traversable(TraversabilityState state, bool optimistic = false);

/**
 * @brief Check whether a state is considered intraversable, i.e., it is INTRAVERSABLE.
 * If optimistic is false, UNKNOWN is also considered intraversable.
 */
bool intraversable(TraversabilityState state, bool optimistic = false);

/**
 * @brief Combine two TraversabilityStates into a single state. Order of precedence:
 * Traversed -> Intraversable -> Unknown -> Traversable.
 */
void fuseStates(TraversabilityState from, TraversabilityState& to);
void fuseStates(const TraversabilityStates& from, TraversabilityStates& to);
void fuseStates(TraversabilityState from, TraversabilityStates& to);

/**
 * @brief Compute the minimum (pessimistic) and maximum (optimistic) length of
 * traversable voxels in a sequence.
 * @return <min_length, max_length> in number of voxels.
 */
std::pair<size_t, size_t> computeMinMaxTraversability(
    const TraversabilityStates& states);

/**
 * @brief Simplify the traversability states if they can be represented by a single
 * state.
 * @param states The traversability states to simplify.
 * @return True if the states were modified.
 */
bool simplifyTraversabilityStates(TraversabilityStates& states);

/**
 * @brief Fitler the traversability states to reflect the traversability for a robot of
 * a specific width.
 * @param states The traversability states to filter.
 * @param width The width of the robot in number of voxels.
 * @return True if the states were modified.
 */
bool filterTraversabilityStates(TraversabilityStates& states, size_t width);

/**
 * @brief More human readable definitions of the four sides of a rectangle.
 */
struct Side {
  static constexpr size_t BOTTOM = 0;
  static constexpr size_t LEFT = 1;
  static constexpr size_t TOP = 2;
  static constexpr size_t RIGHT = 3;
  static constexpr size_t INVALID = 4;
  static const std::array<Side, 4> ALL;

  Side(size_t idx) : index(idx < INVALID ? idx : INVALID) {}

  operator size_t() const { return index; }
  Side& operator=(size_t idx) {
    index = idx < INVALID ? idx : INVALID;
    return *this;
  }

  /**
   * @brief Whether the side is valid.
   */
  bool valid() const { return index < INVALID; }

  /**
   * @brief Returns true if the side has a horizontal orientation, i.e. top or bottom.
   */
  bool horizontal() const { return index == BOTTOM || index == TOP; }

  /**
   * @brief Returns true if the side has a vertical orientation, i.e. left or right.
   */
  bool vertical() const { return index == LEFT || index == RIGHT; }

  /**
   * @brief Returns the opposite side.
   */
  Side opposite() const { return Side((index + 2) % 4); }
  /**
   * @brief Returns the next side in clockwise order.
   */
  Side next() const { return Side((index + 1) % 4); }

  /**
   * @brief Returns the previous side in clockwise order, i.e. the next side
   * counter-clockwise.
   */
  Side previous() const { return Side((index + 3) % 4); }

  /**
   * @brief Returns a string representation of the side.
   */
  std::string str() const;

 private:
  size_t index;
};

/**
 * @brief Actionable interface to query and compare boundaries for an axis-aligned 2D
 * rectangle.
 * @note The preferred use is to construct a Boundary from TraversabilityNodeAttributes
 * to interact with it. If it is modified use toAttributes() to write it back.
 */
struct Boundary {
  //! Boundary limits in world coordinates.
  Eigen::Vector2d min;
  Eigen::Vector2d max;

  //! Traversability states for each side of the boundary.
  // TODO(lschmid): Just copying this is not the most efficient but seems ok for now.
  std::array<TraversabilityStates, 4> states;

  // Construction.
  Boundary() = default;
  explicit Boundary(const TraversabilityNodeAttributes& attrs);
  explicit Boundary(const BoundaryInfo& info,
                    const Eigen::Vector3d& position = Eigen::Vector3d::Zero());
  Boundary(const Eigen::Vector2d& min,
           const Eigen::Vector2d& max,
           const std::array<TraversabilityStates, 4>& states = {});

  // Operators.
  operator bool() const { return valid(); }

  // Properties.
  bool valid() const;
  double width() const;
  double height() const;
  double area() const;
  Eigen::Vector2d center() const;
  void toAttributes(TraversabilityNodeAttributes& attrs) const;
  std::string str() const;

  // Lookup.
  bool contains(const Eigen::Vector2d& point) const;
  bool contains(const Eigen::Vector3d& point) const;
  bool intersects(const Boundary& other) const;
  Boundary intersection(const Boundary& other) const;
  double xIntersection(const Boundary& other) const;
  double yIntersection(const Boundary& other) const;
  double xDistance(const Boundary& other) const;
  double yDistance(const Boundary& other) const;
  double distanceToSide(const Side side, const Eigen::Vector2d& point) const;

  // Sides: 0: bottom, 1: left, 2: top, 3: right.
  double getCoordinate(Side side) const;
  void setCoordinate(Side side, double coordinate);
  bool containsOtherBoxWidth(const Boundary& other, bool vertical) const;
  Side lineIntersectsSide(const Eigen::Vector2d& source) const;
  Side lineIntersectsSide(const Eigen::Vector3d& source) const;

  /**
   * @brief Whether the other boundary is besides a side of this boundary.
   * @param other The other boundary to check.
   * @return The side of this boundary that the other boundary is on, or INVALID if
   * it intersects or is not aligned with any side.
   */
  Side isOnSide(const Boundary& other) const;

  /**
   * @brief Compute the maximum width of the traversable distance between two aligning
   * sides.
   * @param other The other boundary to compare with.
   * @param side The side of this boundary to compare with the opposite side on the
   * other boundary.
   */
  double maxTraversableDistance(const Boundary& other, Side side) const;

  /**
   * @brief 1D representation of a boundary side for side-based computations.
   */
  struct BoundarySide {
    double& min;
    double& max;
    TraversabilityStates& states;

    BoundarySide(double& min, double& max, TraversabilityStates& states)
        : min(min), max(max), states(states) {}

    /**
     * @brief Get the voxel size for this boundary side.
     */
    double voxelSize() const { return (max - min) / states.size(); }

    /**
     * @brief Compute the maximum width of the traversable distance between two aligning
     * sides.
     */
    double maxTraversableDistance(const BoundarySide& other) const;

    /**
     * @brief Get the traversability state at a specific coordinate. This assumes the
     * coordinate is within the range of the side.
     */
    TraversabilityState getState(double coordinate) const;

    /**
     * @brief Get all traversability states that fall within the given range. This will
     * not pad the result with unobserved states.
     */
    TraversabilityStates getStates(double from, double to) const;
  };

  BoundarySide side(Side side);
  const BoundarySide side(Side side) const;
};

}  // namespace spark_dsg
