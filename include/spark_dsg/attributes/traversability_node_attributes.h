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

#include <array>
#include <vector>

#include "spark_dsg/attributes/semantic_node_attributes.h"

namespace spark_dsg {

/**
 * @brief The traversability state of a traversability boundary.
 */
enum class TraversabilityState : uint8_t {
  UNKNOWN = 0,
  TRAVERSABLE = 1,
  INTRAVERSABLE = 2,
  TRAVERSED = 3
};

using TraversabilityStates = std::vector<TraversabilityState>;

/**
 * @brief Compact information to store a grid aligned traversability boundary.
 */
struct BoundaryInfo {
  //! Coordinates of the boundary w.r.t. the attribute center.
  Eigen::Vector2d min;
  Eigen::Vector2d max;

  /**
   * @brief Traversability states for each side, ordered bottom, left, top, right.
   * Each side can be empty (=UNKNOWN), a single state, or a uniform tessellation.
   * States on each side run from the lower to the higher coordinate.
   */
  std::array<TraversabilityStates, 4> states;

  bool operator==(const BoundaryInfo& other) const;
  bool operator!=(const BoundaryInfo& other) const { return !(*this == other); }
};

/**
 * @brief First simple implementation of traversability places.
 */
struct TraversabilityNodeAttributes : public SemanticNodeAttributes {
 public:
  using Ptr = std::unique_ptr<TraversabilityNodeAttributes>;

  TraversabilityNodeAttributes() = default;
  virtual ~TraversabilityNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! Timestamps when this place was first and last observed.
  uint64_t first_observed_ns = 0;
  uint64_t last_observed_ns = 0;

  //! Boundary information
  BoundaryInfo boundary;

  // TODO(lschmid): Reconsider in the future.
  //! Distance to the nearest intraversable obstacle.
  double distance = 0.0;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;

  const serialization::RegistrationInfo& registrationImpl() const override;
};

/**
 * @brief Simpler version of traversability places for region growing places, spanning a
 * polygon from uniform rays starting at the center (=the position).
 * @todo Sort out unifiying interfaces eventually. Implemented as new attributes as
 * discussed w/ Nathan.
 * @todo Find better names for this...
 */
struct TravNodeAttributes : public SemanticNodeAttributes {
 public:
  using Ptr = std::unique_ptr<TravNodeAttributes>;

  TravNodeAttributes() = default;
  virtual ~TravNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! Timestamps when this place was first and last observed.
  uint64_t first_observed_ns = 0;
  uint64_t last_observed_ns = 0;

  //! Radii for the boundary rays.
  std::vector<double> radii;

  //! Corresponding traversability states for each boundary ray.
  TraversabilityStates states;

  //! Approximation of the boundary as circles.
  double min_radius = 0.0;
  double max_radius = 0.0;

  /**
   * @brief Compute the boundary from exterior points in world coordinates. This assumes
   * the current position as the center and the current size of the radii as the number
   * of rays.
   * @param points_W Exterior points in world coordinates.
   * @param states Optional traversability states for each input point.
   */
  void fromExteriorPoints(const std::vector<Eigen::Vector3d>& points_W,
                          const TraversabilityStates& states_in = {});

  /**
   * @brief Clear all voxel information. This does not clear timestamps or radii.
   */
  void clear();

  /**
   * @brief Check if a point in world coordinates is within the traversability boundary.
   * @param point Point in world coordinates.
   */
  bool contains(const Eigen::Vector3d& point_W) const;

  /**
   * @brief Check if this traversability boundary intersects with another.
   * @param other Other traversability boundary.
   */
  bool intersects(const TravNodeAttributes& other) const;

  /**
   * @brief Compute the area of the traversability boundary.
   */
  double area() const;

  /**
   * @brief Get the bin index for a point in local coordinates. The bin will always be
   * valid and the floored index is returned.
   * @param point Point in local coordinates.
   */
  size_t getBin(const Eigen::Vector3d& point_L) const;

  /**
   * @brief Get the linear coordinates [0,1] around the circle (i.e., all bins).
   * @param point Point in local coordinates.
   */
  double getBinPercentage(const Eigen::Vector3d& point_L) const;

  /**
   * @brief Get the boundary point for a given bin index.
   * @param bin Bin index.
   * @param in_world_frame Whether to return the point in world frame (true) or
   * local frame (false).
   */
  Eigen::Vector3d getBoundaryPoint(size_t bin, bool in_world_frame = true) const;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;

  const serialization::RegistrationInfo& registrationImpl() const override;
};

}  // namespace spark_dsg
