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
#include <Eigen/Geometry>
#include <iostream>

namespace spark_dsg {

/**
 * @brief Bounding box representation
 *
 * Union of AABB and OBB bounding box formats. Can directly be extracted from
 * point-cloud if needed
 */
struct BoundingBox {
  /**
   * @brief type of bounding box (defaults to AABB)
   */
  enum class Type : int32_t {
    INVALID = 0,          /**< an invalid bounding box */
    AABB = 1,             /**< an axis-aligned bounding box */
    OBB = 2,              /**< an oriented bounding box */
    RAABB = 3             /**< a (yaw-adjusted) axis-aligned bounding box */
  } type = Type::INVALID; /**< type of the current bounding box */

  /**
   * @brief Construct an INVALID (unitialized) bounding box
   */
  BoundingBox();

  // TODO(nathan) fix calculating position for this constructor
  /**
   * @brief Construct an AABB bounding box
   */
  BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);

  /**
   * @brief Construct an OBB bounding box
   */
  BoundingBox(const Eigen::Vector3f& min,
              const Eigen::Vector3f& max,
              const Eigen::Vector3f& world_P_center,
              const Eigen::Quaternionf& world_R_center);

  /**
   * @brief Construct an arbitrary bounding box
   */
  BoundingBox(BoundingBox::Type type,
              const Eigen::Vector3f& min,
              const Eigen::Vector3f& max,
              const Eigen::Vector3f& world_P_center,
              const Eigen::Matrix3f& world_R_center);

  virtual ~BoundingBox() = default;

  // TODO(nathan) consider condensing
  bool isInside(const Eigen::Vector3d& point) const;

  bool isInside(const Eigen::Vector3f& point) const;

  float volume() const;

  //! minimum corner extent
  Eigen::Vector3f min;
  //! maximum corner extent
  Eigen::Vector3f max;
  //! world position (only for OBB)
  Eigen::Vector3f world_P_center;
  //! world orientation (only for OBB)
  Eigen::Matrix3f world_R_center;

  /**
   * @brief output bounding box information
   * @param out output stream
   * @param bounding_box bounding box to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const BoundingBox& bounding_box);

 protected:
  bool isInsideOBB(const Eigen::Vector3f& point_world) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace spark_dsg
