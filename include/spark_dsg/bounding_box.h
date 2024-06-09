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

#include "spark_dsg/mesh.h"

namespace spark_dsg {

/**
 * @brief Union of AABB, RAABB, and OBB bounding box formats. Can directly be extracted
 * from point-cloud data if needed.
 */
struct BoundingBox {
  // Types.
  /**
   * @brief type of bounding box (defaults to AABB)
   */
  enum class Type : int32_t {
    INVALID = 0, /**< an invalid bounding box */
    AABB = 1,    /**< an axis-aligned bounding box */
    OBB = 2,     /**< an oriented bounding box */
    RAABB = 3    /**< a (yaw-adjusted) axis-aligned bounding box */
  };

  /**
   * Interface to lookup points to allow fitting a bounding box to a set of points.
   */
  struct PointAdaptor {
    virtual size_t size() const = 0;
    virtual Eigen::Vector3f get(size_t index) const = 0;
    Eigen::Vector3f operator[](size_t index) const { return get(index); }
  };

  // Constructors.
  /**
   * @brief Construct an INVALID (unitialized) bounding box.
   */
  BoundingBox() = default;

  /**
   * @brief Construct an AABB bounding box from only an extent.
   * @param dimensions The dimensions of the bounding box along each axis.
   */
  explicit BoundingBox(const Eigen::Vector3f& dimensions);

  /**
   * @brief Construct an AABB bounding box in world coordinate frame.
   * @param dimensions The dimensions of the bounding box along each axis.
   * @param world_P_center The center of the bounding box in world coordinates.
   */
  BoundingBox(const Eigen::Vector3f& dimensions, const Eigen::Vector3f& world_P_center);

  /**
   * @brief Construct an RAABB bounding box from min and max points in world coordinate.
   * frame.
   * @param dimensions The dimensions of the bounding box along each axis.
   * @param world_P_center The center of the bounding box in world coordinates.
   * @param yaw The yaw angle of the bounding box in radians.
   */
  BoundingBox(const Eigen::Vector3f& dimensions,
              const Eigen::Vector3f& world_P_center,
              float yaw);

  /**
   * @brief Construct an OBB bounding box.
   * @param dimensions The dimensions of the bounding box along each axis.
   * @param world_P_center The center of the bounding box in world coordinates.
   * @param world_R_center The orientation of the bounding box in world coordinates.
   */
  BoundingBox(const Eigen::Vector3f& dimensions,
              const Eigen::Vector3f& world_P_center,
              const Eigen::Quaternionf& world_R_center);

  /**
   * @brief Construct an arbitrary bounding box.
   * @param type The type of the bounding box.
   * @param dimensions The dimensions of the bounding box along each axis.
   * @param world_P_center The center of the bounding box in world coordinates.
   * @param world_R_center The orientation of the bounding box in world coordinates.
   */
  BoundingBox(BoundingBox::Type type,
              const Eigen::Vector3f& dimensions,
              const Eigen::Vector3f& world_P_center,
              const Eigen::Matrix3f& world_R_center);

  /**
   * @brief Fit a bounding box of a given type to a set of points in world coordinates.
   * @param points Adaptor to lookup the points to fit the bounding box to.
   * @param type The type of bounding box to fit. Defaults to AABB.
   */
  explicit BoundingBox(const PointAdaptor& points,
                       BoundingBox::Type type = BoundingBox::Type::AABB);

  /**
   * @brief Fit a bounding box of a given type to a set of points in world coordinates.
   * @param points Points to fit the bounding box to.
   * @param type The type of bounding box to fit. Defaults to AABB.
   */
  explicit BoundingBox(const std::vector<Eigen::Vector3f>& points,
                       BoundingBox::Type type = BoundingBox::Type::AABB);

  /**
   * @brief Fit a bounding box of a given type to a mesh in world coordinates.
   * @param mesh Adaptor to lookup the points to fit the bounding box to.
   * @param type The type of bounding box to fit. Defaults to AABB.
   */
  explicit BoundingBox(const Mesh& mesh,
                       BoundingBox::Type type = BoundingBox::Type::AABB);

  virtual ~BoundingBox() = default;

  // Operators.
  bool operator==(const BoundingBox& other) const;
  bool operator!=(const BoundingBox& other) const { return !(*this == other); }
  operator bool() const { return isValid(); }

  // Bounding box properties.
  /**
   * @brief Check if a bounding box is setup properly (i.e., it must have positive
   * volume).
   */
  bool isValid() const;

  /**
   * @brief Get the volume of the bounding box in m3.
   */
  float volume() const;

  /**
   * @brief Returns true if the bounding box has a non-identity rotation.
   */
  bool hasRotation() const;

  /**
   * @brief Get all corners of the bounding box in world frame.
   */
  std::array<Eigen::Vector3f, 8> corners() const;

  // Bounding box operations.

  /**
   * @brief Merge this bounding box with another bounding box, computing a bounding box
   * that contains both.
   */
  void merge(const BoundingBox& other);

  // Bounding box inference.
  /**
   * @brief Transform a point from world frame to bounding box frame.
   * @param point_W The point to transform in world coordinates.
   * @returns The point in bounding box coordinates.
   */
  Eigen::Vector3f pointToWorldFrame(const Eigen::Vector3f& point_B) const;

  /**
   * @brief Transform a point from bounding box frame to world frame.
   * @param point_B The point to transform in bounding box coordinates.
   * @returns The point in world coordinates.
   */
  Eigen::Vector3f pointToBoxFrame(const Eigen::Vector3f& point_W) const;

  /**
   * @brief Check whether this bounding box contains the given point in world
   * coordinates.
   */
  bool contains(const Eigen::Vector3f& point) const;
  bool contains(const Eigen::Vector3d& point) const;

  // Bounding box comparisons.
  /**
   * @brief Check whether this bounding box intersects with another bounding box.
   * @note Currently only supports AABB-AABB intersection.
   */
  bool intersects(const BoundingBox& other) const;

  /**
   * @brief Compute the intersection over union (IoU) of this bounding box with another.
   * @note Currently only supports AABB-AABB intersection.
   */
  float computeIoU(const BoundingBox& other) const;

  /**
   * @brief output bounding box information
   * @param out output stream
   * @param bounding_box bounding box to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const BoundingBox& bounding_box);

  // Properties.
  // Type of the current bounding box.
  Type type = Type::INVALID;

  //! Dimensions along each axis.
  Eigen::Vector3f dimensions = Eigen::Vector3f::Zero();

  //! Position of the bounding box center in world frame.
  Eigen::Vector3f world_P_center = Eigen::Vector3f::Zero();

  //! World orientation (only for OBB)
  Eigen::Matrix3f world_R_center = Eigen::Matrix3f::Identity();

 protected:
  /**
   * @brief Check whether a point is inside the bounding box.
   * @param point_B The point to check in bounding box coordinates.
   */
  bool isInside(const Eigen::Vector3f& point_B) const;
  void mergeAABB(const BoundingBox& other);
  void mergeGeneral(const BoundingBox& other);

  // Min and max corners of an AABB in world frame.
  Eigen::Vector3f minCorner() const;
  Eigen::Vector3f maxCorner() const;

 public:
  // Specialized point adaptors.
  struct MeshAdaptor : PointAdaptor {
    MeshAdaptor(const Mesh& mesh, const std::vector<size_t>* indices = nullptr)
        : mesh(mesh), indices(indices) {}
    size_t size() const override;
    Eigen::Vector3f get(size_t index) const override;
    const Mesh& mesh;
    const std::vector<size_t>* indices;
  };

  struct PointVectorAdaptor : PointAdaptor {
    PointVectorAdaptor(const std::vector<Eigen::Vector3f>& points) : points(points) {}
    size_t size() const override;
    Eigen::Vector3f get(size_t index) const override;
    const std::vector<Eigen::Vector3f>& points;
  };
};

}  // namespace spark_dsg
