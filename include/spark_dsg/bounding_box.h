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
#define PCL_NO_PRECOMPILE
#include <pcl/features/moment_of_inertia_estimation.h>
#undef PCL_NO_PRECOMPILE
#include <Eigen/Geometry>
#include <iostream>

namespace spark_dsg {

template <typename PointT>
PointT getRotatedPoint(const Eigen::Matrix3f& new_R_old,
                       const Eigen::Vector3f& new_t_old,
                       const PointT& original) {
  PointT new_point = original;
  Eigen::Vector3f pos(new_point.x, new_point.y, new_point.z);
  Eigen::Vector3f new_pos = new_R_old * pos + new_t_old;
  new_point.x = new_pos.x();
  new_point.y = new_pos.y();
  new_point.z = new_pos.z();
  return new_point;
}

// forward declare functions
struct BoundingBox;

template <typename PointT>
BoundingBox extractRAABBBox(const pcl::MomentOfInertiaEstimation<PointT>& estimator,
                            const pcl::IndicesPtr& indices);

// TODO(nathan) fix bbox type enum container

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

  // TODO(nathan) PCL switches to std::shared at some point.
  /**
   * @brief construct a bounding box directly from a pointcloud
   */
  template <typename PclPtr>
  static BoundingBox extract(const PclPtr& cloud,
                             Type type = Type::AABB,
                             const pcl::IndicesPtr& active_indices = nullptr) {
    if (!cloud) {
      throw std::runtime_error("invalid point cloud pointer");
    }

    if (cloud->empty() || type == Type::INVALID) {
      return {};
    }

    using PointT = typename PclPtr::element_type::PointType;
    pcl::MomentOfInertiaEstimation<PointT> estimator;
    estimator.setInputCloud(cloud);
    if (active_indices) {
      estimator.setIndices(active_indices);
    }
    estimator.compute();

    PointT pcl_min;
    PointT pcl_max;
    PointT pcl_position;
    Eigen::Matrix3f pcl_rotation;
    BoundingBox box;
    box.type = type;
    // TODO(nathan) consider using constructors
    switch (type) {
      case Type::AABB:
        estimator.getAABB(pcl_min, pcl_max);
        box.min << pcl_min.x, pcl_min.y, pcl_min.z;
        box.max << pcl_max.x, pcl_max.y, pcl_max.z;
        box.world_P_center = (box.max + box.min) / 2.0;
        break;
      case Type::OBB:
        estimator.getOBB(pcl_min, pcl_max, pcl_position, pcl_rotation);
        box.min << pcl_min.x, pcl_min.y, pcl_min.z;
        box.max << pcl_max.x, pcl_max.y, pcl_max.z;
        box.world_P_center << pcl_position.x, pcl_position.y, pcl_position.z;
        box.world_R_center = Eigen::Quaternionf(pcl_rotation).toRotationMatrix();
        break;
      case Type::RAABB:
        box = extractRAABBBox<PointT>(estimator, active_indices);
        break;
      default:
        std::stringstream ss;
        ss << "Unknown bounding box type: " << static_cast<int>(type);
        throw std::runtime_error(ss.str());
    }

    return box;
  }

 protected:
  bool isInsideOBB(const Eigen::Vector3f& point_world) const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename PointT>
BoundingBox extractRAABBBox(const pcl::MomentOfInertiaEstimation<PointT>& estimator,
                            const pcl::IndicesPtr& indices) {
  PointT pcl_min;
  PointT pcl_max;
  PointT pcl_position;
  Eigen::Matrix3f pcl_rotation;

  estimator.getOBB(pcl_min, pcl_max, pcl_position, pcl_rotation);

  const auto& cloud = estimator.getInputCloud();

  typename pcl::PointCloud<PointT>::Ptr rotated_cloud(new pcl::PointCloud<PointT>());

  // from the lavalle motion planning book (http://planning.cs.uiuc.edu/node103.html)
  // this is likely incorrect sometimes
  float yaw = std::atan2(pcl_rotation(1, 0), pcl_rotation(0, 0));
  // note that this is the inverse of a standard z-axis rotation
  Eigen::Matrix3f new_R_old;
  new_R_old << std::cos(yaw), std::sin(yaw), 0.0f, -std::sin(yaw), std::cos(yaw), 0.0f,
      0.0f, 0.0f, 1.0f;

  Eigen::Vector3f old_t_new(pcl_position.x, pcl_position.y, pcl_position.z);
  Eigen::Vector3f new_t_old = -new_R_old * old_t_new;

  if (indices) {
    rotated_cloud->reserve(indices->size());
    for (const auto& idx : *indices) {
      rotated_cloud->push_back(getRotatedPoint(new_R_old, new_t_old, cloud->at(idx)));
    }
  } else {
    rotated_cloud->reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      rotated_cloud->push_back(getRotatedPoint(new_R_old, new_t_old, cloud->at(i)));
    }
  }

  pcl::MomentOfInertiaEstimation<PointT> new_estimator;
  new_estimator.setInputCloud(rotated_cloud);
  new_estimator.compute();
  new_estimator.getAABB(pcl_min, pcl_max);
  Eigen::Vector3f min;
  min << pcl_min.x, pcl_min.y, pcl_min.z;
  Eigen::Vector3f max;
  max << pcl_max.x, pcl_max.y, pcl_max.z;
  return BoundingBox(
      BoundingBox::Type::RAABB, min, max, old_t_new, new_R_old.transpose());
}

}  // namespace spark_dsg
