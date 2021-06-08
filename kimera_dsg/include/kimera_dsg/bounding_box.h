#pragma once
#include <glog/logging.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/Geometry>
#include <iostream>

namespace kimera {

struct BoundingBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief type of bounding box (defaults to AABB)
   */
  enum class Type : int { INVALID = 0, AABB = 1, OBB = 2 } type = Type::INVALID;

  BoundingBox();

  BoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max);

  BoundingBox(const Eigen::Vector3f& min,
              const Eigen::Vector3f& max,
              const Eigen::Vector3f world_P_center,
              const Eigen::Quaternionf& world_R_center);

  virtual ~BoundingBox() = default;

  //! minimum corner extent
  Eigen::Vector3f min;
  //! maximum corner extent
  Eigen::Vector3f max;
  //! world position (only for OBB)
  Eigen::Vector3f world_P_center;
  //! world orientation (only for OBB)
  Eigen::Quaternionf world_R_center;

  friend std::ostream& operator<<(std::ostream& os,
                                  const BoundingBox& bounding_box);

  // TODO(nathan) PCL switches to std::shared at some point.
  // template deduction doesn't handle derived typenames well, so we can't
  // just use Ptr and CostPtr
  // static method rather than namespaced function to make using Type easier
  template <typename CloudT>
  static BoundingBox extract(const boost::shared_ptr<const CloudT>& cloud,
                             Type type = Type::AABB) {
    if (!cloud) {
      throw std::runtime_error("invalid point cloud pointer");
    }

    if (cloud->empty()) {
      throw std::runtime_error("invalid point cloud");
    }

    if (type == Type::INVALID) {
      throw std::runtime_error("Can't make a bounding box of type INVALID!");
    }

    VLOG(5) << "Eigen alignment: min -> " << EIGEN_MIN_ALIGN_BYTES
            << " max (ideal) -> " << EIGEN_IDEAL_MAX_ALIGN_BYTES << " max -> "
            << EIGEN_MAX_ALIGN_BYTES << std::endl;

    using PointT = typename CloudT::PointType;
    pcl::MomentOfInertiaEstimation<PointT> estimator;
    estimator.setInputCloud(cloud);
    estimator.compute();

    PointT pcl_min;
    PointT pcl_max;
    PointT pcl_position;
    Eigen::Matrix3f pcl_rotation;
    BoundingBox box;
    box.type = type;
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
        box.world_R_center = Eigen::Quaternionf(pcl_rotation);
        break;
      default:
        std::stringstream ss;
        ss << "Unknown bounding box type: " << static_cast<int>(type);
        throw std::runtime_error(ss.str());
    }

    return box;
  }

  // TODO(nathan) PCL switches to std::shared at some point.
  template <typename CloudT>
  static BoundingBox extract(const boost::shared_ptr<CloudT>& cloud,
                             Type type = Type::AABB) {
    if (!cloud) {
      throw std::runtime_error("invalid point cloud pointer");
    }
    return extract(boost::const_pointer_cast<const CloudT>(cloud), type);
  }
};

}  // namespace kimera
