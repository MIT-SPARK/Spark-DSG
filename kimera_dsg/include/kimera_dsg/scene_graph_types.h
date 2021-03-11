#pragma once
#include <Eigen/Dense>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>
#include <pcl_ros/point_cloud.h>
#include <string>

namespace kimera {

//! Specifies the id of a node in a layer
typedef int64_t NodeId;
//! Specifies the id of a layer in the graph
typedef int64_t EdgeId;
typedef std::string NodeName;

/**
 * @brief The LayerId enum The numbering is important, as it determines the
 * parentesco: a higher number corresponds to parents.
 */
enum class LayerId {
  kInvalidLayerId = 0,
  kObjectsLayerId = 1,
  kAgentsLayerId = 2,
  kPlacesLayerId = 3,
  kRoomsLayerId = 4,
  kBuildingsLayerId = 5
};

inline std::string getStringFromLayerId(const LayerId& layer_id) {
  switch (layer_id) {
    case LayerId::kBuildingsLayerId:
      return "B";
    case LayerId::kRoomsLayerId:
      return "R";
    case LayerId::kPlacesLayerId:
      return "P";
    case LayerId::kObjectsLayerId:
      return "O";
    default:
      return "NA";
  }
}

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef pcl::PointXYZI IntensityPoint;
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

typedef uint8_t SemanticLabel; // TODO(nathan) take this directly from kimera_semantics
typedef uint64_t Timestamp;

enum class BoundingBoxType { kAABB = 0, kOBB = 1 };

template <class PointT>
struct BoundingBox {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BoundingBoxType type_ = BoundingBoxType::kAABB;
  PointT max_ = PointT();
  PointT min_ = PointT();
  // Position and rotation is only used for BB type of bb box.
  PointT position_ = PointT();
  Eigen::Matrix3f orientation_matrix = Eigen::Matrix3f();

 public:
  std::string print() const;

 private:
  friend std::ostream& operator<<(std::ostream& os,
                                  const BoundingBox& bounding_box) {
    os << " - max_ " << bounding_box.max_ << '\n'
       << " - min_ " << bounding_box.min_ << '\n'
       << " - position_ " << bounding_box.position_ << '\n';
    return os;
  };
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    // TODO(nathan) consider serializing type
    ar& BOOST_SERIALIZATION_NVP(max_);
    ar& BOOST_SERIALIZATION_NVP(min_);
    ar& BOOST_SERIALIZATION_NVP(position_);
  }
};

template <class PointT>
std::string BoundingBox<PointT>::print() const {
  std::stringstream ss;
  ss << "BoundingBox: " << '\n'
     << " - max_ " << max_ << '\n'
     << " - min_ " << min_ << '\n'
     << " - position_ " << position_ << '\n';
  return ss.str();
}

// Add way of printing strongly typed enums (enum class).
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

}  // namespace kimera
