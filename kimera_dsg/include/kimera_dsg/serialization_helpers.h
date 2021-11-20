#pragma once
#include "kimera_dsg/bounding_box.h"
#include "kimera_dsg/node_attributes.h"

#include <pcl/PolygonMesh.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace nlohmann {

template <typename Scalar>
struct adl_serializer<Eigen::Matrix<Scalar, 3, 1>> {
  static void to_json(json& j, const Eigen::Matrix<Scalar, 3, 1>& vector) {
    j = json{vector.x(), vector.y(), vector.z()};
  }

  static void from_json(const json& j, Eigen::Matrix<Scalar, 3, 1>& vector) {
    for (size_t i = 0; i < 3; ++i) {
      vector(i) = j.at(i).is_null() ? std::numeric_limits<Scalar>::quiet_NaN()
                                    : j.at(i).get<Scalar>();
    }
  }
};

template <typename Scalar>
struct adl_serializer<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> {
  static void to_json(json& j, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vector) {
    for (int r = 0; r < vector.rows(); ++r) {
      j.push_back(vector(r, 0));
    }
  }

  static void from_json(const json& j,
                        Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vector) {
    vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(j.size(), 1);
    for (size_t r = 0; r < j.size(); ++r) {
      vector(r, 0) = j.at(r).get<Scalar>();
    }
  }
};

template <typename Scalar>
struct adl_serializer<Eigen::Quaternion<Scalar>> {
  static void to_json(json& j, const Eigen::Quaternion<Scalar>& q) {
    j = json{{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
  }

  static void from_json(const json& j, Eigen::Quaternion<Scalar>& q) {
    q = Eigen::Quaternion<Scalar>(j.at("w").get<Scalar>(),
                                  j.at("x").get<Scalar>(),
                                  j.at("y").get<Scalar>(),
                                  j.at("z").get<Scalar>());
  }
};

}  // namespace nlohmann

namespace pcl {

void to_json(nlohmann::json& j, const pcl::PointCloud<pcl::PointXYZRGBA>& vertices);

void to_json(nlohmann::json& j, const std::vector<pcl::Vertices>& faces);

void to_json(nlohmann::json& j, const pcl::PointXYZRGBA& point);

void from_json(const nlohmann::json& j, pcl::PointCloud<pcl::PointXYZRGBA>& vertices);

void from_json(const nlohmann::json& j, std::vector<pcl::Vertices>& faces);

void from_json(const nlohmann::json& j, pcl::PointXYZRGBA& point);

}  // namespace pcl

namespace kimera {

NLOHMANN_JSON_SERIALIZE_ENUM(BoundingBox::Type,
                             {
                                 {BoundingBox::Type::INVALID, nullptr},
                                 {BoundingBox::Type::AABB, "AABB"},
                                 {BoundingBox::Type::RAABB, "RAABB"},
                                 {BoundingBox::Type::OBB, "OBB"},
                             });

void to_json(nlohmann::json& j, const BoundingBox& b);

void from_json(const nlohmann::json& j, BoundingBox& b);

void to_json(nlohmann::json& j, const NearestVertexInfo& b);

void from_json(const nlohmann::json& j, NearestVertexInfo& b);

}  // namespace kimera
