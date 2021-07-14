#pragma once
#include "kimera_dsg/bounding_box.h"

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
    vector << j.at(0).get<Scalar>(), j.at(1).get<Scalar>(), j.at(2).get<Scalar>();
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

void to_json(nlohmann::json& j, const pcl::PolygonMesh& mesh);

void from_json(const nlohmann::json& j, pcl::PolygonMesh& mesh);

}  // namespace pcl

namespace kimera {

NLOHMANN_JSON_SERIALIZE_ENUM(BoundingBox::Type,
                             {
                                 {BoundingBox::Type::INVALID, nullptr},
                                 {BoundingBox::Type::AABB, "AABB"},
                                 {BoundingBox::Type::OBB, "OBB"},
                             });

void to_json(nlohmann::json& j, const BoundingBox& b);

void from_json(const nlohmann::json& j, BoundingBox& b);

}  // namespace kimera
