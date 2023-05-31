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
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "spark_dsg/bounding_box.h"
#include "spark_dsg/node_attributes.h"

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

namespace spark_dsg {

NLOHMANN_JSON_SERIALIZE_ENUM(BoundingBox::Type,
                             {
                                 {BoundingBox::Type::INVALID, "INVALID"},
                                 {BoundingBox::Type::AABB, "AABB"},
                                 {BoundingBox::Type::RAABB, "RAABB"},
                                 {BoundingBox::Type::OBB, "OBB"},
                             });

void to_json(nlohmann::json& j, const BoundingBox& b);

void from_json(const nlohmann::json& j, BoundingBox& b);

void to_json(nlohmann::json& j, const NearestVertexInfo& b);

void from_json(const nlohmann::json& j, NearestVertexInfo& b);

}  // namespace spark_dsg
