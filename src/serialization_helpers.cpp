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
#include "spark_dsg/serialization_helpers.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using nlohmann::json;

namespace pcl {

void to_json(json& j, const pcl::PointXYZRGBA& point) {
  j = json{{"x", point.x},
           {"y", point.y},
           {"z", point.z},
           {"r", point.r},
           {"g", point.g},
           {"b", point.b}};
}

void from_json(const json& j, pcl::PointXYZRGBA& point) {
  point.x = j.at("x").is_null() ? std::numeric_limits<decltype(point.x)>::quiet_NaN()
                                : j.at("x").get<decltype(point.x)>();
  point.y = j.at("y").is_null() ? std::numeric_limits<decltype(point.y)>::quiet_NaN()
                                : j.at("y").get<decltype(point.y)>();
  point.z = j.at("z").is_null() ? std::numeric_limits<decltype(point.z)>::quiet_NaN()
                                : j.at("z").get<decltype(point.z)>();
  point.r = j.at("r").get<decltype(point.r)>();
  point.g = j.at("g").get<decltype(point.g)>();
  point.b = j.at("b").get<decltype(point.b)>();
}

void to_json(json& j, const pcl::PointCloud<pcl::PointXYZRGBA>& vertices) {
  j = json::array();
  for (const auto& vertex : vertices) {
    json vertex_json = vertex;
    j.push_back(vertex_json);
  }
}

void from_json(const json& j, pcl::PointCloud<pcl::PointXYZRGBA>& vertices) {
  for (const auto& vertex : j) {
    vertices.push_back(vertex.get<pcl::PointXYZRGBA>());
  }
}

void to_json(json& j, const std::vector<pcl::Vertices>& faces) {
  j = json::array();
  for (const auto& face : faces) {
    j.push_back(face.vertices);
  }
}

void from_json(const json& j, std::vector<pcl::Vertices>& faces) {
  for (const auto& face : j) {
    pcl::Vertices face_indices;
    face_indices.vertices = face.get<decltype(face_indices.vertices)>();
    faces.push_back(face_indices);
  }
}

}  // namespace pcl

namespace spark_dsg {

void to_json(json& j, const BoundingBox& b) {
  j = json{{"type", b.type},
           {"min", b.min},
           {"max", b.max},
           {"world_P_center", b.world_P_center},
           {"world_R_center", Eigen::Quaternionf(b.world_R_center)}};
}

void from_json(const json& j, BoundingBox& b) {
  if (j.at("type").is_null()) {
    b.type = BoundingBox::Type::RAABB;
  } else {
    b.type = j.at("type").get<BoundingBox::Type>();
  }

  if (b.type == BoundingBox::Type::INVALID) {
    return;
  }

  b.min = j.at("min").get<Eigen::Vector3f>();
  b.max = j.at("max").get<Eigen::Vector3f>();
  b.world_P_center = j.at("world_P_center").get<Eigen::Vector3f>();
  auto world_q_center = j.at("world_R_center").get<Eigen::Quaternionf>();
  b.world_R_center = world_q_center.toRotationMatrix();
}

void to_json(json& j, const NearestVertexInfo& info) {
  j = json{
      {"block", info.block}, {"voxel_pos", info.voxel_pos}, {"vertex", info.vertex}};
}

void from_json(const json& j, NearestVertexInfo& info) {
  info.block[0] = j.at("block").at(0).get<int32_t>();
  info.block[1] = j.at("block").at(1).get<int32_t>();
  info.block[2] = j.at("block").at(2).get<int32_t>();
  info.voxel_pos[0] = j.at("voxel_pos").at(0).get<double>();
  info.voxel_pos[1] = j.at("voxel_pos").at(1).get<double>();
  info.voxel_pos[2] = j.at("voxel_pos").at(2).get<double>();
  info.vertex = j.at("vertex");
}

}  // namespace spark_dsg
