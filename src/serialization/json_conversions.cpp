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
#include "spark_dsg/serialization/json_conversions.h"

#include <fstream>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/mesh.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using nlohmann::json;

void to_json(json& j, const BoundingBox& b) {
  j = json{{"type", b.type},
           {"dimensions", b.dimensions},
           {"world_P_center", b.world_P_center},
           {"world_R_center", Eigen::Quaternionf(b.world_R_center)}};
}

void from_json(const json& j, BoundingBox& b) {
  if (j.at("type").is_null()) {
    b.type = BoundingBox::Type::INVALID;
  } else {
    b.type = j.at("type").get<BoundingBox::Type>();
  }

  if (b.type == BoundingBox::Type::INVALID) {
    return;
  }
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 0, 3)) {
    // Legacy bboxes with min/max encoding.
    b.dimensions =
        j.at("max").get<Eigen::Vector3f>() - j.at("min").get<Eigen::Vector3f>();
    io::warnOutdatedHeader(header);
  } else {
    // New bboxes with dimensions encoding.
    b.dimensions = j.at("dimensions").get<Eigen::Vector3f>();
  }

  b.world_P_center = j.at("world_P_center").get<Eigen::Vector3f>();
  auto world_q_center = j.at("world_R_center").get<Eigen::Quaternionf>();
  b.world_R_center = world_q_center.toRotationMatrix();
}

void to_json(json& j, const NearestVertexInfo& info) {
  j = json{
      {"block", info.block}, {"voxel_pos", info.voxel_pos}, {"vertex", info.vertex}};

  if (info.label) {
    j["label"] = info.label.value();
  } else {
    j["label"] = nullptr;
  }
}

void from_json(const json& j, NearestVertexInfo& info) {
  info.block[0] = j.at("block").at(0).get<int32_t>();
  info.block[1] = j.at("block").at(1).get<int32_t>();
  info.block[2] = j.at("block").at(2).get<int32_t>();
  info.voxel_pos[0] = j.at("voxel_pos").at(0).get<double>();
  info.voxel_pos[1] = j.at("voxel_pos").at(1).get<double>();
  info.voxel_pos[2] = j.at("voxel_pos").at(2).get<double>();
  info.vertex = j.at("vertex");

  if (j.contains("label") && !j.at("label").is_null()) {
    info.label = j.at("label").get<uint32_t>();
  }
}

void to_json(json& record, const Color& c) {
  record["r"] = c.r;
  record["g"] = c.g;
  record["b"] = c.b;
  record["a"] = c.a;
}

void from_json(const json& record, Color& c) {
  // Support scene graphs 1.0.2 and earlier where colors were encoded as vectors.
  // TODO(lschmid): Remove this in the future. 
  if (record.is_array()) {
    io::warnOutdatedHeader(io::GlobalInfo::loadedHeader());
    c.r = record.at(0).get<uint8_t>();
    c.g = record.at(1).get<uint8_t>();
    c.b = record.at(2).get<uint8_t>();
    if (record.size() > 3) {
      c.a = record.at(3).get<uint8_t>();
    } else {
      c.a = 255;
    }
    return;
  }
  c.r = record.at("r").get<uint8_t>();
  c.g = record.at("g").get<uint8_t>();
  c.b = record.at("b").get<uint8_t>();
  if (record.contains("a")) {
    c.a = record.at("a").get<uint8_t>();
  }
}

// TODO(nathan) think about mesh serialization

void to_json(json& record, const NodeAttributes& attributes) {
  serialization::Visitor::to(record, attributes);
}

void to_json(json& record, const EdgeAttributes& attributes) {
  serialization::Visitor::to(record, attributes);
}

namespace io {

void to_json(json& record, const FileHeader& header) {
  record = {{"project_name", header.project_name},
            {"version",
             {{"major", header.version.major},
              {"minor", header.version.minor},
              {"patch", header.version.patch}}}};
}

void from_json(const json& record, FileHeader& header) {
  header.project_name = record.at("project_name").get<std::string>();
  header.version.major = record.at("version").at("major").get<uint8_t>();
  header.version.minor = record.at("version").at("minor").get<uint8_t>();
  header.version.patch = record.at("version").at("patch").get<uint8_t>();
}

}  // namespace io

}  // namespace spark_dsg
