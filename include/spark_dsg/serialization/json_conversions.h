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

#include <nlohmann/json.hpp>

#include "spark_dsg/bounding_box.h"

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

struct NearestVertexInfo;
void to_json(nlohmann::json& j, const NearestVertexInfo& b);
void from_json(const nlohmann::json& j, NearestVertexInfo& b);

struct Color;
void to_json(nlohmann::json& record, const Color& c);
void from_json(const nlohmann::json& record, Color& c);

class Mesh;
void to_json(nlohmann::json& j, const Mesh& mesh);
void from_json(const nlohmann::json& j, Mesh& mesh);

struct NodeAttributes;
void to_json(nlohmann::json& j, const NodeAttributes& attrs);

struct EdgeAttributes;
void to_json(nlohmann::json& j, const EdgeAttributes& attrs);

namespace io {

struct FileHeader;
void to_json(nlohmann::json& record, const FileHeader& header);
void from_json(const nlohmann::json& record, FileHeader& header);

}  // namespace io

}  // namespace spark_dsg

namespace nlohmann {

template <typename Scalar, int Rows, int Cols>
struct adl_serializer<Eigen::Matrix<Scalar, Rows, Cols>> {
  static void to_json(json& j, const Eigen::Matrix<Scalar, Rows, Cols>& mat) {
    json* vec = &j;
    if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic) {
      j["rows"] = mat.rows();
      j["cols"] = mat.cols();
      vec = &j["data"];
    }

    for (int r = 0; r < mat.rows(); ++r) {
      for (int c = 0; c < mat.cols(); ++c) {
        vec->push_back(mat(r, c));
      }
    }
  }

  static void from_json(const json& j, Eigen::Matrix<Scalar, Rows, Cols>& mat) {
    const auto* vec = &j;
    int rows = Rows;
    int cols = Cols;
    if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic) {
      rows = j.at("rows").get<int>();
      cols = j.at("cols").get<int>();
      vec = &(j.at("data"));
    } else if (Rows == Eigen::Dynamic) {
      rows = j.size() / Cols;
    } else if (Cols == Eigen::Dynamic) {
      cols = j.size() / Rows;
    }

    if (vec->size() != static_cast<size_t>(rows * cols)) {
      std::stringstream ss;
      ss << "cannot decode matrix: [" << rows << ", " << cols << "] from "
         << vec->size() << " values";
      throw std::runtime_error(ss.str());
    }

    mat = Eigen::Matrix<Scalar, Rows, Cols>(rows, cols);
    for (size_t i = 0; i < vec->size(); ++i) {
      int r = i / cols;
      int c = i % cols;
      const auto& value = vec->at(i);
      mat(r, c) = value.is_null() ? std::numeric_limits<Scalar>::quiet_NaN()
                                  : value.get<Scalar>();
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
