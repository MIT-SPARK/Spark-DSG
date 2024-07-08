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
#include "spark_dsg/bounding_box_extraction.h"

#include <optional>

namespace spark_dsg {
namespace bounding_box {

float getAngle(const Eigen::Vector3f& curr, const Eigen::Vector3f& root) {
  const Eigen::Vector2f vec = (curr.head<2>() - root.head<2>()).normalized();
  // range of x should be between 1 and -1 (with y >= 0)
  // we want function mapping [1, -1] to [0, c]
  return 1.0f - vec.x();
}

float getDist(const Eigen::Vector3f& curr, const Eigen::Vector3f& root) {
  return (curr.head<2>() - root.head<2>()).array().abs().sum();
}

float getJointDirection(const Eigen::Vector3f& prev,
                        const Eigen::Vector3f& curr,
                        const Eigen::Vector3f& next) {
  const Eigen::Vector2f v1 = curr.head<2>() - prev.head<2>();
  const Eigen::Vector2f v2 = next.head<2>() - prev.head<2>();
  return v1.x() * v2.y() - v1.y() * v2.x();
}

std::list<size_t> get2dConvexHull(const PointAdaptor& points) {
  size_t root = 0;
  Eigen::Vector3f root_pos = points[0];
  for (size_t i = 0; i < points.size(); ++i) {
    const auto curr_pos = points[i];
    if (curr_pos.y() > root_pos.y()) {
      continue;
    }

    if (curr_pos.y() == root_pos.y() && curr_pos.x() > root_pos.x()) {
      continue;
    }

    root_pos = curr_pos;
    root = i;
  }

  const auto compare = [&](size_t i, size_t j) -> bool {
    const auto p_i = points[i];
    const auto p_j = points[j];
    const auto a_i = getAngle(p_i, root_pos);
    const auto a_j = getAngle(p_j, root_pos);
    if (std::abs(a_i - a_j) < 1.0e-9f) {
      const auto d_i = getDist(p_i, root_pos);
      const auto d_j = getDist(p_j, root_pos);
      return d_i < d_j;
    } else {
      return a_i < a_j;
    }
  };

  std::vector<size_t> indices;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i != root) {
      indices.push_back(i);
    }
  }

  std::sort(indices.begin(), indices.end(), compare);

  std::list<size_t> hull{root};
  for (const auto& idx : indices) {
    while (hull.size() > 1) {
      auto prev = --hull.cend();
      auto curr = prev;
      --curr;
      const auto angle = getJointDirection(points[*prev], points[*curr], points[idx]);
      if (angle < 0.0f) {
        break;
      }

      hull.pop_back();
    }

    hull.push_back(idx);
  }

  return hull;
}

struct BoxResult2D {
  Eigen::Vector2f x_min = Eigen::Vector2f::Zero();
  Eigen::Vector2f x_max = Eigen::Vector2f::Zero();
  std::optional<float> min_area;
  float yaw = 0.0f;
};

BoxResult2D getMin2DBox(const PointAdaptor& points, const std::list<size_t>& hull) {
  BoxResult2D result;
  std::vector<size_t> indices(hull.begin(), hull.end());
  // technically this can be implemented in O(n) instead via rotation calipers,
  // but this is easier to understand and n << points.size() due to 2d projection
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto curr_idx = indices[i];
    const auto next_idx = indices[(i + 1) % indices.size()];
    const Eigen::Vector2f x_c = points[curr_idx].head<2>();
    const Eigen::Vector2f x_n = points[next_idx].head<2>();
    // normals and offsets for height / width hyperplanes
    const Eigen::Vector2f n_h = (x_n - x_c).normalized();
    const Eigen::Vector2f n_w(-n_h.y(), n_h.x());  // equivalent to a 90 degree rotation
    const auto b_w = -n_w.dot(x_c);
    const auto b_h = -n_h.dot(x_c);

    // distances from hyperplanes
    float max_w = 0.0f;
    float min_h = 0.0f;
    float max_h = 0.0f;
    for (size_t j = 1; j < indices.size(); ++j) {
      const Eigen::Vector2f x_j = points[indices[(i + j) % indices.size()]].head<2>();
      const auto w_dist = n_w.dot(x_j) + b_w;
      const auto h_dist = n_h.dot(x_j) + b_h;
      if (w_dist >= max_w) {
        max_w = w_dist;
      }

      if (h_dist <= min_h) {
        min_h = h_dist;
      }

      if (h_dist >= max_h) {
        max_h = h_dist;
      }
    }

    // technically (max_w - min_w) * (max_h - min_h) but min_w is 0
    const auto area = max_w * (max_h - min_h);
    if (result.min_area && area >= *result.min_area) {
      continue;
    }

    result.min_area = area;
    Eigen::Matrix2f A;
    A.row(0) = n_w.transpose();
    A.row(1) = n_h.transpose();
    const auto A_inv = A.inverse();
    // hyperplane along normal d dist away is n * x + b - d
    // we want lower right corner
    const Eigen::Vector2f b_low(b_w, b_h - min_h);
    // we want upper left corner
    const Eigen::Vector2f b_high(b_w - max_w, b_h - max_h);
    result.x_min = -A_inv * b_low;
    result.x_max = -A_inv * b_high;
    result.yaw = std::atan2(n_h.y(), n_h.x());
  }

  return result;
}

BoundingBox extractAABB(const PointAdaptor& points) {
  Eigen::Vector3f min = points[0];
  Eigen::Vector3f max = min;
  for (size_t i = 1; i < points.size(); ++i) {
    min = min.array().min(points[i].array());
    max = max.array().max(points[i].array());
  }
  return BoundingBox(max - min, (min + max) / 2.0f);
}

BoundingBox extractOBB(const PointAdaptor&) { return {}; }

BoundingBox extractRAABB(const PointAdaptor& points) {
  auto hull = get2dConvexHull(points);
  const auto min_2d_box = getMin2DBox(points, hull);
  if (!min_2d_box.min_area) {
    return {};
  }

  float min_z = points[0].z();
  float max_z = min_z;
  for (size_t i = 1; i < points.size(); ++i) {
    const auto curr_z = points[i].z();
    min_z = std::min(curr_z, min_z);
    max_z = std::max(curr_z, max_z);
  }

  const auto& yaw = min_2d_box.yaw;
  Eigen::Vector3f p_min;
  p_min << min_2d_box.x_min, min_z;
  Eigen::Vector3f p_max;
  p_max << min_2d_box.x_max, max_z;

  BoundingBox result(Eigen::Vector3f::Zero(), (p_min + p_max) / 2.0f, yaw);
  result.dimensions = result.pointToBoxFrame(p_max) - result.pointToBoxFrame(p_min);
  return result;
}

BoundingBox extract(const PointAdaptor& points, BoundingBox::Type type) {
  if (points.size() == 0 || type == BoundingBox::Type::INVALID) {
    return {};
  }

  switch (type) {
    case BoundingBox::Type::AABB:
      return extractAABB(points);
    case BoundingBox::Type::OBB:
      return extractOBB(points);
    case BoundingBox::Type::RAABB:
      return extractRAABB(points);
    default:
      return {};
  }
}

}  // namespace bounding_box

}  // namespace spark_dsg
