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

#include <algorithm>
#include <functional>
#include <iterator>
#include <numeric>
#include <optional>

namespace spark_dsg::bounding_box {
namespace {

bool comparePoints(const PointAdaptor& points, size_t i, size_t j) {
  const auto p_i = points[i];
  const auto p_j = points[j];
  return p_i.x() < p_j.x() || (p_i.x() == p_j.x() && p_i.y() < p_j.y());
}

bool equalPointsXY(const PointAdaptor& points, size_t i, size_t j) {
  return points[i].head<2>() == points[j].head<2>();
}

double getJointDirection(const Eigen::Vector3f& prev,
                         const Eigen::Vector3f& curr,
                         const Eigen::Vector3f& next) {
  const Eigen::Vector2d root = prev.head<2>().cast<double>();
  const Eigen::Vector2d v1 = curr.head<2>().cast<double>() - root;
  const Eigen::Vector2d v2 = next.head<2>().cast<double>() - root;
  return v1.x() * v2.y() - v1.y() * v2.x();
}

void appendHullPoint(const PointAdaptor& points,
                     size_t index,
                     size_t begin,
                     std::vector<size_t>& hull) {
  while (hull.size() >= begin + 2) {
    const auto prev = hull[hull.size() - 2];
    const auto curr = hull.back();
    const auto direction = getJointDirection(points[prev], points[curr], points[index]);
    if (direction > 0.0) {
      break;
    }

    hull.pop_back();
  }

  hull.push_back(index);
}

}  // namespace

std::vector<size_t> get2dConvexHull(const PointAdaptor& points) {
  using namespace std::placeholders;
  std::vector<size_t> indices(points.size());
  std::iota(indices.begin(), indices.end(), size_t{0});
  const auto compare = std::bind(comparePoints, std::cref(points), _1, _2);
  std::sort(indices.begin(), indices.end(), compare);
  const auto equal = std::bind(equalPointsXY, std::cref(points), _1, _2);
  indices.erase(std::unique(indices.begin(), indices.end(), equal), indices.end());
  if (indices.size() <= 1) {
    return {indices.begin(), indices.end()};
  }

  std::vector<size_t> hull;
  hull.reserve(points.size());
  for (const auto index : indices) {
    appendHullPoint(points, index, 0, hull);
  }

  const auto upper_begin = hull.size() - 1;
  for (auto iter = std::next(indices.rbegin()); iter != indices.rend(); ++iter) {
    appendHullPoint(points, *iter, upper_begin, hull);
  }

  hull.pop_back();  // The first point closes both chains.
  return hull;
}

BoxResult2D getMin2DBox(const PointAdaptor& points, const std::vector<size_t>& hull) {
  std::vector<size_t> indices;
  if (hull.empty()) {
    indices = get2dConvexHull(points);
  } else {
    indices = hull;
  }

  BoxResult2D result;
  if (indices.size() <= 1) {
    return result;
  }

  if (indices.size() == 2) {
    const Eigen::Vector2d a = points[indices.front()].head<2>().cast<double>();
    const Eigen::Vector2d b = points[indices.back()].head<2>().cast<double>();
    const Eigen::Vector2d edge = b - a;
    result.min_area = 0.0f;
    result.dims << edge.norm(), 0.0f;
    result.center = (a + 0.5 * edge).cast<float>();
    result.yaw = std::atan2(edge.y(), edge.x());
    return result;
  }

  // technically this can be implemented in O(n) instead via rotation calipers,
  // but this is easier to understand and n << points.size() due to 2d projection
  for (size_t i = 0; i < indices.size(); ++i) {
    const auto curr_idx = indices[i];
    const auto next_idx = indices[(i + 1) % indices.size()];
    const Eigen::Vector2d p_c = points[curr_idx].head<2>().cast<double>();
    const Eigen::Vector2d p_n = points[next_idx].head<2>().cast<double>();
    const Eigen::Vector2d edge = p_n - p_c;
    if (edge.squaredNorm() == 0.0) {
      continue;
    }

    Eigen::Matrix2d R;
    R.col(0) = edge.normalized();
    R.col(1) = Eigen::Vector2d(-R(1, 0), R(0, 0));
    Eigen::Vector2d min = Eigen::Vector2d::Zero();
    Eigen::Vector2d max = Eigen::Vector2d::Zero();
    for (const auto index : indices) {
      const Eigen::Vector2d offset = points[index].head<2>().cast<double>() - p_c;
      const Eigen::Vector2d local = R.transpose() * offset;
      min = min.cwiseMin(local);
      max = max.cwiseMax(local);
    }

    const Eigen::Vector2d dims = max - min;
    const auto area = dims.prod();
    if (result.min_area && area >= *result.min_area) {
      continue;
    }

    result.min_area = area;
    result.dims = dims.cast<float>();
    result.yaw = std::atan2(R(1, 0), R(0, 0));
    // transform center point to global coordinates
    result.center = (R * (0.5 * (min + max)) + p_c).cast<float>();
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

  Eigen::Vector3f dims;
  dims << min_2d_box.dims, max_z - min_z;
  Eigen::Vector3f center;
  center << min_2d_box.center, 0.5f * (max_z + min_z);
  return BoundingBox(dims, center, min_2d_box.yaw);
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

}  // namespace spark_dsg::bounding_box
