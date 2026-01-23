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
#include <gtest/gtest.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/traversability_boundary.h>

using State = spark_dsg::TraversabilityState;
using States = spark_dsg::TraversabilityStates;
using Vec = Eigen::Vector3d;
constexpr auto TRAV = State::TRAVERSABLE;
constexpr auto INTRAV = State::INTRAVERSABLE;
constexpr auto TRAVERSED = State::TRAVERSED;
constexpr auto UNKNOWN = State::UNKNOWN;

const std::vector<Eigen::Vector3d> offsets{
    {1.0, 0.0, 0.0},   // 0 deg
    {1.0, 1.0, 0.0},   // 45 deg
    {0.0, 1.0, 0.0},   // 90 deg
    {-1, 1, 0.0},      // 135 deg
    {-1.0, 0.0, 0.0},  // 180 deg
    {-1, -1, 0.0},     // 225 deg
    {0.0, -1.0, 0.0},  // 270 deg
    {1, -1, 0.0}       // 315 deg
};

TEST(RayBoundary, Angles) {
  // Simple 4-bin boundary
  spark_dsg::TravNodeAttributes boundary;
  boundary.radii.resize(4, 1.0);

  EXPECT_NEAR((boundary.getBoundaryPoint(0) - Vec(1.0, 0.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR((boundary.getBoundaryPoint(1) - Vec(0.0, 1.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR((boundary.getBoundaryPoint(2) - Vec(-1.0, 0.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR((boundary.getBoundaryPoint(3) - Vec(0.0, -1.0, 0.0)).norm(), 0.0, 1e-6);

  // Check world frame
  boundary.position = Vec(1.0, 1.0, 0.0);
  EXPECT_NEAR(
      (boundary.getBoundaryPoint(0, true) - Vec(2.0, 1.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR(
      (boundary.getBoundaryPoint(1, true) - Vec(1.0, 2.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR(
      (boundary.getBoundaryPoint(2, true) - Vec(0.0, 1.0, 0.0)).norm(), 0.0, 1e-6);
  EXPECT_NEAR(
      (boundary.getBoundaryPoint(3, true) - Vec(1.0, 0.0, 0.0)).norm(), 0.0, 1e-6);

  // Check bins.
  for (size_t i = 0; i < offsets.size(); ++i) {
    double bin_percentage = boundary.getBinPercentage(offsets[i]);
    size_t bin = boundary.getBin(offsets[i]);
    EXPECT_EQ(bin, static_cast<size_t>(i / 2));
    EXPECT_EQ(bin_percentage, i / 8.0);
  }
}

TEST(RayBoundary, Area) {
  spark_dsg::TravNodeAttributes boundary;

  boundary.radii.resize(4);
  EXPECT_NEAR(boundary.area(), 0.0, 1e-6);

  boundary.radii = std::vector<double>(4, 1.0);
  EXPECT_NEAR(boundary.area(), 2.0, 1e-6);

  boundary.radii.resize(8, 1.0);
  EXPECT_NEAR(boundary.area(), 2 * std::sqrt(2.0), 1e-6);

  boundary.radii.resize(10000, 1.0);
  EXPECT_NEAR(boundary.area(), M_PI, 1e-6);
}

TEST(RayBoundary, Contains) {
  spark_dsg::TravNodeAttributes boundary;
  boundary.radii = std::vector<double>(4, 1.0);
  boundary.min_radius = 0;  // Disable the radius check
  boundary.max_radius = 10;

  // Note that these are circle interpolations.
  EXPECT_TRUE(boundary.contains(Vec(1.0, 0.0, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(0.7, 0.7, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(0.0, 1.0, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(-0.7, 0.7, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(-0.7, -0.7, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(0.0, -1.0, 0.0)));
  EXPECT_TRUE(boundary.contains(Vec(0.7, -0.7, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(1.1, 0.0, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(0.8, 0.8, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(0.0, 1.1, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(-0.8, 0.8, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(-1.1, 0.0, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(-0.8, -0.8, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(0.0, -1.1, 0.0)));
  EXPECT_FALSE(boundary.contains(Vec(0.8, -0.8, 0.0)));
}