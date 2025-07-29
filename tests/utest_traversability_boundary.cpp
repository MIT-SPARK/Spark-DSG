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
#include <spark_dsg/traversability_boundary.h>

using spark_dsg::Boundary;
using spark_dsg::Side;
using spark_dsg::TraversabilityState;
using spark_dsg::TraversabilityStates;

TEST(TraversabilityBoundary, IntersectsSide) {
  Boundary boundary({0, 0}, {10, 10});

  // Check simple sides.
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(5, -1)), Side::BOTTOM);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(-1, 5)), Side::LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(5, 11)), Side::TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(11, 5)), Side::RIGHT);

  // Check more general rectangles.
  boundary = Boundary({2, -5}, {4, 5});
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(2, -5.1)), Side::BOTTOM);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(1.9, -5)), Side::LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(1.9, 5)), Side::LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(2, 5.1)), Side::TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4, 5.1)), Side::TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4.1, 5)), Side::RIGHT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4.1, -5)), Side::RIGHT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4, -5.1)), Side::BOTTOM);
}

TEST(TraversabilityBoundary, IsOnSide) {
  Boundary boundary({0, 0}, {10, 10});

  // Intersects.
  EXPECT_EQ(boundary.isOnSide(Boundary({5, 5}, {15, 15})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {5, 5})), Side::INVALID);

  // Bottom side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {5, -1})), Side::BOTTOM);
  EXPECT_EQ(boundary.isOnSide(Boundary({5, -5}, {15, -1})), Side::BOTTOM);

  // Left side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {-1, 5})), Side::LEFT);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 5}, {-1, 15})), Side::LEFT);

  // Top side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 11}, {5, 15})), Side::TOP);
  EXPECT_EQ(boundary.isOnSide(Boundary({5, 11}, {15, 15})), Side::TOP);

  // Right side.
  EXPECT_EQ(boundary.isOnSide(Boundary({11, -5}, {15, 5})), Side::RIGHT);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, 5}, {15, 15})), Side::RIGHT);

  // Corners.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {-1, -1})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 11}, {-1, 15})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, -5}, {15, -1})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, 11}, {15, 15})), Side::INVALID);
}