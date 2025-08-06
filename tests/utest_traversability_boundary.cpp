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
using State = spark_dsg::TraversabilityState;
using States = spark_dsg::TraversabilityStates;

Boundary defaultBoundary() {
  return Boundary({0, 0},
                  {10, 10},
                  {States(10, State::TRAVERSABLE),
                   States(10, State::TRAVERSABLE),
                   States(10, State::TRAVERSABLE),
                   States(10, State::TRAVERSABLE)});
}

TEST(TraversabilityBoundary, IntersectsSide) {
  Boundary boundary = defaultBoundary();

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
  Boundary boundary = defaultBoundary();

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

TEST(TraversabilityBoundary, Voxels) {
  Boundary boundary = defaultBoundary();

  // Voxel size.
  EXPECT_DOUBLE_EQ(boundary.voxelSize(Side::BOTTOM), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(Side::LEFT), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(Side::TOP), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(Side::RIGHT), 1.0);

  // Set coordinates: Shrink
  boundary.setCoordinate(Side::BOTTOM, 2.0);
  EXPECT_DOUBLE_EQ(boundary.min.y(), 2.0);
  EXPECT_EQ(boundary.states[Side::LEFT].size(), 8);
  EXPECT_EQ(boundary.states[Side::RIGHT].size(), 8);
  EXPECT_EQ(boundary.states[Side::LEFT], States(8, State::TRAVERSABLE));
  EXPECT_EQ(boundary.states[Side::RIGHT], States(8, State::TRAVERSABLE));
  EXPECT_EQ(boundary.voxelSize(Side::BOTTOM), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::LEFT), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::TOP), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::RIGHT), 1.0);

  boundary.setCoordinate(Side::LEFT, 3.0);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 3.0);
  EXPECT_EQ(boundary.states[Side::BOTTOM].size(), 7);
  EXPECT_EQ(boundary.states[Side::TOP].size(), 7);
  EXPECT_EQ(boundary.states[Side::BOTTOM], States(7, State::TRAVERSABLE));
  EXPECT_EQ(boundary.states[Side::TOP], States(7, State::TRAVERSABLE));
  EXPECT_EQ(boundary.voxelSize(Side::BOTTOM), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::LEFT), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::TOP), 1.0);
  EXPECT_EQ(boundary.voxelSize(Side::RIGHT), 1.0);

  boundary.setCoordinate(Side::TOP, 8.0);
  EXPECT_DOUBLE_EQ(boundary.max.y(), 8.0);
  EXPECT_EQ(boundary.states[Side::LEFT].size(), 6);
  EXPECT_EQ(boundary.states[Side::RIGHT].size(), 6);
  EXPECT_EQ(boundary.states[Side::LEFT], States(6, State::TRAVERSABLE));
  EXPECT_EQ(boundary.states[Side::RIGHT], States(6, State::TRAVERSABLE));

  boundary.setCoordinate(Side::RIGHT, 7.0);
  EXPECT_DOUBLE_EQ(boundary.max.x(), 7.0);
  EXPECT_EQ(boundary.states[Side::BOTTOM].size(), 4);
  EXPECT_EQ(boundary.states[Side::TOP].size(), 4);
  EXPECT_EQ(boundary.states[Side::BOTTOM], States(4, State::TRAVERSABLE));
  EXPECT_EQ(boundary.states[Side::TOP], States(4, State::TRAVERSABLE));

  // Set coordinates: Extend
  boundary.setCoordinate(Side::BOTTOM, -1.0);  // 2 --> -1
  EXPECT_DOUBLE_EQ(boundary.min.y(), -1.0);
  EXPECT_EQ(boundary.states[Side::LEFT].size(), 9);
  EXPECT_EQ(boundary.states[Side::RIGHT].size(), 9);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(boundary.states[Side::LEFT][i], State::UNKNOWN);
    EXPECT_EQ(boundary.states[Side::RIGHT][i], State::UNKNOWN);
  }
  for (size_t i = 3; i < 9; ++i) {
    EXPECT_EQ(boundary.states[Side::LEFT][i], State::TRAVERSABLE);
    EXPECT_EQ(boundary.states[Side::RIGHT][i], State::TRAVERSABLE);
  }

  boundary.setCoordinate(Side::LEFT, 1.0);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 1.0);
  EXPECT_EQ(boundary.states[Side::BOTTOM].size(), 6);
  EXPECT_EQ(boundary.states[Side::TOP].size(), 6);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(boundary.states[Side::BOTTOM][i], State::UNKNOWN);
    EXPECT_EQ(boundary.states[Side::TOP][i], State::UNKNOWN);
  }
  for (size_t i = 3; i < 6; ++i) {
    EXPECT_EQ(boundary.states[Side::BOTTOM][i], State::TRAVERSABLE);
    EXPECT_EQ(boundary.states[Side::TOP][i], State::TRAVERSABLE);
  }

  boundary.setCoordinate(Side::TOP, 9.0);
  EXPECT_DOUBLE_EQ(boundary.max.y(), 9.0);
  EXPECT_EQ(boundary.states[Side::LEFT].size(), 10);
  EXPECT_EQ(boundary.states[Side::RIGHT].size(), 10);
  EXPECT_EQ(boundary.states[Side::LEFT][9], State::UNKNOWN);
  EXPECT_EQ(boundary.states[Side::RIGHT][9], State::UNKNOWN);

  boundary.setCoordinate(Side::RIGHT, 10.0);
  EXPECT_DOUBLE_EQ(boundary.max.x(), 10.0);
  EXPECT_EQ(boundary.states[Side::BOTTOM].size(), 9);
  EXPECT_EQ(boundary.states[Side::TOP].size(), 9);
  EXPECT_EQ(boundary.states[Side::BOTTOM][8], State::UNKNOWN);
  EXPECT_EQ(boundary.states[Side::TOP][8], State::UNKNOWN);

  // Round voxel sizes.
  boundary = defaultBoundary();
  boundary.setCoordinate(Side::BOTTOM, 2.45);
  EXPECT_DOUBLE_EQ(boundary.min.y(), 2);
  EXPECT_EQ(boundary.states[Side::LEFT].size(), 8);
  EXPECT_EQ(boundary.states[Side::RIGHT].size(), 8);

  boundary.setCoordinate(Side::LEFT, 3.55);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 4);
  EXPECT_EQ(boundary.states[Side::BOTTOM].size(), 6);
  EXPECT_EQ(boundary.states[Side::TOP].size(), 6);
}

TEST(TraversabilityBoundary, MaxTraversableDistance) {
  Boundary from = defaultBoundary();
  Boundary to = from;

  // Perfect overlap.
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 10.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 10.0);

  // Partial overlap.
  to.min = Eigen::Vector2d(2, 2);
  to.max = Eigen::Vector2d(8, 8);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 6.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 6.0);

  // Overflow.
  to.max = Eigen::Vector2d(15, 15);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 8.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 8.0);

  // With intraversable states.
  to = from;
  for (size_t i = 0; i < 5; ++i) {
    from.states[Side::BOTTOM][i] = State::INTRAVERSABLE;
    from.states[Side::LEFT][9 - i] = State::INTRAVERSABLE;
  }
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 5.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 5.0);

  // With offset.
  to.min = Eigen::Vector2d(2, 2);
  to.max = Eigen::Vector2d(12, 12);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 5.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 3.0);

  to.min = Eigen::Vector2d(-2, -2);
  to.max = Eigen::Vector2d(8, 8);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::BOTTOM), 3.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, Side::LEFT), 5.0);
}
