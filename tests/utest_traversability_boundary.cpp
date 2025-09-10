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
using spark_dsg::Boundary;
using spark_dsg::Side;
using State = spark_dsg::TraversabilityState;
using States = spark_dsg::TraversabilityStates;
constexpr auto TOP = Side::TOP;
constexpr auto BOTTOM = Side::BOTTOM;
constexpr auto LEFT = Side::LEFT;
constexpr auto RIGHT = Side::RIGHT;
constexpr auto TRAV = State::TRAVERSABLE;
constexpr auto INTRAV = State::INTRAVERSABLE;
constexpr auto TRAVERSED = State::TRAVERSED;
constexpr auto UNKNOWN = State::UNKNOWN;

Boundary defaultBoundary() {
  return Boundary(
      {0, 0},
      {10, 10},
      {States(10, TRAV), States(10, TRAV), States(10, TRAV), States(10, TRAV)});
}

State fuse(State from, State to, bool optimistic = true) {
  spark_dsg::fuseStates(from, to, optimistic);
  return to;
}

TEST(TraversabilityBoundary, FuseStates) {
  // Optimistic Fusion.
  EXPECT_EQ(fuse(TRAVERSED, TRAVERSED), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, TRAVERSED), TRAVERSED);
  EXPECT_EQ(fuse(INTRAV, TRAVERSED), TRAVERSED);
  EXPECT_EQ(fuse(UNKNOWN, TRAVERSED), TRAVERSED);
  EXPECT_EQ(fuse(TRAVERSED, TRAV), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, TRAV), TRAV);
  EXPECT_EQ(fuse(INTRAV, TRAV), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, TRAV), TRAV);
  EXPECT_EQ(fuse(TRAVERSED, INTRAV), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, INTRAV), INTRAV);
  EXPECT_EQ(fuse(INTRAV, INTRAV), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, INTRAV), INTRAV);
  EXPECT_EQ(fuse(TRAVERSED, UNKNOWN), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, UNKNOWN), TRAV);
  EXPECT_EQ(fuse(INTRAV, UNKNOWN), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, UNKNOWN), UNKNOWN);

  // Pessimistic fusion.
  EXPECT_EQ(fuse(TRAVERSED, TRAVERSED, false), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, TRAVERSED, false), TRAVERSED);
  EXPECT_EQ(fuse(INTRAV, TRAVERSED, false), TRAVERSED);
  EXPECT_EQ(fuse(UNKNOWN, TRAVERSED, false), TRAVERSED);
  EXPECT_EQ(fuse(TRAVERSED, TRAV, false), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, TRAV, false), TRAV);
  EXPECT_EQ(fuse(INTRAV, TRAV, false), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, TRAV, false), UNKNOWN);
  EXPECT_EQ(fuse(TRAVERSED, INTRAV, false), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, INTRAV, false), INTRAV);
  EXPECT_EQ(fuse(INTRAV, INTRAV, false), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, INTRAV, false), INTRAV);
  EXPECT_EQ(fuse(TRAVERSED, UNKNOWN, false), TRAVERSED);
  EXPECT_EQ(fuse(TRAV, UNKNOWN, false), UNKNOWN);
  EXPECT_EQ(fuse(INTRAV, UNKNOWN, false), INTRAV);
  EXPECT_EQ(fuse(UNKNOWN, UNKNOWN, false), UNKNOWN);
}

TEST(TraversabilityBoundary, IntersectsSide) {
  Boundary boundary = defaultBoundary();

  // Check simple sides.
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(5, -1)), BOTTOM);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(-1, 5)), LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(5, 11)), TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(11, 5)), RIGHT);

  // Check more general rectangles.
  boundary = Boundary({2, -5}, {4, 5});
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(2, -5.1)), BOTTOM);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(1.9, -5)), LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(1.9, 5)), LEFT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(2, 5.1)), TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4, 5.1)), TOP);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4.1, 5)), RIGHT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4.1, -5)), RIGHT);
  EXPECT_EQ(boundary.lineIntersectsSide(Eigen::Vector2d(4, -5.1)), BOTTOM);
}

TEST(TraversabilityBoundary, IsOnSide) {
  Boundary boundary = defaultBoundary();

  // Intersects.
  EXPECT_EQ(boundary.isOnSide(Boundary({5, 5}, {15, 15})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {5, 5})), Side::INVALID);

  // Bottom side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {5, -1})), BOTTOM);
  EXPECT_EQ(boundary.isOnSide(Boundary({5, -5}, {15, -1})), BOTTOM);

  // Left side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {-1, 5})), LEFT);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 5}, {-1, 15})), LEFT);

  // Top side.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 11}, {5, 15})), TOP);
  EXPECT_EQ(boundary.isOnSide(Boundary({5, 11}, {15, 15})), TOP);

  // Right side.
  EXPECT_EQ(boundary.isOnSide(Boundary({11, -5}, {15, 5})), RIGHT);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, 5}, {15, 15})), RIGHT);

  // Corners.
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, -5}, {-1, -1})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({-5, 11}, {-1, 15})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, -5}, {15, -1})), Side::INVALID);
  EXPECT_EQ(boundary.isOnSide(Boundary({11, 11}, {15, 15})), Side::INVALID);
}

TEST(TraversabilityBoundary, Distance1D) {
  Boundary boundary = defaultBoundary();

  // Check distances to each side.
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(BOTTOM, -5), 5.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(BOTTOM, 4), -4.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(LEFT, -3), 3.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(LEFT, 2), -2.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(TOP, 12), 2.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(TOP, 7), -3.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(RIGHT, 14), 4.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(RIGHT, 5), -5.0);
}

TEST(TraversabilityBoundary, Voxels) {
  Boundary boundary = defaultBoundary();

  // Voxel size.
  EXPECT_DOUBLE_EQ(boundary.voxelSize(BOTTOM), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(LEFT), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(TOP), 1.0);
  EXPECT_DOUBLE_EQ(boundary.voxelSize(RIGHT), 1.0);

  // Set coordinates: Shrink
  boundary.setCoordinate(BOTTOM, 2.0);
  EXPECT_DOUBLE_EQ(boundary.min.y(), 2.0);
  EXPECT_EQ(boundary.states[LEFT].size(), 8);
  EXPECT_EQ(boundary.states[RIGHT].size(), 8);
  EXPECT_EQ(boundary.states[LEFT], States(8, TRAV));
  EXPECT_EQ(boundary.states[RIGHT], States(8, TRAV));
  EXPECT_EQ(boundary.voxelSize(BOTTOM), 1.0);
  EXPECT_EQ(boundary.voxelSize(LEFT), 1.0);
  EXPECT_EQ(boundary.voxelSize(TOP), 1.0);
  EXPECT_EQ(boundary.voxelSize(RIGHT), 1.0);

  boundary.setCoordinate(LEFT, 3.0);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 3.0);
  EXPECT_EQ(boundary.states[BOTTOM].size(), 7);
  EXPECT_EQ(boundary.states[TOP].size(), 7);
  EXPECT_EQ(boundary.states[BOTTOM], States(7, TRAV));
  EXPECT_EQ(boundary.states[TOP], States(7, TRAV));
  EXPECT_EQ(boundary.voxelSize(BOTTOM), 1.0);
  EXPECT_EQ(boundary.voxelSize(LEFT), 1.0);
  EXPECT_EQ(boundary.voxelSize(TOP), 1.0);
  EXPECT_EQ(boundary.voxelSize(RIGHT), 1.0);

  boundary.setCoordinate(TOP, 8.0);
  EXPECT_DOUBLE_EQ(boundary.max.y(), 8.0);
  EXPECT_EQ(boundary.states[LEFT].size(), 6);
  EXPECT_EQ(boundary.states[RIGHT].size(), 6);
  EXPECT_EQ(boundary.states[LEFT], States(6, TRAV));
  EXPECT_EQ(boundary.states[RIGHT], States(6, TRAV));

  boundary.setCoordinate(RIGHT, 7.0);
  EXPECT_DOUBLE_EQ(boundary.max.x(), 7.0);
  EXPECT_EQ(boundary.states[BOTTOM].size(), 4);
  EXPECT_EQ(boundary.states[TOP].size(), 4);
  EXPECT_EQ(boundary.states[BOTTOM], States(4, TRAV));
  EXPECT_EQ(boundary.states[TOP], States(4, TRAV));

  // Set coordinates: Extend
  boundary.setCoordinate(BOTTOM, -1.0);  // 2 --> -1
  EXPECT_DOUBLE_EQ(boundary.min.y(), -1.0);
  EXPECT_EQ(boundary.states[LEFT].size(), 9);
  EXPECT_EQ(boundary.states[RIGHT].size(), 9);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(boundary.states[LEFT][i], UNKNOWN);
    EXPECT_EQ(boundary.states[RIGHT][i], UNKNOWN);
  }
  for (size_t i = 3; i < 9; ++i) {
    EXPECT_EQ(boundary.states[LEFT][i], TRAV);
    EXPECT_EQ(boundary.states[RIGHT][i], TRAV);
  }

  boundary.setCoordinate(LEFT, 1.0);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 1.0);
  EXPECT_EQ(boundary.states[BOTTOM].size(), 6);
  EXPECT_EQ(boundary.states[TOP].size(), 6);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_EQ(boundary.states[BOTTOM][i], UNKNOWN);
    EXPECT_EQ(boundary.states[TOP][i], UNKNOWN);
  }
  for (size_t i = 3; i < 6; ++i) {
    EXPECT_EQ(boundary.states[BOTTOM][i], TRAV);
    EXPECT_EQ(boundary.states[TOP][i], TRAV);
  }

  boundary.setCoordinate(TOP, 9.0);
  EXPECT_DOUBLE_EQ(boundary.max.y(), 9.0);
  EXPECT_EQ(boundary.states[LEFT].size(), 10);
  EXPECT_EQ(boundary.states[RIGHT].size(), 10);
  EXPECT_EQ(boundary.states[LEFT][9], UNKNOWN);
  EXPECT_EQ(boundary.states[RIGHT][9], UNKNOWN);

  boundary.setCoordinate(RIGHT, 10.0);
  EXPECT_DOUBLE_EQ(boundary.max.x(), 10.0);
  EXPECT_EQ(boundary.states[BOTTOM].size(), 9);
  EXPECT_EQ(boundary.states[TOP].size(), 9);
  EXPECT_EQ(boundary.states[BOTTOM][8], UNKNOWN);
  EXPECT_EQ(boundary.states[TOP][8], UNKNOWN);

  // Round voxel sizes.
  boundary = defaultBoundary();
  boundary.setCoordinate(BOTTOM, 2.45);
  EXPECT_DOUBLE_EQ(boundary.min.y(), 2);
  EXPECT_EQ(boundary.states[LEFT].size(), 8);
  EXPECT_EQ(boundary.states[RIGHT].size(), 8);

  boundary.setCoordinate(LEFT, 3.55);
  EXPECT_DOUBLE_EQ(boundary.min.x(), 4);
  EXPECT_EQ(boundary.states[BOTTOM].size(), 6);
  EXPECT_EQ(boundary.states[TOP].size(), 6);
}

TEST(TraversabilityBoundary, MaxTraversableDistance) {
  Boundary from = defaultBoundary();
  Boundary to = from;

  // Perfect overlap.
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 10.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 10.0);

  // Partial overlap.
  to.min = Eigen::Vector2d(2, 2);
  to.max = Eigen::Vector2d(8, 8);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 6.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 6.0);

  // Overflow.
  to.max = Eigen::Vector2d(15, 15);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 8.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 8.0);

  // With intraversable states.
  to = from;
  for (size_t i = 0; i < 5; ++i) {
    from.states[BOTTOM][i] = INTRAV;
    from.states[LEFT][9 - i] = INTRAV;
  }
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 5.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 5.0);

  // With offset.
  to.min = Eigen::Vector2d(2, 2);
  to.max = Eigen::Vector2d(12, 12);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 5.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 3.0);

  to.min = Eigen::Vector2d(-2, -2);
  to.max = Eigen::Vector2d(8, 8);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, BOTTOM), 3.0);
  EXPECT_DOUBLE_EQ(from.maxTraversableDistance(to, LEFT), 5.0);
}

TEST(TraversabilityBoundary, FuseBoundaryStates) {
  auto b = Boundary({0, 0}, {10, 10});
  auto other = Boundary({5, 5}, {8, 8});

  // Simple cases.
  other.states[BOTTOM] = {TRAV};
  b.side(BOTTOM).fuseBoundaryStates(other.side(BOTTOM));
  EXPECT_EQ(b.states[BOTTOM].size(), 1);
  EXPECT_EQ(b.states[BOTTOM][0], TRAV);

  other.states[BOTTOM] = {INTRAV};
  b.side(BOTTOM).fuseBoundaryStates(other.side(BOTTOM));
  EXPECT_EQ(b.states[BOTTOM].size(), 1);
  EXPECT_EQ(b.states[BOTTOM][0], INTRAV);

  // With voxels.
  b.states[BOTTOM] = States(10, UNKNOWN);
  other.states[BOTTOM] = {TRAV, INTRAV, TRAVERSED};
  b.side(BOTTOM).fuseBoundaryStates(other.side(BOTTOM));
  EXPECT_EQ(b.states[BOTTOM].size(), 10);
  EXPECT_EQ(b.states[BOTTOM][4], UNKNOWN);
  EXPECT_EQ(b.states[BOTTOM][5], TRAV);
  EXPECT_EQ(b.states[BOTTOM][6], INTRAV);
  EXPECT_EQ(b.states[BOTTOM][7], TRAVERSED);

  // Partial overlap.
  b.states[BOTTOM] = States(10, UNKNOWN);
  other.min = Eigen::Vector2d(-1, -1);
  other.max = Eigen::Vector2d(2, 2);
  b.side(BOTTOM).fuseBoundaryStates(other.side(BOTTOM));
  EXPECT_EQ(b.states[BOTTOM].size(), 10);
  EXPECT_EQ(b.states[BOTTOM][0], INTRAV);
  EXPECT_EQ(b.states[BOTTOM][1], TRAVERSED);
  EXPECT_EQ(b.states[BOTTOM][2], UNKNOWN);

  other.min = Eigen::Vector2d(8, 8);
  other.max = Eigen::Vector2d(11, 11);
  b.side(BOTTOM).fuseBoundaryStates(other.side(BOTTOM));
  EXPECT_EQ(b.states[BOTTOM].size(), 10);
  EXPECT_EQ(b.states[BOTTOM][7], UNKNOWN);
  EXPECT_EQ(b.states[BOTTOM][8], TRAV);
  EXPECT_EQ(b.states[BOTTOM][9], INTRAV);
}

TEST(TraversabilityBoundary, MergeBoundaryStates) {
  auto b = Boundary({0.1, 0.1},
                    {10.1, 10.1},
                    {States(10, UNKNOWN),
                     States(10, UNKNOWN),
                     States(10, UNKNOWN),
                     States(10, UNKNOWN)});
  auto other = Boundary({5, 5},
                        {15, 15},
                        {States(10, UNKNOWN),
                         States(10, UNKNOWN),
                         States(10, UNKNOWN),
                         States(10, UNKNOWN)});

  // Contained boundary should be traversable.
  b.mergeTraversabilityStates(other, 2);
  EXPECT_EQ(b.states[TOP][0], UNKNOWN);
  EXPECT_EQ(b.states[TOP][1], UNKNOWN);
  EXPECT_EQ(b.states[TOP][2], UNKNOWN);
  EXPECT_EQ(b.states[TOP][3], UNKNOWN);
  EXPECT_EQ(b.states[TOP][4], TRAV);
  EXPECT_EQ(b.states[TOP][5], TRAV);
  EXPECT_EQ(b.states[TOP][6], TRAV);
  EXPECT_EQ(b.states[TOP][7], TRAV);
  EXPECT_EQ(b.states[TOP][8], TRAV);
  EXPECT_EQ(b.states[TOP][9], TRAV);

  EXPECT_EQ(b.states[RIGHT][0], UNKNOWN);
  EXPECT_EQ(b.states[RIGHT][1], UNKNOWN);
  EXPECT_EQ(b.states[RIGHT][2], UNKNOWN);
  EXPECT_EQ(b.states[RIGHT][3], UNKNOWN);
  EXPECT_EQ(b.states[RIGHT][4], TRAV);
  EXPECT_EQ(b.states[RIGHT][5], TRAV);
  EXPECT_EQ(b.states[RIGHT][6], TRAV);
  EXPECT_EQ(b.states[RIGHT][7], TRAV);
  EXPECT_EQ(b.states[RIGHT][8], TRAV);
  EXPECT_EQ(b.states[RIGHT][9], TRAV);
  EXPECT_EQ(b.states[BOTTOM], States(10, UNKNOWN));
  EXPECT_EQ(b.states[LEFT], States(10, UNKNOWN));

  // With intraversable.
  other.min = Eigen::Vector2d(-1, -1);
  other.max = Eigen::Vector2d(9, 9);
  other.states[LEFT] = States(10, INTRAV);
  other.states[TOP][6] = INTRAV;
  other.states[TOP][7] = INTRAV;

  b.mergeTraversabilityStates(other, 2);
  EXPECT_EQ(b.states[BOTTOM][0], TRAV);
  EXPECT_EQ(b.states[BOTTOM][1], TRAV);
  EXPECT_EQ(b.states[BOTTOM][8], TRAV);
  EXPECT_EQ(b.states[BOTTOM][9], UNKNOWN);

  EXPECT_EQ(b.states[LEFT][0], INTRAV);
  EXPECT_EQ(b.states[LEFT][1], INTRAV);
  EXPECT_EQ(b.states[LEFT][8], INTRAV);
  EXPECT_EQ(b.states[LEFT][9], UNKNOWN);

  EXPECT_EQ(b.states[TOP][0], UNKNOWN);
  EXPECT_EQ(b.states[TOP][1], UNKNOWN);
  EXPECT_EQ(b.states[TOP][2], UNKNOWN);
  EXPECT_EQ(b.states[TOP][3], UNKNOWN);
  EXPECT_EQ(b.states[TOP][4], TRAV);
  EXPECT_EQ(b.states[TOP][5], INTRAV);
  EXPECT_EQ(b.states[TOP][6], INTRAV);
  EXPECT_EQ(b.states[TOP][7], TRAV);
  EXPECT_EQ(b.states[TOP][8], TRAV);
  EXPECT_EQ(b.states[TOP][9], TRAV);
}

TEST(TraversabilityBoundary, filterTraversabilityStates) {
  // Unchanged.
  auto states = States(10, TRAV);
  States filtered = states;
  EXPECT_FALSE(spark_dsg::filterTraversabilityStates(filtered, 2));
  EXPECT_EQ(states, filtered);

  states = States(10, INTRAV);
  filtered = states;
  EXPECT_FALSE(spark_dsg::filterTraversabilityStates(filtered, 2));
  EXPECT_EQ(states, filtered);

  // Short sequences.
  states = {TRAV, TRAV, TRAV};
  filtered = states;
  EXPECT_FALSE(spark_dsg::filterTraversabilityStates(filtered, 3));
  EXPECT_EQ(states, filtered);

  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 4));
  EXPECT_EQ(filtered, (States{INTRAV, INTRAV, INTRAV}));

  // Few points out of reach.
  states = {TRAV, TRAV, INTRAV, TRAV, TRAV, INTRAV, TRAV, TRAV, TRAV, TRAV};
  filtered = states;
  EXPECT_FALSE(spark_dsg::filterTraversabilityStates(filtered, 2));
  EXPECT_EQ(states, filtered);

  // Few points within reach.
  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 3));
  EXPECT_EQ(
      filtered,
      (States{INTRAV, INTRAV, INTRAV, INTRAV, INTRAV, INTRAV, TRAV, TRAV, TRAV, TRAV}));

  // Sequence ends.
  states = {TRAV, TRAV, INTRAV, TRAV, TRAV, TRAV, INTRAV, TRAV, TRAV};
  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 3));
  EXPECT_EQ(filtered,
            (States{INTRAV, INTRAV, INTRAV, TRAV, TRAV, TRAV, INTRAV, INTRAV, INTRAV}));

  // With unknowns, optimistic.
  states = {TRAV, TRAV, UNKNOWN, TRAV, TRAV, TRAV, UNKNOWN, TRAV, TRAV};
  filtered = states;
  EXPECT_FALSE(spark_dsg::filterTraversabilityStates(filtered, 3, true));
  EXPECT_EQ(states, filtered);

  // Unknowns, pessimistic.
  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 3, false));
  EXPECT_EQ(
      filtered,
      (States{UNKNOWN, UNKNOWN, UNKNOWN, TRAV, TRAV, TRAV, UNKNOWN, UNKNOWN, UNKNOWN}));

  // Mixed states, pessimistic.
  states = {TRAV, UNKNOWN, INTRAV, TRAV, UNKNOWN, TRAV, UNKNOWN, INTRAV, UNKNOWN, TRAV};
  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 2, false));
  EXPECT_EQ(filtered,
            (States{UNKNOWN,
                    UNKNOWN,
                    INTRAV,
                    UNKNOWN,
                    UNKNOWN,
                    UNKNOWN,
                    UNKNOWN,
                    INTRAV,
                    UNKNOWN,
                    UNKNOWN}));

  filtered = states;
  EXPECT_TRUE(spark_dsg::filterTraversabilityStates(filtered, 3, false));
  EXPECT_EQ(filtered,
            (States{INTRAV,
                    INTRAV,
                    INTRAV,
                    UNKNOWN,
                    UNKNOWN,
                    UNKNOWN,
                    UNKNOWN,
                    INTRAV,
                    INTRAV,
                    INTRAV}));
}
