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

State fuse(State from, State to, bool optimistic = true) {
  spark_dsg::fuseStates(from, to, optimistic);
  return to;
}

TEST(TraversabilityBoundary, FuseStates) {
  EXPECT_EQ(fuse(State::TRAVERSED, State::TRAVERSED), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::TRAVERSED), State::TRAVERSED);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::TRAVERSED), State::TRAVERSED);
  EXPECT_EQ(fuse(State::UNKNOWN, State::TRAVERSED), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSED, State::TRAVERSABLE), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::TRAVERSABLE), State::TRAVERSABLE);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::TRAVERSABLE), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::TRAVERSABLE), State::TRAVERSABLE);
  EXPECT_EQ(fuse(State::TRAVERSED, State::INTRAVERSABLE), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::INTRAVERSABLE), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::INTRAVERSABLE), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::INTRAVERSABLE), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::TRAVERSED, State::UNKNOWN), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::UNKNOWN), State::TRAVERSABLE);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::UNKNOWN), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::UNKNOWN), State::UNKNOWN);

  // Pessimistic fusion.
  EXPECT_EQ(fuse(State::TRAVERSED, State::TRAVERSED, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::TRAVERSED, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::TRAVERSED, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::UNKNOWN, State::TRAVERSED, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSED, State::TRAVERSABLE, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::TRAVERSABLE, false), State::TRAVERSABLE);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::TRAVERSABLE, false),
            State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::TRAVERSABLE, false), State::UNKNOWN);
  EXPECT_EQ(fuse(State::TRAVERSED, State::INTRAVERSABLE, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::INTRAVERSABLE, false),
            State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::INTRAVERSABLE, false),
            State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::INTRAVERSABLE, false), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::TRAVERSED, State::UNKNOWN, false), State::TRAVERSED);
  EXPECT_EQ(fuse(State::TRAVERSABLE, State::UNKNOWN, false), State::UNKNOWN);
  EXPECT_EQ(fuse(State::INTRAVERSABLE, State::UNKNOWN, false), State::INTRAVERSABLE);
  EXPECT_EQ(fuse(State::UNKNOWN, State::UNKNOWN, false), State::UNKNOWN);
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

TEST(TraversabilityBoundary, Distance1D) {
  Boundary boundary = defaultBoundary();

  // Check distances to each side.
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::BOTTOM, -5), 5.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::BOTTOM, 4), -4.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::LEFT, -3), 3.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::LEFT, 2), -2.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::TOP, 12), 2.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::TOP, 7), -3.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::RIGHT, 14), 4.0);
  EXPECT_DOUBLE_EQ(boundary.distanceToSide1D(Side::RIGHT, 5), -5.0);
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

TEST(TraversabilityBoundary, FuseBoundaryStates) {
  auto b = Boundary({0, 0}, {10, 10});
  auto other = Boundary({5, 5}, {8, 8});

  // Simple cases.
  other.states[Side::BOTTOM] = {State::TRAVERSABLE};
  b.side(Side::BOTTOM).fuseBoundaryStates(other.side(Side::BOTTOM));
  EXPECT_EQ(b.states[Side::BOTTOM].size(), 1);
  EXPECT_EQ(b.states[Side::BOTTOM][0], State::TRAVERSABLE);

  other.states[Side::BOTTOM] = {State::INTRAVERSABLE};
  b.side(Side::BOTTOM).fuseBoundaryStates(other.side(Side::BOTTOM));
  EXPECT_EQ(b.states[Side::BOTTOM].size(), 1);
  EXPECT_EQ(b.states[Side::BOTTOM][0], State::INTRAVERSABLE);

  // With voxels.
  b.states[Side::BOTTOM] = States(10, State::UNKNOWN);
  other.states[Side::BOTTOM] = {
      State::TRAVERSABLE, State::INTRAVERSABLE, State::TRAVERSED};
  b.side(Side::BOTTOM).fuseBoundaryStates(other.side(Side::BOTTOM));
  EXPECT_EQ(b.states[Side::BOTTOM].size(), 10);
  EXPECT_EQ(b.states[Side::BOTTOM][4], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::BOTTOM][5], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][6], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][7], State::TRAVERSED);

  // Partial overlap.
  b.states[Side::BOTTOM] = States(10, State::UNKNOWN);
  other.min = Eigen::Vector2d(-1, -1);
  other.max = Eigen::Vector2d(2, 2);
  b.side(Side::BOTTOM).fuseBoundaryStates(other.side(Side::BOTTOM));
  EXPECT_EQ(b.states[Side::BOTTOM].size(), 10);
  EXPECT_EQ(b.states[Side::BOTTOM][0], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][1], State::TRAVERSED);
  EXPECT_EQ(b.states[Side::BOTTOM][2], State::UNKNOWN);

  other.min = Eigen::Vector2d(8, 8);
  other.max = Eigen::Vector2d(11, 11);
  b.side(Side::BOTTOM).fuseBoundaryStates(other.side(Side::BOTTOM));
  EXPECT_EQ(b.states[Side::BOTTOM].size(), 10);
  EXPECT_EQ(b.states[Side::BOTTOM][7], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::BOTTOM][8], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][9], State::INTRAVERSABLE);
}

TEST(TraversabilityBoundary, MergeBoundaryStates) {
  auto b = Boundary({0.1, 0.1},
                    {10.1, 10.1},
                    {States(10, State::UNKNOWN),
                     States(10, State::UNKNOWN),
                     States(10, State::UNKNOWN),
                     States(10, State::UNKNOWN)});
  auto other = Boundary({5, 5},
                        {15, 15},
                        {States(10, State::UNKNOWN),
                         States(10, State::UNKNOWN),
                         States(10, State::UNKNOWN),
                         States(10, State::UNKNOWN)});

  // Contained boundary should be traversable.
  b.mergeTraversabilityStates(other, 2);
  EXPECT_EQ(b.states[Side::TOP][0], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][1], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][2], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][3], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][4], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][5], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][6], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][7], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][8], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][9], State::TRAVERSABLE);

  EXPECT_EQ(b.states[Side::RIGHT][0], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::RIGHT][1], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::RIGHT][2], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::RIGHT][3], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::RIGHT][4], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::RIGHT][5], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::RIGHT][6], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::RIGHT][7], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::RIGHT][8], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::RIGHT][9], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM], States(10, State::UNKNOWN));
  EXPECT_EQ(b.states[Side::LEFT], States(10, State::UNKNOWN));

  // With intraversable.
  other.min = Eigen::Vector2d(-1, -1);
  other.max = Eigen::Vector2d(9, 9);
  other.states[Side::LEFT] = States(10, State::INTRAVERSABLE);
  other.states[Side::TOP][6] = State::INTRAVERSABLE;
  other.states[Side::TOP][7] = State::INTRAVERSABLE;

  b.mergeTraversabilityStates(other, 2);
  EXPECT_EQ(b.states[Side::BOTTOM][0], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][1], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][8], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::BOTTOM][9], State::UNKNOWN);

  EXPECT_EQ(b.states[Side::LEFT][0], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::LEFT][1], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::LEFT][8], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::LEFT][9], State::UNKNOWN);

  EXPECT_EQ(b.states[Side::TOP][0], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][1], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][2], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][3], State::UNKNOWN);
  EXPECT_EQ(b.states[Side::TOP][4], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][5], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][6], State::INTRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][7], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][8], State::TRAVERSABLE);
  EXPECT_EQ(b.states[Side::TOP][9], State::TRAVERSABLE);
}
