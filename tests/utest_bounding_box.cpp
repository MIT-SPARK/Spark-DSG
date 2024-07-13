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
#include <spark_dsg/bounding_box.h>

#include <Eigen/Geometry>

namespace spark_dsg {

namespace {
inline float getRotationError(const Eigen::Quaternionf& rotation,
                              const BoundingBox& box) {
  return rotation.angularDistance(Eigen::Quaternionf(box.world_R_center));
}
}  // namespace

TEST(BoundingBoxTests, AABBConstructor) {
  Eigen::Vector3f pos{1.0f, 2.0f, 3.0f};
  Eigen::Vector3f dim{3.0f, 4.0f, 5.0f};
  BoundingBox box(dim, pos);
  EXPECT_EQ(BoundingBox::Type::AABB, box.type);
  EXPECT_EQ(pos, box.world_P_center);
  EXPECT_EQ(dim, box.dimensions);
}

TEST(BoundingBoxTests, OBBConstructor) {
  Eigen::Vector3f pos{1.0f, 2.0f, 3.0f};
  Eigen::Vector3f dim{3.0f, 4.0f, 5.0f};
  Eigen::Quaternionf rotation(0.0, 1.0, 0.0, 0.0);  // 180 deg around Y.

  BoundingBox box(dim, pos, rotation);
  EXPECT_EQ(BoundingBox::Type::OBB, box.type);
  EXPECT_EQ(0.0, (pos - box.world_P_center).norm());
  EXPECT_EQ(0.0, (dim - box.dimensions).norm());
  EXPECT_EQ(0.0, getRotationError(rotation, box));
}

TEST(BoundingBoxTests, InvalidVolumeChecksCorrect) {
  BoundingBox box;
  EXPECT_FALSE(box.volume() > 0);
  EXPECT_FALSE(box.contains(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
}

TEST(BoundingBoxTests, AABBVolumeChecksCorrect) {
  const Eigen::Vector3f size{1.0f, 2.0f, 3.0f};
  BoundingBox box(size);
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, BasicOBBVolumeChecksCorrect) {
  const Eigen::Vector3f size{1.0f, 2.0f, 3.0f};
  BoundingBox box(size, size / 2, Eigen::Quaternionf::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, RotatedOBBVolumeChecksCorrect) {
  // positive pi / 6 rotation around z
  BoundingBox box(
      Eigen::Vector3f(2.0, 3.0, 4.0),
      Eigen::Vector3f(5.0, 5.5, 6.0),
      Eigen::Quaternionf(std::cos(M_PI / 12.0), 0.0, 0.0, std::sin(M_PI / 12.0)));
  EXPECT_NEAR(24.0f, box.volume(), 1.0e-8f);

  {  // previously inside, but now outside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // actually inside (at center)
    Eigen::Vector3f test_point1(5.0, 5.0, 5.0);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // actually inside (with rotation)
    Eigen::Vector3f test_point1(4.6, 6.9, 6.0);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksCorrectBasic) {
  const Eigen::Vector3f size(1.0, 2.0, 3.0);
  BoundingBox box(
      BoundingBox::Type::RAABB, size, size / 2, Eigen::Matrix3f::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksNonZeroOrigin) {
  BoundingBox box(BoundingBox::Type::RAABB,
                  Eigen::Vector3f(1, 2, 3),
                  Eigen::Vector3f(0, -1, -1.5),
                  Eigen::Matrix3f::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(-0.5, -1.5, -2.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-1.5, -2.5, -3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(2.5, 1.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-1.5, 1.5, -2.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksCorrectWithRotation) {
  // positive pi / 12 rotation around z
  BoundingBox box(
      Eigen::Vector3f(1.0, 2.0, 3.0), Eigen::Vector3f(0.5, 1.0, 1.5), M_PI / 12.0);
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside (at center)
    Eigen::Vector3f test_point1(0.5, 1.0, 1.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }

  {  // outside (with rotation) that would have been in non-rotated
    Eigen::Vector3f test_point1(0.9, 1.9, 0.1);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.contains(test_point1));
    EXPECT_FALSE(box.contains(test_point2));
  }

  {  // actually inside (with rotation)
    Eigen::Vector3f test_point1(-0.1, 1.85, 0.1);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.contains(test_point1));
    EXPECT_TRUE(box.contains(test_point2));
  }
}

TEST(BoundingBoxTests, AABBIntersects) {
  Eigen::Vector3f size(1.0, 2.0, 3.0);
  Eigen::Vector3f pos(0.0, 0.0, 0.0);
  BoundingBox box(size, pos);
  BoundingBox box2 = box;

  // Self.
  EXPECT_TRUE(box.intersects(box2));

  // Translation inside.
  box2.world_P_center = Eigen::Vector3f(0.9, 1.9, 2.9);
  EXPECT_TRUE(box.intersects(box2));

  // Translation outside.
  box2.world_P_center = Eigen::Vector3f(0.9, 1.9, 3.1);
  EXPECT_FALSE(box.intersects(box2));

  // Touches exactly.
  box2.world_P_center = Eigen::Vector3f(0.9, 1.9, 3.0);
  EXPECT_FALSE(box.intersects(box2));
}

TEST(BoundingBoxTests, AABBcomputeIoU) {
  Eigen::Vector3f size(1.0, 2.0, 3.0);
  Eigen::Vector3f pos(0.0, 0.0, 0.0);
  BoundingBox box(size, pos);
  BoundingBox box2 = box;

  // Self.
  EXPECT_EQ(box.computeIoU(box2), 1.0f);

  // Half.
  box2.world_P_center = Eigen::Vector3f(0.5, 0, 0);
  EXPECT_NEAR(box.computeIoU(box2), 1.0f / 3, 10e-6);

  // Corner.
  box2.world_P_center = Eigen::Vector3f(0.5, 1, 1.5);
  EXPECT_NEAR(box.computeIoU(box2), 0.06666666667, 10e-6);

  // No overlap.
  box2.world_P_center = Eigen::Vector3f(1, 0, 0);
  EXPECT_EQ(box.computeIoU(box2), 0.0f);
}

TEST(BoundingBoxTests, Corners) {
  Eigen::Vector3f size(1.0, 2.0, 3.0);
  Eigen::Vector3f pos(0.5, 1.0, 1.5);
  const float yaw = M_PI / 6;
  const BoundingBox box(size, pos, yaw);
  const auto corners = box.corners();

  // Min corner.
  EXPECT_NEAR(0.566987, corners[0](0), 1.0e-6f);
  EXPECT_NEAR(-0.116025, corners[0](1), 1.0e-6f);
  EXPECT_NEAR(0, corners[0](2), 1.0e-6f);

  // Max corner.
  EXPECT_NEAR(0.433012, corners[6](0), 1.0e-6f);
  EXPECT_NEAR(2.116025, corners[6](1), 1.0e-6f);
  EXPECT_NEAR(3, corners[6](2), 1.0e-6f);
}

TEST(BoundingBoxTests, frameConversions) {
  const Eigen::Vector3f size(1.0, 2.0, 3.0);
  const Eigen::Vector3f pos(0.5, 1.0, 1.5);
  const float yaw = M_PI / 6;
  const BoundingBox box(size, pos, yaw);

  // Center.
  const Eigen::Vector3f p1 = box.pointToWorldFrame(Eigen::Vector3f::Zero());
  EXPECT_NEAR(0.5, p1(0), 1.0e-6f);
  EXPECT_NEAR(1.0, p1(1), 1.0e-6f);
  EXPECT_NEAR(1.5, p1(2), 1.0e-6f);

  const Eigen::Vector3f p2 = box.pointToBoxFrame(pos);
  EXPECT_NEAR(0.0, p2(0), 1.0e-6f);
  EXPECT_NEAR(0.0, p2(1), 1.0e-6f);
  EXPECT_NEAR(0.0, p2(2), 1.0e-6f);

  // Random point.
  const Eigen::Vector3f p3 = box.pointToWorldFrame(Eigen::Vector3f(0.3, 0.4, 0.5));
  EXPECT_NEAR(0.55980762, p3(0), 1.0e-6f);
  EXPECT_NEAR(1.49641016, p3(1), 1.0e-6f);
  EXPECT_NEAR(2, p3(2), 1.0e-6f);

  const Eigen::Vector3f p4 = box.pointToBoxFrame(Eigen::Vector3f(0.3, 0.4, 0.5));
  EXPECT_NEAR(-0.4732050, p4(0), 1.0e-6f);
  EXPECT_NEAR(-0.4196152, p4(1), 1.0e-6f);
  EXPECT_NEAR(-1, p4(2), 1.0e-6f);
}

TEST(BoundingBoxTests, merge) {
  // Aligned AABB.
  BoundingBox box1(Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(0.5, 1.0, 1.5));
  BoundingBox box2(Eigen::Vector3f(3, 2, 1), Eigen::Vector3f(0.5, 1.0, 1.5));
  box1.merge(box2);
  EXPECT_EQ(Eigen::Vector3f(3, 2, 3), box1.dimensions);
  EXPECT_EQ(Eigen::Vector3f(0.5, 1.0, 1.5), box1.world_P_center);

  // Translated AABB.
  box1 = BoundingBox(Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(0.5, 1.0, 1.5));
  box2 = BoundingBox(Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(-0.5, -1.0, -1.5));
  box1.merge(box2);
  EXPECT_EQ(Eigen::Vector3f(2, 4, 6), box1.dimensions);
  EXPECT_EQ(Eigen::Vector3f(0, 0, 0), box1.world_P_center);

  // General Box (RAABB).
  box1 =
      BoundingBox(Eigen::Vector3f(1, 2, 3), Eigen::Vector3f(0.5, 1.0, 1.5), M_PI / 6);
  box2 = BoundingBox(
      Eigen::Vector3f(1.5, 1.5, 1.5), Eigen::Vector3f(0.5, 1.0, 1.5), M_PI / 6);
  box1.merge(box2);
  const float yaw = box1.world_R_center.eulerAngles(0, 1, 2)[2];
  // The solution finds the 90deg rotated box, i.e. y=x and yaw=-60deg from 30 deg.
  EXPECT_NEAR(2, box1.dimensions(0), 1.0e-6f);
  EXPECT_NEAR(1.5, box1.dimensions(1), 1.0e-6f);
  EXPECT_NEAR(3, box1.dimensions(2), 1.0e-6f);
  EXPECT_NEAR(-M_PI / 3, yaw, 1.0e-6f);
}

}  // namespace spark_dsg
