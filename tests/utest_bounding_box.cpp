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
  Eigen::Vector3f min{1.0f, 2.0f, 3.0f};
  Eigen::Vector3f max{3.0f, 4.0f, 5.0f};
  BoundingBox box(min, max);
  EXPECT_EQ(BoundingBox::Type::AABB, box.type);
  EXPECT_EQ(min, box.min);
  EXPECT_EQ(max, box.max);
}

TEST(BoundingBoxTests, OBBConstructor) {
  Eigen::Vector3f min;
  min << 1.0f, 2.0f, 3.0f;
  Eigen::Vector3f max;
  max << 3.0f, 4.0f, 5.0f;
  Eigen::Vector3f position;
  position << 2.0f, 3.0f, 4.0f;
  Eigen::Quaternionf rotation(0.0, 1.0, 0.0, 0.0);

  BoundingBox box(min, max, position, rotation);
  EXPECT_EQ(BoundingBox::Type::OBB, box.type);
  EXPECT_EQ(0.0, (min - box.min).norm());
  EXPECT_EQ(0.0, (max - box.max).norm());
  EXPECT_EQ(0.0, (position - box.world_P_center).norm());
  EXPECT_EQ(0.0, getRotationError(rotation, box));
}

TEST(BoundingBoxTests, InvalidVolumeChecksCorrect) {
  Eigen::Vector3f test_point1(1.0, 2.0, 3.0);
  Eigen::Vector3d test_point2(1.0, 2.0, 3.0);

  BoundingBox box;
  EXPECT_FALSE(std::isfinite(box.volume()));
  EXPECT_FALSE(box.isInside(Eigen::Vector3f(1.0f, 2.0f, 3.0f)));
  EXPECT_FALSE(box.isInside(Eigen::Vector3f(1.0, 2.0, 3.0)));
}

TEST(BoundingBoxTests, AABBVolumeChecksCorrect) {
  BoundingBox box(Eigen::Vector3f::Zero(), Eigen::Vector3f(1.0, 2.0, 3.0));
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, BasicOBBVolumeChecksCorrect) {
  BoundingBox box(Eigen::Vector3f::Zero(),
                  Eigen::Vector3f(1.0, 2.0, 3.0),
                  Eigen::Vector3f::Zero(),
                  Eigen::Quaternionf::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, RotatedOBBVolumeChecksCorrect) {
  // positive pi / 6 rotation around z
  BoundingBox box(
      Eigen::Vector3f(-1.0, -1.0, -1.0),
      Eigen::Vector3f(1.0, 2.0, 3.0),
      Eigen::Vector3f(5.0, 5.0, 5.0),
      Eigen::Quaternionf(std::cos(M_PI / 12.0), 0.0, 0.0, std::sin(M_PI / 12.0)));
  EXPECT_NEAR(24.0f, box.volume(), 1.0e-8f);

  {  // previously inside, but now outside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // actually inside (at center)
    Eigen::Vector3f test_point1(5.0, 5.0, 5.0);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // actually inside (with rotation)
    Eigen::Vector3f test_point1(4.6, 6.9, 6.0);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksCorrectBasic) {
  BoundingBox box(BoundingBox::Type::RAABB,
                  Eigen::Vector3f::Zero(),
                  Eigen::Vector3f(1.0, 2.0, 3.0),
                  Eigen::Vector3f::Zero(),
                  Eigen::Matrix3f::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(0.5, 0.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-0.5, -0.5, -0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(3.5, 3.5, 3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-0.5, 3.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksNonZeroOrigin) {
  BoundingBox box(BoundingBox::Type::RAABB,
                  Eigen::Vector3f(-0.5, 0.0, 0.0),
                  Eigen::Vector3f(0.5, 2.0, 3.0),
                  Eigen::Vector3f(-0.5, -2.0, -3.0),
                  Eigen::Matrix3f::Identity());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside
    Eigen::Vector3f test_point1(-0.5, -1.5, -2.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // outside low
    Eigen::Vector3f test_point1(-1.5, -2.5, -3.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside high
    Eigen::Vector3f test_point1(2.5, 1.5, 0.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // outside mixed
    Eigen::Vector3f test_point1(-1.5, 1.5, -2.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, RAABBVolumeChecksCorrectWithRotation) {
  // positive pi / 12 rotation around z
  BoundingBox box(
      BoundingBox::Type::RAABB,
      Eigen::Vector3f(0.0, 0.0, 0.0),
      Eigen::Vector3f(1.0, 2.0, 3.0),
      Eigen::Vector3f::Zero(),
      Eigen::Quaternionf(std::cos(M_PI / 12.0), 0.0, 0.0, std::sin(M_PI / 12.0))
          .toRotationMatrix());
  EXPECT_NEAR(6.0f, box.volume(), 1.0e-8f);

  {  // inside (at center)
    Eigen::Vector3f test_point1(0.5, 1.0, 1.5);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }

  {  // outside (with rotation) that would have been in non-rotated
    Eigen::Vector3f test_point1(0.9, 1.9, 0.1);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_FALSE(box.isInside(test_point1));
    EXPECT_FALSE(box.isInside(test_point2));
  }

  {  // actually inside (with rotation)
    Eigen::Vector3f test_point1(-0.1, 1.95, 0.1);
    Eigen::Vector3d test_point2 = test_point1.cast<double>();
    EXPECT_TRUE(box.isInside(test_point1));
    EXPECT_TRUE(box.isInside(test_point2));
  }
}

TEST(BoundingBoxTests, DimensionsCorrect) {
  {  // test invalid
    BoundingBox box;
    EXPECT_NEAR(box.dimensions().norm(), 0.0f, 1.0e-9f);
  }  // test invalid

  {  // test AABB
    Eigen::Vector3f min{1.0f, 2.0f, 3.0f};
    Eigen::Vector3f max{3.0f, 4.0f, 5.0f};
    BoundingBox box(min, max);
    const Eigen::Vector3f expected{2, 2, 2};
    EXPECT_NEAR((expected - box.dimensions()).norm(), 0.0f, 1.0e-8f);
  }

  {  // test RAABB
    BoundingBox box(
        BoundingBox::Type::RAABB,
        Eigen::Vector3f(-1.0, -2.0, -3.0),
        Eigen::Vector3f(0.0, 1.0, 2.0),
        Eigen::Vector3f::Zero(),
        Eigen::Quaternionf(std::cos(M_PI / 12.0), 0.0, 0.0, std::sin(M_PI / 12.0))
            .toRotationMatrix());

    const Eigen::Vector3f expected{1, 3, 5};
    EXPECT_NEAR((expected - box.dimensions()).norm(), 0.0f, 1.0e-8f)
        << "got " << box.dimensions() << " (expected " << expected << ")";
  }  // test RAABB
}

}  // namespace spark_dsg
