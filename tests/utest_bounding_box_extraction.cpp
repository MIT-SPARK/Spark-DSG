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
#include <spark_dsg/bounding_box_extraction.h>

#include <Eigen/Geometry>

namespace spark_dsg {

namespace {
inline float getRotationError(const Eigen::Quaternionf& rotation,
                              const BoundingBox& box) {
  // we only care up to 180 degrees orientation
  return std::fmod(rotation.angularDistance(Eigen::Quaternionf(box.world_R_center)),
                   M_PI);
}

struct TestAdaptor : public bounding_box::PointAdaptor {
  size_t size() const override { return points.size(); }

  Eigen::Vector3f get(size_t index) const override {
    return Eigen::Map<const Eigen::Vector3f>(points.at(index).data());
  }

  std::vector<std::array<float, 3>> points;
};

}  // namespace

TEST(BoundingBoxExtractionTests, InvalidFromPoints) {
  BoundingBox box;
  EXPECT_EQ(box.type, BoundingBox::Type::INVALID);

  TestAdaptor adaptor;
  box = bounding_box::extract(adaptor, BoundingBox::Type::INVALID);
  EXPECT_EQ(box.type, BoundingBox::Type::INVALID);
}

TEST(BoundingBoxExtractionTests, AABBFromPoints) {
  TestAdaptor adaptor;
  adaptor.points = {
      // lower and upper x
      {1.0f, 0.0f, 0.0f},
      {-1.0f, 0.0f, 0.0f},
      // lower and upper y
      {0.0f, 1.5f, 0.0f},
      {0.0f, -0.5f, 0.0f},
      // lower and upper z
      {0.0f, 0.0f, 1.0f},
      {0.0f, 0.0f, 4.0f},
  };

  // get bounding box from pointcloud
  BoundingBox box(adaptor);
  EXPECT_EQ(BoundingBox::Type::AABB, box.type);
  EXPECT_EQ(2.0f, box.dimensions(0));
  EXPECT_EQ(2.0f, box.dimensions(1));
  EXPECT_EQ(4.0f, box.dimensions(2));
  EXPECT_EQ(0.0f, box.world_P_center(0));
  EXPECT_EQ(0.5f, box.world_P_center(1));
  EXPECT_EQ(2.0f, box.world_P_center(2));
}

TEST(BoundingBoxExtractionTests, ConvexHull) {
  TestAdaptor adaptor;
  adaptor.points = {
      {1.0f, 0.0f, 4.0f},
      {0.5f, 0.0f, 3.0f},
      {0.5f, 0.5f, 0.0f},
      {0.2f, 0.2f, 0.2f},
      {0.0f, 1.0f, 1.1f},
      {0.4f, 0.6f, -1.0f},
      {1.0f, 1.0f, 4.0f},
      {0.9f, 0.1f, 3.0f},
      {0.0f, 0.0f, 2.0f},
      {0.1f, 0.9f, -0.5f},
  };

  const auto hull = bounding_box::get2dConvexHull(adaptor);
  std::list<size_t> expected{8, 0, 6, 4};
  EXPECT_EQ(hull, expected);
}

TEST(BoundingBoxExtractionTests, RAABBFromTwoPoints) {
  TestAdaptor adaptor;
  adaptor.points = {
      {0.0f, 0.0f, 0.0f},
      {5.0f, 0.0f, 0.0f},
  };

  BoundingBox box = bounding_box::extract(adaptor, BoundingBox::Type::RAABB);
  EXPECT_EQ(BoundingBox::Type::RAABB, box.type);
  EXPECT_NEAR(5.0f, box.dimensions(0), 1.0e-6);
  EXPECT_NEAR(0.0f, box.dimensions(1), 1.0e-6);
  EXPECT_NEAR(0.0f, box.dimensions(2), 1.0e-6);
  EXPECT_NEAR(2.5f, box.world_P_center(0), 1.0e-6);
  EXPECT_NEAR(0.0f, box.world_P_center(1), 1.0e-6);
  EXPECT_NEAR(0.0f, box.world_P_center(2), 1.0e-6);

  Eigen::Quaternionf expected_rotation = Eigen::Quaternionf::Identity();
  EXPECT_NEAR(0.0f, getRotationError(expected_rotation, box), 1.0e-6f);
}

TEST(BoundingBoxExtractionTests, RAABBFromPoints) {
  TestAdaptor adaptor;
  adaptor.points = {
      {0.0f, 0.0f, 0.0f},
      {5.0f, 0.0f, 0.0f},
      {2.5f, 2.5f, 1.0f},
  };

  // get bounding box from pointcloud
  BoundingBox box = bounding_box::extract(adaptor, BoundingBox::Type::RAABB);
  EXPECT_EQ(BoundingBox::Type::RAABB, box.type);
  EXPECT_NEAR(5.0f, box.dimensions(0), 1.0e-6);
  EXPECT_NEAR(2.5f, box.dimensions(1), 1.0e-6);
  EXPECT_NEAR(1.0f, box.dimensions(2), 1.0e-6);
  EXPECT_NEAR(2.5f, box.world_P_center(0), 1.0e-6);
  EXPECT_NEAR(1.25f, box.world_P_center(1), 1.0e-6);
  EXPECT_NEAR(0.5f, box.world_P_center(2), 1.0e-6);

  Eigen::Quaternionf expected_rotation = Eigen::Quaternionf::Identity();
  EXPECT_NEAR(0.0f, getRotationError(expected_rotation, box), 1.0e-6f);
}

TEST(BoundingBoxExtractionTests, RAABBFromPointsNonTrivial) {
  TestAdaptor adaptor;
  const size_t num_steps = 2;
  const float angle = M_PI / 6.0f;
  const float length = 5.0;
  const float width = 2.0;
  const float height = 0.4;

  Eigen::Vector3f world_p_box(1.0f, 2.0f, 3.0f);

  Eigen::Matrix3f world_R_box;
  world_R_box << std::cos(angle), -std::sin(angle), 0.0f, std::sin(angle),
      std::cos(angle), 0.0f, 0.0f, 0.0f, 1.0f;
  const Eigen::Vector3f box_centroid(length / 2.0f, width / 2.0f, height / 2.0f);

  for (size_t i = 0; i <= num_steps; ++i) {
    float x = length * (static_cast<float>(i) / num_steps);

    for (size_t j = 0; j <= num_steps; ++j) {
      float y = width * (static_cast<float>(j) / num_steps);

      for (size_t k = 0; k <= num_steps; ++k) {
        float z = height * (static_cast<float>(k) / num_steps);
        Eigen::Vector3f p_box(x, y, z);
        Eigen::Vector3f p_world = world_R_box * p_box + world_p_box;
        adaptor.points.push_back({{p_world.x(), p_world.y(), p_world.z()}});
      }
    }
  }

  // get bounding box from pointcloud
  BoundingBox box = bounding_box::extract(adaptor, BoundingBox::Type::RAABB);
  EXPECT_EQ(BoundingBox::Type::RAABB, box.type);

  EXPECT_NEAR(length, box.dimensions(0), 1.0e-6);
  EXPECT_NEAR(width, box.dimensions(1), 1.0e-6);
  EXPECT_NEAR(height, box.dimensions(2), 1.0e-6);

  const Eigen::Vector3f expected_pos = world_R_box * box_centroid + world_p_box;
  EXPECT_NEAR(0.0f, (expected_pos - box.world_P_center).norm(), 1.0e-6f)
      << "box: " << box.world_P_center.transpose();

  Eigen::Quaternionf expected_rotation(world_R_box);
  EXPECT_NEAR(0.0f, getRotationError(expected_rotation, box), 1.0e-6) << box;
}

/*
TEST(BoundingBoxTests, PCLConstructorOBB) {
  // tolerance is low because PCL OBB is not very accurate
  constexpr const float TOLERANCE = 1.0e-2f;

  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
  // lower and upper x
  cloud->push_back(PointXYZ(0.5f, 0.0f, 0.0f));
  cloud->push_back(PointXYZ(-0.5f, 0.0f, 0.0f));
  // lower and upper y
  cloud->push_back(PointXYZ(0.0f, 1.5f, 0.0f));
  cloud->push_back(PointXYZ(0.0f, -0.5f, 0.0f));
  // lower and upper z
  cloud->push_back(PointXYZ(0.0f, 0.0f, -1.0f));
  cloud->push_back(PointXYZ(0.0f, 0.0f, 4.0f));

  // get bounding box from pointcloud
  BoundingBox box = bounding_box::extract(cloud, BoundingBox::Type::OBB);
  EXPECT_EQ(BoundingBox::Type::OBB, box.type);

  Eigen::Vector3f expected_min;
  expected_min << -2.5f, -1.0f, -0.5f;
  Eigen::Vector3f expected_max = -1.0f * expected_min;
  Eigen::Vector3f expected_position;
  expected_position << 0.0f, 0.5f, 1.5f;
  // new axis: x = z, y = y, z = x
  // expected quaterion is non-trivial, so grabbed from PCL output
  Eigen::Quaternionf expected_rotation(
      0.0f, 1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f));

  EXPECT_NEAR(0.0f, (box.min - expected_min).norm(), TOLERANCE);
  EXPECT_NEAR(0.0f, (box.max - expected_max).norm(), TOLERANCE);
  // position and rotation are especially bad
  EXPECT_NEAR(0.0f, (box.world_P_center - expected_position).norm(), 0.1f);
  EXPECT_NEAR(0.0f, getRotationError(expected_rotation, box), 0.1f);
}
*/

}  // namespace spark_dsg
