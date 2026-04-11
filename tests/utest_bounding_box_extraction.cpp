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
#include <numbers>

namespace spark_dsg {

using bounding_box::BoxResult2D;

namespace {

inline float getRotationError(const Eigen::Quaternionf& rotation,
                              const BoundingBox& box) {
  // we only care up to 180 degrees orientation
  return std::fmod(rotation.angularDistance(Eigen::Quaternionf(box.world_R_center)),
                   std::numbers::pi);
}

struct TestAdaptor : public bounding_box::PointAdaptor {
  size_t size() const override { return points.size(); }

  Eigen::Vector3f get(size_t index) const override {
    return Eigen::Map<const Eigen::Vector3f>(points.at(index).data());
  }

  std::vector<std::array<float, 3>> points;
};

inline float computeIoU(const BoxResult2D& lhs, const BoxResult2D& rhs) {
  BoundingBox box_lhs(Eigen::Vector3f(lhs.dims.x(), lhs.dims.y(), 1.0),
                      Eigen::Vector3f(lhs.center.x(), lhs.center.y(), 0.0),
                      lhs.yaw);
  BoundingBox box_rhs(Eigen::Vector3f(rhs.dims.x(), rhs.dims.y(), 1.0),
                      Eigen::Vector3f(rhs.center.x(), rhs.center.y(), 0.0),
                      rhs.yaw);
  return box_lhs.computeIoU(box_rhs);
}

}  // namespace

TEST(BoundingBoxExtraction, ConvexHull) {
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

struct BoxPointPair {
  std::vector<Eigen::Vector2f> points;
  Eigen::Vector2f center;
  Eigen::Vector2f dim;
  float yaw;
};

struct Box2dFixture : public testing::TestWithParam<BoxPointPair> {
  Box2dFixture() {}
  virtual ~Box2dFixture() = default;
};

const BoxPointPair box_2d_test_cases[] = {
    {
        // test case 0
        {
            // points
            {293.5415669288347, 294.1611649378775},
            {337.89583570804365, 254.47576655648004},
            {370.5779284927239, 280.1545537444431},
            {320.7766442494016, 569.624518408754},
            {272.5316501386831, 585.1874197347923},
            {230.5118165583799, 550.9490368175082},
        },
        {309.8407897949219, 420.96734619140625},
        {336.75946044921875, 92.11454010009766},
        -1.3301002269476985,
    },
    {
        // test case 1
        {
            // points
            {165.14763098901932, 298.05189026938706},
            {255.41245868004103, 323.7306774573501},
            {197.0515787073977, 536.9424256240738},
        },
        {223.75648498535156, 411.80035400390625},
        {86.07124328613281, 241.01156616210938},
        -0.13276486373614693,
    },
    {
        // test case 2
        {
            // points
            {218.83964056385122, 121.4129602188533},
            {193.93899844219004, 200.00561191534632},
            {277.97866560279647, 216.34665830768643},
            {295.0978570614385, 138.53215167749534},
        },
        {244.55438232421875, 169.10560607910156},
        {82.10726928710938, 85.58794403076172},
        -1.3542459101289566,
    },
    {
        // test case 3
        {
            // points
            {27.684988459078227, 259.97798882060135},
            {75.38375650045873, 169.5639061152981},
            {151.55940098445444, 226.5176590005285},
            {136.60904085208142, 282.75948997469345},
            {63.281084012347264, 316.2198197947664},
        },
        {91.10934448242188, 254.38047790527344},
        {135.3591766357422, 106.969482421875},
        -1.0853454030201926,
    }};

INSTANTIATE_TEST_SUITE_P(BoundingBoxExtraction,
                         Box2dFixture,
                         testing::ValuesIn(box_2d_test_cases));

TEST_P(Box2dFixture, ExtractionCorrect) {
  const auto param = GetParam();

  TestAdaptor adaptor;
  for (const auto& p : param.points) {
    auto& point = adaptor.points.emplace_back();
    point = {p.x(), p.y(), 0.0f};
  }

  const BoxResult2D expected{param.center, param.dim, 0.0, param.yaw};
  const auto result = bounding_box::getMin2DBox(adaptor);

  EXPECT_TRUE(result.min_area);
  EXPECT_GT(result.min_area.value(), 0.0);
  EXPECT_NEAR(computeIoU(result, expected), 1.0f, 1.0e-2);
}

TEST(BoundingBoxExtraction, InvalidFromPoints) {
  BoundingBox box;
  EXPECT_EQ(box.type, BoundingBox::Type::INVALID);

  TestAdaptor adaptor;
  box = bounding_box::extract(adaptor, BoundingBox::Type::INVALID);
  EXPECT_EQ(box.type, BoundingBox::Type::INVALID);
}

TEST(BoundingBoxExtraction, AABBFromPoints) {
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

TEST(BoundingBoxExtraction, RAABBFromTwoPoints) {
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

TEST(BoundingBoxExtraction, RAABBFromPoints) {
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

TEST(BoundingBoxExtraction, RAABBFromPointsNonTrivial) {
  TestAdaptor adaptor;
  const size_t num_steps = 2;
  const float angle = std::numbers::pi / 6.0f;
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

  const Eigen::Vector3f expected_pos = world_R_box * box_centroid + world_p_box;
  const BoundingBox expected(
      Eigen::Vector3f(length, width, height), expected_pos, angle);
  EXPECT_NEAR(1.0f, box.computeIoU(expected, 5000), 1.0e-2f);
}

}  // namespace spark_dsg
