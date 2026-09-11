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
#include <algorithm>
#include <limits>
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

void checkEnclosure(const std::vector<Eigen::Vector3f>& points,
                    double expected_area,
                    double area_tolerance) {
  const BoundingBox::PointVectorAdaptor adaptor(points);
  const auto hull = bounding_box::get2dConvexHull(adaptor);
  ASSERT_GE(hull.size(), 3u);
  std::vector<size_t> indices(hull.begin(), hull.end());
  for (size_t i = 0; i < indices.size(); ++i) {
    const Eigen::Vector2d a = points[indices[i]].head<2>().cast<double>();
    const Eigen::Vector2d edge =
        points[indices[(i + 1) % indices.size()]].head<2>().cast<double>() - a;
    for (const auto& point : points) {
      const Eigen::Vector2d offset = point.head<2>().cast<double>() - a;
      EXPECT_GE(edge.x() * offset.y() - edge.y() * offset.x(), -1.0e-12);
    }
  }

  const BoundingBox box(points, BoundingBox::Type::RAABB);
  ASSERT_TRUE(box.isValid());
  EXPECT_NEAR(static_cast<double>(box.dimensions.x()) * box.dimensions.y(),
              expected_area,
              area_tolerance);
  for (const auto& point : points) {
    const Eigen::Vector3d local =
        box.world_R_center.cast<double>().transpose() *
        (point.cast<double>() - box.world_P_center.cast<double>());
    // Float center, dimensions and rotation each round once. Account for their
    // scale, including center rounding for boxes far from the origin.
    const double tolerance = 4.0 * std::numeric_limits<float>::epsilon() *
                             (point.cast<double>().cwiseAbs().maxCoeff() +
                              box.dimensions.cast<double>().maxCoeff());
    EXPECT_LE((local.cwiseAbs() - box.dimensions.cast<double>() / 2.0).maxCoeff(),
              tolerance);
  }
}

std::vector<Eigen::Vector3f> angularTiePoints() {
  // Minimized from the saved trashcan. The first two directions from the last
  // point share a float angle key despite having different polar angles.
  return {{-8.850000381469727f, 31.249998092651367f, 1.047795295715332f},
          {-8.849999427795410f, 31.249998092651367f, 1.354702115058899f},
          {-8.843938827514648f, 31.250000000000000f, 1.049999952316284f},
          {-9.349999427795410f, 31.192356109619140f, 1.149999976158142f}};
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
  // This triangle has several minimum rectangles with different centers/yaws.
  EXPECT_NEAR(12.5f, box.dimensions.x() * box.dimensions.y(), 1.0e-5f);
  EXPECT_NEAR(1.0f, box.dimensions.z(), 1.0e-6f);
  EXPECT_NEAR(0.5f, box.world_P_center.z(), 1.0e-6f);
  for (size_t i = 0; i < adaptor.size(); ++i) {
    const Eigen::Vector3f local =
        box.world_R_center.transpose() * (adaptor[i] - box.world_P_center);
    EXPECT_LE((local.cwiseAbs() - box.dimensions / 2.0f).maxCoeff(), 1.0e-6f);
  }
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

TEST(BoundingBoxExtraction, AngularTieEnclosure) {
  const auto original = angularTiePoints();
  std::array<size_t, 4> order{0, 1, 2, 3};
  do {
    std::vector<Eigen::Vector3f> points;
    for (const auto i : order) {
      points.push_back(original[i]);
    }

    // Independent minimum rectangle of the represented float coordinates.
    checkEnclosure(points, 0.00034844631772374903, 1.0e-10);
  } while (std::next_permutation(order.begin(), order.end()));
}

TEST(BoundingBoxExtraction, ThinRectangleFromSuppliedHull) {
  const auto points = angularTiePoints();
  const BoundingBox::PointVectorAdaptor adaptor(points);
  for (const auto& hull : {std::list<size_t>{3, 2, 0}, std::list<size_t>{0, 2, 3}}) {
    const auto result = bounding_box::getMin2DBox(adaptor, hull);
    ASSERT_TRUE(result.min_area);
    EXPECT_NEAR(*result.min_area, 0.00034844631772374903, 1.0e-10);
  }
}

TEST(BoundingBoxExtraction, RepeatedRotatedTranslatedPoints) {
  for (const float width : {0.0001f, 0.5f}) {
    for (const float angle : {0.0f, 0.3f, 1.7f, -2.4f}) {
      for (const float distance : {0.0f, 32.0f, 10000.0f}) {
        // At 10 km, float coordinates cannot represent the thin rectangle.
        if (width < 0.001f && distance > 100.0f) {
          continue;
        }

        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const Eigen::Vector3f translation(distance, -distance, distance);
        std::vector<Eigen::Vector3f> points;
        for (const float x : {0.0f, 1.0f, 2.0f}) {
          for (const float y : {0.0f, width}) {
            for (const float z : {0.0f, 0.75f}) {
              const Eigen::Vector3f point =
                  rotation * Eigen::Vector3f(x, y, z) + translation;
              points.push_back(point);
              points.push_back(point);
            }
          }
        }

        // Coordinate quantization perturbs the nominal area by perimeter times
        // coordinate error. The factor also covers float output rounding.
        const double area_tolerance =
            16.0 * std::numeric_limits<float>::epsilon() * (distance + 2.0f);
        checkEnclosure(points, 2.0 * width, area_tolerance);
        std::reverse(points.begin(), points.end());
        checkEnclosure(points, 2.0 * width, area_tolerance);
      }
    }
  }
}

TEST(BoundingBoxExtraction, DegenerateHullInputs) {
  std::vector<Eigen::Vector3f> points;
  const BoundingBox::PointVectorAdaptor adaptor(points);
  EXPECT_TRUE(bounding_box::get2dConvexHull(adaptor).empty());
  EXPECT_FALSE(bounding_box::getMin2DBox(adaptor).min_area);
  EXPECT_EQ(BoundingBox(points, BoundingBox::Type::RAABB).type,
            BoundingBox::Type::INVALID);
  points = {{1.0f, 2.0f, 0.0f}};
  EXPECT_EQ(bounding_box::get2dConvexHull(adaptor).size(), 1u);
  EXPECT_EQ(BoundingBox(points, BoundingBox::Type::RAABB).type,
            BoundingBox::Type::INVALID);
  points.push_back({1.0f, 2.0f, 4.0f});
  EXPECT_EQ(bounding_box::get2dConvexHull(adaptor).size(), 1u);
  EXPECT_EQ(BoundingBox(points, BoundingBox::Type::RAABB).type,
            BoundingBox::Type::INVALID);
  points.insert(points.end(), {{2.0f, 2.0f, 1.0f}, {3.0f, 2.0f, 2.0f}});
  EXPECT_EQ(bounding_box::get2dConvexHull(adaptor).size(), 2u);
  const BoundingBox box(points, BoundingBox::Type::RAABB);
  EXPECT_EQ(box.type, BoundingBox::Type::RAABB);
  EXPECT_FALSE(box.isValid());
  EXPECT_EQ(box.dimensions, Eigen::Vector3f(2.0f, 0.0f, 4.0f));
  EXPECT_EQ(box.world_P_center, Eigen::Vector3f(2.0f, 2.0f, 2.0f));
  points = {{0.0f, 0.0f, 0.0f}, {1.0f, 3.0f, 1.0f}, {2.0f, 6.0f, 2.0f}};
  const BoundingBox diagonal(points, BoundingBox::Type::RAABB);
  EXPECT_EQ(bounding_box::get2dConvexHull(adaptor).size(), 2u);
  EXPECT_EQ(diagonal.type, BoundingBox::Type::RAABB);
  EXPECT_FALSE(diagonal.isValid());
  EXPECT_EQ(diagonal.dimensions.y(), 0.0f);
  EXPECT_NEAR(diagonal.dimensions.x(), std::sqrt(40.0f), 1.0e-6f);
  EXPECT_EQ(diagonal.world_P_center, Eigen::Vector3f(1.0f, 3.0f, 1.0f));
}

}  // namespace spark_dsg
