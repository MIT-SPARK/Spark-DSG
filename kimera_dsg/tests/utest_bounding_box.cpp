#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <kimera_dsg/bounding_box.h>

using kimera::BoundingBox;
using pcl::PointXYZ;

TEST(BoundingBoxTests, InvalidConstructor) {
  BoundingBox box;
  EXPECT_EQ(box.type, BoundingBox::Type::INVALID);

  try {
    pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
    box = BoundingBox::extract(cloud, BoundingBox::Type::INVALID);
    FAIL() << "Should not be able to extract a bounding box of type INVALID";
  } catch (const std::runtime_error& e) {
    SUCCEED();
  }
}

inline float getRotationError(const Eigen::Quaternionf& rotation,
                              const BoundingBox& box) {
  return rotation.angularDistance(Eigen::Quaternionf(box.world_R_center));
}

TEST(BoundingBoxTests, AABBConstructor) {
  Eigen::Vector3f min;
  min << 1.0f, 2.0f, 3.0f;
  Eigen::Vector3f max;
  max << 3.0f, 4.0f, 5.0f;
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

TEST(BoundingBoxTests, PCLConstructorAABB) {
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
  // lower and upper x
  cloud->push_back(PointXYZ(1.0f, 0.0f, 0.0f));
  cloud->push_back(PointXYZ(-1.0f, 0.0f, 0.0f));
  // lower and upper y
  cloud->push_back(PointXYZ(0.0f, 1.5f, 0.0f));
  cloud->push_back(PointXYZ(0.0f, -0.5f, 0.0f));
  // lower and upper z
  cloud->push_back(PointXYZ(0.0f, 0.0f, 1.0f));
  cloud->push_back(PointXYZ(0.0f, 0.0f, 4.0f));

  // get bounding box from pointcloud
  BoundingBox box = BoundingBox::extract(cloud);
  EXPECT_EQ(BoundingBox::Type::AABB, box.type);

  EXPECT_EQ(-1.0f, box.min(0));
  EXPECT_EQ(1.0f, box.max(0));
  EXPECT_EQ(-0.5f, box.min(1));
  EXPECT_EQ(1.5f, box.max(1));
  EXPECT_EQ(0.0f, box.min(2));  // "lower z" is superseded by x and y points
  EXPECT_EQ(4.0f, box.max(2));
}

TEST(BoundingBoxTests, PCLConstructorAABBSpecifiedIndices) {
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
  // lower and upper x
  cloud->push_back(PointXYZ(1.0f, 0.0f, 0.0f));
  cloud->push_back(PointXYZ(-1.0f, 0.0f, 0.0f));
  // lower and upper y
  cloud->push_back(PointXYZ(0.0f, 1.5f, 0.0f));
  cloud->push_back(PointXYZ(0.0f, -0.5f, 0.0f));
  // lower and upper z
  cloud->push_back(PointXYZ(0.0f, 0.0f, 1.0f));
  cloud->push_back(PointXYZ(0.0f, 0.0f, 4.0f));

  // add some points to influence the result
  cloud->push_back(PointXYZ(-5.0f, -5.0f, -5.0f));
  cloud->push_back(PointXYZ(-5.0f, -5.0f, -5.0f));
  cloud->push_back(PointXYZ(-5.0f, -5.0f, -5.0f));

  pcl::IndicesPtr indices(new std::vector<int>{0, 1, 2, 3, 4, 5});

  // get bounding box from pointcloud
  BoundingBox box = BoundingBox::extract(cloud, BoundingBox::Type::AABB, indices);

  EXPECT_EQ(-1.0f, box.min(0));
  EXPECT_EQ(1.0f, box.max(0));
  EXPECT_EQ(-0.5f, box.min(1));
  EXPECT_EQ(1.5f, box.max(1));
  EXPECT_EQ(0.0f, box.min(2));  // "lower z" is superseded by x and y points
  EXPECT_EQ(4.0f, box.max(2));
}

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
  BoundingBox box = BoundingBox::extract(cloud, BoundingBox::Type::OBB);
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

pcl::PointXYZRGB makeRGBPoint(float x, float y, float z) {
  pcl::PointXYZRGB point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST(BoundingBoxTests, PCLConstructorAABBColor) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  // lower and upper x
  cloud->push_back(makeRGBPoint(1.0f, 0.0f, 0.0f));
  cloud->push_back(makeRGBPoint(-1.0f, 0.0f, 0.0f));
  // lower and upper y
  cloud->push_back(makeRGBPoint(0.0f, 1.5f, 0.0f));
  cloud->push_back(makeRGBPoint(0.0f, -0.5f, 0.0f));
  // lower and upper z
  cloud->push_back(makeRGBPoint(0.0f, 0.0f, 1.0f));
  cloud->push_back(makeRGBPoint(0.0f, 0.0f, 4.0f));

  // get bounding box from pointcloud
  BoundingBox box = BoundingBox::extract(cloud);
  EXPECT_EQ(BoundingBox::Type::AABB, box.type);

  EXPECT_EQ(-1.0f, box.min(0));
  EXPECT_EQ(1.0f, box.max(0));
  EXPECT_EQ(-0.5f, box.min(1));
  EXPECT_EQ(1.5f, box.max(1));
  EXPECT_EQ(0.0f, box.min(2));  // "lower z" is superseded by x and y points
  EXPECT_EQ(4.0f, box.max(2));
}

TEST(BoundingBoxTests, InvalidVolumeChecksCorrect) {
  Eigen::Vector3f test_point1(1.0, 2.0, 3.0);
  Eigen::Vector3d test_point2(1.0, 2.0, 3.0);

  BoundingBox box;
  EXPECT_FALSE(std::isfinite(box.volume()));
  try {
    box.isInside(test_point1);
    FAIL();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }

  try {
    box.isInside(test_point2);
    FAIL();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }
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
