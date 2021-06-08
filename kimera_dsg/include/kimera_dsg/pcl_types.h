#pragma once
#include <pcl_ros/point_cloud.h>

namespace kimera {

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB ColorPoint;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
typedef pcl::PointXYZI IntensityPoint;
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

}  // namespace kimera
