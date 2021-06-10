#pragma once
#include <pcl_ros/point_cloud.h>

namespace kimera {

//! alias for normal pcl point
typedef pcl::PointXYZ Point;
//! alias for colored pcl point
typedef pcl::PointXYZRGB ColorPoint;
//! alias for normal pcl point cloud
typedef pcl::PointCloud<Point> PointCloud;
//! alias for colored pcl point cloud
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;
//! alias for intensity pcl point
typedef pcl::PointXYZI IntensityPoint;
//! alias for intensity pcl point cloud
typedef pcl::PointCloud<IntensityPoint> IntensityPointCloud;

}  // namespace kimera
