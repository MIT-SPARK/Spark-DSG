#include <kimera_dsg_visualizer/dsg_mesh_plugin.h>

#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

namespace kimera {

using visualization_msgs::Marker;

DsgMeshPlugin::DsgMeshPlugin(const ros::NodeHandle& nh, const std::string& name)
    : DsgVisualizerPlugin(nh, name), name_(name), published_mesh_(false) {
  ros::NodeHandle temp(nh);
  mesh_pub_ = temp.advertise<Marker>("mesh", 1, true);
}

inline geometry_msgs::Point pointFromPcl(const pcl::PointXYZRGBA& point) {
  geometry_msgs::Point msg;
  msg.x = point.x;
  msg.y = point.y;
  msg.z = point.z;
  return msg;
}

inline std_msgs::ColorRGBA colorFromPcl(const pcl::PointXYZRGBA& point) {
  std_msgs::ColorRGBA msg;
  msg.r = static_cast<float>(point.r) / 255.0f;
  msg.g = static_cast<float>(point.g) / 255.0f;
  msg.b = static_cast<float>(point.b) / 255.0f;
  msg.a = 1.0;
  return msg;
}

void DsgMeshPlugin::draw(const std_msgs::Header& header,
                         const DynamicSceneGraph& graph) {
  if (!graph.hasMesh()) {
    ROS_WARN("Attempting to visualize unitialized mesh");
    return;
  }

  // TODO(nathan) check if we actually need a redraw

  Marker msg;
  msg.header = header;
  msg.ns = name_;
  msg.id = 0;
  msg.type = Marker::TRIANGLE_LIST;
  msg.action = Marker::ADD;

  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  tf2::convert(origin, msg.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), msg.pose.orientation);

  msg.scale.x = 1.0;
  msg.scale.y = 1.0;
  msg.scale.z = 1.0;

  const auto& vertices = *graph.getMeshVertices();
  const auto& faces = *graph.getMeshFaces();

  for (const auto& face : faces) {
    for (const auto& vertex_idx : face.vertices) {
      const auto& point = vertices.at(vertex_idx);
      msg.points.push_back(pointFromPcl(point));
      msg.colors.push_back(colorFromPcl(point));
    }
  }

  mesh_pub_.publish(msg);
  published_mesh_ = true;
}

void DsgMeshPlugin::reset(const std_msgs::Header& header, const DynamicSceneGraph&) {
  Marker msg;
  msg.header = header;
  msg.ns = name_;
  msg.id = 0;
  msg.action = Marker::DELETE;

  mesh_pub_.publish(msg);
}

}  // namespace kimera
