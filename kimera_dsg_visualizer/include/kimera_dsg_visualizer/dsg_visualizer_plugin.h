#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>
#include <ros/ros.h>

namespace kimera {

class DsgVisualizerPlugin {
 public:
  DsgVisualizerPlugin(const ros::NodeHandle& nh, const std::string& name)
      : nh_(nh, name) {}

  virtual ~DsgVisualizerPlugin() = default;

  virtual void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) = 0;

 protected:
  ros::NodeHandle nh_;
};

}  // namespace kimera
