#include "kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h"

#include <kimera_dsg/scene_graph.h>
#include <ros/ros.h>

#include <glog/logging.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kimera_dsg_visualizer_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string visualizer_ns;
  nh_private.param<std::string>(
      "visualizer_ns", visualizer_ns, "/kimera_dsg_visualizer");

  std::string scene_graph_input_path;
  nh_private.param<std::string>("scene_graph_input_path", scene_graph_input_path, "");

  if (scene_graph_input_path.empty()) {
    ROS_FATAL("Empty scene graph input path...");
    ros::shutdown();
    return 1;
  }

  ROS_DEBUG_STREAM("Loading scene graph from: " << scene_graph_input_path.c_str());
  kimera::DynamicSceneGraph::Ptr scene_graph(new kimera::DynamicSceneGraph());
  scene_graph->load(scene_graph_input_path);
  ROS_DEBUG("Loaded scene graph");

  kimera::DynamicSceneGraphVisualizer scene_graph_visualizer(
      ros::NodeHandle(visualizer_ns), kimera::getDefaultLayerIds());

  scene_graph_visualizer.setGraph(scene_graph);
  scene_graph_visualizer.start();
  ROS_DEBUG("Visualizer running");
  ros::spin();

  return 0;
}
