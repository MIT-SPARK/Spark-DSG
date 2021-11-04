#pragma once
#include <kimera_dsg_visualizer/dsg_visualizer_plugin.h>

namespace kimera {

class DsgMeshPlugin : public DsgVisualizerPlugin {
 public:
  DsgMeshPlugin(const ros::NodeHandle& nh, const std::string& name);

  virtual ~DsgMeshPlugin() = default;

  void draw(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

  void reset(const std_msgs::Header& header, const DynamicSceneGraph& graph) override;

 private:
  std::string name_;
  ros::Publisher mesh_pub_;
  bool published_mesh_;
};

}  // namespace kimera
