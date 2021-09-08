#pragma once
#include "kimera_dsg_visualizer/dsg_visualizer_plugin.h"
#include "kimera_dsg_visualizer/scene_graph_visualizer.h"

#include <pcl/PolygonMesh.h>

namespace kimera {

class DynamicSceneGraphVisualizer : public SceneGraphVisualizer {
 public:
  DynamicSceneGraphVisualizer(const ros::NodeHandle& nh,
                              const SceneGraph::LayerIds& layer_ids);

  virtual ~DynamicSceneGraphVisualizer() = default;

  void visualizeMesh(const pcl::PolygonMesh& mesh, bool is_rgb_mesh = true) const;

  void visualizeWalls(const pcl::PolygonMesh& mesh) const;

  virtual bool redraw() override;

  void addPlugin(const std::shared_ptr<DsgVisualizerPlugin>& plugin) {
    plugins_.push_back(plugin);
  }

 protected:
  void visualizeDsgMesh() const;

  visualization_msgs::MarkerArray makeDynamicLayerMarkers() const;

 protected:
  std::list<std::shared_ptr<DsgVisualizerPlugin>> plugins_;

  ros::Publisher wall_pub_;
  ros::Publisher semantic_mesh_pub_;
  ros::Publisher rgb_mesh_pub_;

  ros::Publisher dynamic_layers_viz_pub_;
};

}  // namespace kimera
