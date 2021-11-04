#pragma once
#include "kimera_dsg_visualizer/dsg_visualizer_plugin.h"
#include "kimera_dsg_visualizer/scene_graph_visualizer.h"

#include <pcl/PolygonMesh.h>

namespace kimera {

class DynamicSceneGraphVisualizer : public SceneGraphVisualizer {
 public:
  using DynamicLayerConfigManager = ConfigManager<DynamicLayerConfig>;

  DynamicSceneGraphVisualizer(const ros::NodeHandle& nh,
                              const SceneGraph::LayerIds& layer_ids);

  virtual ~DynamicSceneGraphVisualizer() = default;

  void addPlugin(const std::shared_ptr<DsgVisualizerPlugin>& plugin) {
    plugins_.push_back(plugin);
  }

 protected:
  virtual void resetImpl(const std_msgs::Header& header, MarkerArray& msg) override;

  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg);

  inline std::string getDynamicNodeNamespace(char layer_prefix) const {
    return dynamic_node_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicEdgeNamespace(char layer_prefix) const {
    return dynamic_edge_ns_prefix_ + layer_prefix;
  }

  inline std::string getDynamicLabelNamespace(char layer_prefix) const {
    return dynamic_label_ns_prefix_ + layer_prefix;
  }

  virtual bool hasConfigChanged() const override;

  virtual void clearConfigChangeFlags() override;

 private:
  const DynamicLayerConfig& getConfig(LayerId layer);

  void drawDynamicLayer(const std_msgs::Header& header,
                        const DynamicSceneGraphLayer& layer,
                        const DynamicLayerConfig& config,
                        const LayerConfig& layer_config,
                        const VisualizerConfig& viz_config,
                        MarkerArray& msg);

  void deleteLabel(const std_msgs::Header& header, char prefix, MarkerArray& msg);

  void deleteDynamicLayer(const std_msgs::Header& header,
                          char prefix,
                          MarkerArray& msg);

  void drawDynamicLayers(const std_msgs::Header& header, MarkerArray& msg);

 protected:
  std::map<LayerId, DynamicLayerConfigManager::Ptr> dynamic_configs_;

  std::list<std::shared_ptr<DsgVisualizerPlugin>> plugins_;

  const std::string dynamic_node_ns_prefix_ = "dynamic_nodes_";
  const std::string dynamic_edge_ns_prefix_ = "dynamic_edges_";
  const std::string dynamic_label_ns_prefix_ = "dynamic_label_";

  std::set<std::string> published_dynamic_labels_;

  ros::Publisher dynamic_layers_viz_pub_;
};

}  // namespace kimera
