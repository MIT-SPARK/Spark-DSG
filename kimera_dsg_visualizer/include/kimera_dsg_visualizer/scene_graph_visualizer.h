#pragma once
#include "kimera_dsg_visualizer/visualizer_types.h"

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace kimera {

class SceneGraphVisualizer {
 public:
  using RqtMutexPtr = std::unique_ptr<boost::recursive_mutex>;
  using LayerRqtServer = dynamic_reconfigure::Server<LayerConfig>;
  using LayerRqtCb = LayerRqtServer::CallbackType;
  using RqtServer = dynamic_reconfigure::Server<VisualizerConfig>;
  using ColormapRqtServer = dynamic_reconfigure::Server<ColormapConfig>;

  SceneGraphVisualizer(const ros::NodeHandle& nh,
                       const SceneGraph::LayerIds& layer_ids);

  virtual ~SceneGraphVisualizer() = default;

  void start();

  virtual bool redraw();

  inline void setGraphUpdated() {
    need_redraw_ = true;
  }

  void setGraph(const DynamicSceneGraph::Ptr& scene_graph);

  void clear();

 protected:
  void displayLoop(const ros::TimerEvent&);


  void configUpdateCb(VisualizerConfig& config, uint32_t level);

  void colormapUpdateCb(ColormapConfig& config, uint32_t level);

  void layerConfigUpdateCb(LayerId layer_id, LayerConfig& config, uint32_t level);

  void fillHeader(visualization_msgs::Marker& marker,
                  const ros::Time& current_time) const;

  void displayLayers(const SceneGraph& scene_graph) const;

  void displayEdges(const SceneGraph& scene_graph) const;

  void handleCentroids(const SceneGraphLayer& layer,
                       const LayerConfig& config,
                       const ros::Time& current_time,
                       visualization_msgs::MarkerArray& markers) const;

  void handleMeshEdges(const SceneGraphLayer& layer,
                       const LayerConfig& config,
                       const ros::Time& current_time,
                       visualization_msgs::MarkerArray& markers) const;

  void handleLabels(const SceneGraphLayer& layer,
                    const LayerConfig& config,
                    const ros::Time& current_time,
                    visualization_msgs::MarkerArray& markers) const;

  void handleBoundingBoxes(const SceneGraphLayer& layer,
                           const LayerConfig& config,
                           const ros::Time& current_time,
                           visualization_msgs::MarkerArray& markers) const;

 private:
  void setupDynamicReconfigure(const SceneGraph::LayerIds& layer_ids);

 protected:
  DynamicSceneGraph::Ptr scene_graph_;

  ros::NodeHandle nh_;

  bool need_redraw_;
  std::string world_frame_;
  std::string visualizer_ns_;
  std::string visualizer_layer_ns_;

  ros::Timer visualizer_loop_timer_;

  std::unique_ptr<RqtServer> config_server_;
  RqtMutexPtr config_server_mutex_;
  std::unique_ptr<ColormapRqtServer> colormap_server_;
  RqtMutexPtr colormap_server_mutex_;
  std::map<LayerId, RqtMutexPtr> layer_config_server_mutexes_;
  std::map<LayerId, std::unique_ptr<LayerRqtServer>> layer_config_servers_;
  std::map<LayerId, LayerRqtCb> layer_config_cb_;

  std::map<LayerId, LayerConfig> layer_configs_;
  VisualizerConfig visualizer_config_;
  ColormapConfig places_colormap_;

  ros::Publisher semantic_instance_centroid_pub_;
  ros::Publisher bounding_box_pub_;
  ros::Publisher edges_centroid_pcl_pub_;
  ros::Publisher edges_node_node_pub_;
  ros::Publisher text_markers_pub_;
};

}  // namespace kimera
