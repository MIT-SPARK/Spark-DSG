#pragma once
#include "kimera_dsg_visualizer/visualizer_types.h"

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace kimera {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

template <typename Config>
class ConfigManager {
 public:
  using Ptr = std::shared_ptr<ConfigManager<Config>>;
  using Server = dynamic_reconfigure::Server<Config>;

  ConfigManager(const ros::NodeHandle& nh,
                const std::string& ns,
                std::function<void(const ros::NodeHandle& nh, Config&)> init_func)
      : nh_(nh, ns), changed_(true), mutex_(new boost::recursive_mutex()) {
    init_func(nh_, config_);
    server_ = std::make_unique<Server>(*mutex_, nh_);

    {  // start critical region
      boost::recursive_mutex::scoped_lock lock(*mutex_);
      server_->updateConfig(config_);
    }  // end critical region

    server_->setCallback(
        boost::bind(&ConfigManager<Config>::updateCallback, this, _1, _2));
  }

  ConfigManager(const ros::NodeHandle& nh, const std::string& ns)
      : ConfigManager(nh, ns, [](const ros::NodeHandle&, Config&) {}) {}

  bool hasChange() { return changed_; }

  void clearChangeFlag() { changed_ = false; }

  const Config& get() const { return config_; };

 private:
  void updateCallback(Config& config, uint32_t) {
    config_ = config;
    changed_ = true;
  }

  ros::NodeHandle nh_;

  bool changed_;
  Config config_;

  std::unique_ptr<boost::recursive_mutex> mutex_;
  std::unique_ptr<Server> server_;
};

class SceneGraphVisualizer {
 public:
  using VisualizerConfigManager = ConfigManager<VisualizerConfig>;
  using LayerConfigManager = ConfigManager<LayerConfig>;
  using ColormapConfigManager = ConfigManager<ColormapConfig>;

  SceneGraphVisualizer(const ros::NodeHandle& nh,
                       const SceneGraph::LayerIds& layer_ids);

  virtual ~SceneGraphVisualizer() = default;

  void start();

  bool redraw();

  inline void setGraphUpdated() { need_redraw_ = true; }

  void setGraph(const DynamicSceneGraph::Ptr& scene_graph);

 protected:
  virtual bool hasConfigChanged() const;

  virtual void clearConfigChangeFlags();

  virtual void redrawImpl(const std_msgs::Header& header, MarkerArray& msg);

  inline std::string getLayerNodeNamespace(LayerId layer) const {
    return node_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerEdgeNamespace(LayerId layer) const {
    return edge_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerLabelNamespace(LayerId layer) const {
    return label_ns_prefix_ + std::to_string(layer);
  }

  inline std::string getLayerBboxNamespace(LayerId layer) const {
    return bbox_ns_prefix_ + std::to_string(layer);
  }

  void deleteMultiMarker(const std_msgs::Header& header,
                         const std::string& ns,
                         MarkerArray& msg);

  void addMultiMarkerIfValid(const Marker& marker, MarkerArray& msg);

 private:
  void setupConfigs(const SceneGraph::LayerIds& layer_ids);

  void displayLoop(const ros::WallTimerEvent&);

  void deleteLayer(const std_msgs::Header& header,
                   const SceneGraphLayer& layer,
                   MarkerArray& msg);

  void drawLayer(const std_msgs::Header& header,
                 const SceneGraphLayer& layer,
                 const LayerConfig& config,
                 MarkerArray& msg);

  void drawLayerMeshEdges(const std_msgs::Header& header,
                          LayerId layer_id,
                          const std::string& ns,
                          MarkerArray& msg);

 protected:
  DynamicSceneGraph::Ptr scene_graph_;

  ros::NodeHandle nh_;

  bool need_redraw_;
  std::string world_frame_;
  std::string visualizer_ns_;
  std::string visualizer_layer_ns_;

  ros::WallTimer visualizer_loop_timer_;

  const std::string node_ns_prefix_ = "layer_nodes_";
  const std::string edge_ns_prefix_ = "layer_edges_";
  const std::string label_ns_prefix_ = "layer_labels_";
  const std::string bbox_ns_prefix_ = "layer_bounding_boxes_";
  const std::string mesh_edge_ns_ = "mesh_object_connections";
  const std::string interlayer_edge_ns_prefix_ = "interlayer_edges_";
  const LayerId mesh_edge_source_layer_ = KimeraDsgLayers::OBJECTS;

  std::set<std::string> published_multimarkers_;
  std::map<LayerId, std::set<NodeId>> prev_labels_;
  std::map<LayerId, std::set<NodeId>> curr_labels_;
  std::map<LayerId, std::set<NodeId>> prev_bboxes_;
  std::map<LayerId, std::set<NodeId>> curr_bboxes_;

  std::map<LayerId, LayerConfigManager::Ptr> layer_configs_;
  VisualizerConfigManager::Ptr visualizer_config_;
  ColormapConfigManager::Ptr places_colormap_;

  ros::Publisher dsg_pub_;
};

}  // namespace kimera
