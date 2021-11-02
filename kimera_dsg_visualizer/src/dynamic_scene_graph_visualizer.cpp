#include "kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h"
#include "kimera_dsg_visualizer/colormap_utils.h"
#include "kimera_dsg_visualizer/visualizer_utils.h"

namespace kimera {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using ColorVector = SemanticNodeAttributes::ColorVector;

DynamicSceneGraphVisualizer::DynamicSceneGraphVisualizer(
    const ros::NodeHandle& nh,
    const SceneGraph::LayerIds& layer_ids)
    : SceneGraphVisualizer(nh, layer_ids) {
  dynamic_layers_viz_pub_ = nh_.advertise<MarkerArray>("dynamic_layers_viz", 1, true);
}

const DynamicLayerConfig& DynamicSceneGraphVisualizer::getConfig(LayerId layer) {
  if (!dynamic_configs_.count(layer)) {
    const std::string ns = visualizer_ns_ + "/dynamic_layer/" + std::to_string(layer);

    dynamic_configs_[layer] = std::make_shared<DynamicLayerConfigManager>(
        ros::NodeHandle(""), ns, [](const ros::NodeHandle& nh, auto& config) {
          config = getDynamicLayerConfig(nh);
        });
  }

  return dynamic_configs_.at(layer)->get();
}

ColorVector getNodeColor(const DynamicLayerConfig& config, char prefix) {
  // distance is measured from first relatively readable character prefix
  int color_num = (prefix - '0') % config.num_colors;
  double hue = static_cast<double>(color_num) / static_cast<double>(config.num_colors);
  return dsg_utils::getRgbFromHls(hue, config.luminance, config.saturation);
}

ColorVector getEdgeColor(const DynamicLayerConfig& config, char prefix) {
  // distance is measured from first relatively readable character prefix
  int color_num = (prefix - '0') % config.num_colors;
  double hue = static_cast<double>(color_num) / static_cast<double>(config.num_colors);
  const double saturation = config.saturation * config.edge_sl_ratio;
  const double luminance = config.luminance * config.edge_sl_ratio;
  return dsg_utils::getRgbFromHls(hue, saturation, luminance);
}

void DynamicSceneGraphVisualizer::drawDynamicLayer(const std_msgs::Header& header,
                                                   const DynamicSceneGraphLayer& layer,
                                                   const DynamicLayerConfig& config,
                                                   const LayerConfig& layer_config,
                                                   const VisualizerConfig& viz_config,
                                                   MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(layer.prefix);
  Marker nodes = makeDynamicCentroidMarkers(header,
                                            config,
                                            layer,
                                            layer_config,
                                            viz_config,
                                            getNodeColor(config, layer.prefix),
                                            node_ns);
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(layer.prefix);
  Marker edges = makeDynamicEdgeMarkers(header,
                                        config,
                                        layer,
                                        layer_config,
                                        viz_config,
                                        getEdgeColor(config, layer.prefix),
                                        edge_ns);
  addMultiMarkerIfValid(edges, msg);

  if (layer.numNodes() == 0) {
    deleteLabel(header, layer.prefix, msg);
    return;
  }

  const std::string label_ns = getDynamicLabelNamespace(layer.prefix);
  Marker label =
      makeDynamicLabelMarker(header, config, layer, layer_config, viz_config, label_ns);
  msg.markers.push_back(label);
  published_dynamic_labels_.insert(label_ns);
}

void DynamicSceneGraphVisualizer::deleteLabel(const std_msgs::Header& header,
                                              char prefix,
                                              MarkerArray& msg) {
  const std::string label_ns = getDynamicLabelNamespace(prefix);

  if (published_dynamic_labels_.count(label_ns)) {
    Marker marker = makeDeleteMarker(header, 0, label_ns);
    msg.markers.push_back(marker);
  }
  published_dynamic_labels_.erase(label_ns);
}

void DynamicSceneGraphVisualizer::deleteDynamicLayer(const std_msgs::Header& header,
                                                     char prefix,
                                                     MarkerArray& msg) {
  const std::string node_ns = getDynamicNodeNamespace(prefix);
  deleteMultiMarker(header, node_ns, msg);

  const std::string edge_ns = getDynamicEdgeNamespace(prefix);
  deleteMultiMarker(header, edge_ns, msg);

  deleteLabel(header, prefix, msg);
}

void DynamicSceneGraphVisualizer::drawDynamicLayers(const std_msgs::Header& header,
                                                    MarkerArray& msg) {
  const VisualizerConfig& viz_config = visualizer_config_->get();

  for (const auto& id_layer_map_pair : scene_graph_->dynamicLayers()) {
    const LayerId layer_id = id_layer_map_pair.first;
    if (!layer_configs_.count(layer_id)) {
      continue;
    }

    const LayerConfig& layer_config = layer_configs_.at(layer_id)->get();
    const DynamicLayerConfig& config = getConfig(layer_id);

    for (const auto& prefix_layer_pair : id_layer_map_pair.second) {
      if (!config.visualize) {
        deleteDynamicLayer(header, prefix_layer_pair.first, msg);
        continue;
      }

      const DynamicSceneGraphLayer& layer = *prefix_layer_pair.second;
      drawDynamicLayer(header, layer, config, layer_config, viz_config, msg);
    }
  }
}

void DynamicSceneGraphVisualizer::redrawImpl(const std_msgs::Header& header,
                                             MarkerArray& msg) {
  SceneGraphVisualizer::redrawImpl(header, msg);

  MarkerArray dynamic_markers;
  drawDynamicLayers(header, dynamic_markers);

  std::map<LayerId, LayerConfig> all_configs;
  for (const auto& id_manager_pair : layer_configs_) {
    all_configs[id_manager_pair.first] = id_manager_pair.second->get();
  }

  std::map<LayerId, DynamicLayerConfig> all_dynamic_configs;
  for (const auto& id_manager_pair : dynamic_configs_) {
    all_dynamic_configs[id_manager_pair.first] = id_manager_pair.second->get();
  }

  MarkerArray interlayer_edge_markers =
      makeDynamicGraphEdgeMarkers(header,
                                  *scene_graph_,
                                  all_configs,
                                  all_dynamic_configs,
                                  visualizer_config_->get(),
                                  "dynamic_interlayer_edges_");
  for (const auto& marker : interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
  }

  // TODO(nathan) handle deleting empty previous markers

  if (!dynamic_markers.markers.empty()) {
    dynamic_layers_viz_pub_.publish(dynamic_markers);
  }

  // TODO(nathan) move to scene graph probably
  for (const auto& plugin : plugins_) {
    plugin->draw(header, *scene_graph_);
  }
}

}  // namespace kimera
