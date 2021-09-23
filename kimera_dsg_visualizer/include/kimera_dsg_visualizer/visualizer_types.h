#pragma once
#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg_visualizer/ColormapConfig.h>
#include <kimera_dsg_visualizer/DynamicLayerVisualizerConfig.h>
#include <kimera_dsg_visualizer/LayerVisualizerConfig.h>
#include <kimera_dsg_visualizer/VisualizerConfig.h>

namespace kimera {

using LayerConfig = kimera_dsg_visualizer::LayerVisualizerConfig;
using DynamicLayerConfig = kimera_dsg_visualizer::DynamicLayerVisualizerConfig;
using VisualizerConfig = kimera_dsg_visualizer::VisualizerConfig;
using ColormapConfig = kimera_dsg_visualizer::ColormapConfig;
using NodeColor = SemanticNodeAttributes::ColorVector;

inline double getZOffset(const LayerConfig& config,
                         const VisualizerConfig& visualizer_config) {
  if (visualizer_config.collapse_layers) {
    return 0.0;
  }

  return config.z_offset_scale * visualizer_config.layer_z_step;
}

VisualizerConfig getVisualizerConfig(const ros::NodeHandle& nh);

inline VisualizerConfig getVisualizerConfig(const std::string& visualizer_namespace) {
  return getVisualizerConfig(ros::NodeHandle(visualizer_namespace));
}

LayerConfig getLayerConfig(const ros::NodeHandle& nh);

inline LayerConfig getLayerConfig(const std::string& layer_namespace) {
  return getLayerConfig(ros::NodeHandle(layer_namespace));
}

ColormapConfig getColormapConfig(const ros::NodeHandle& nh);

inline ColormapConfig getColormapConfig(const std::string& colormap_namespace) {
  return getColormapConfig(ros::NodeHandle(colormap_namespace));
}

DynamicLayerConfig getDynamicLayerConfig(const ros::NodeHandle& nh);

}  // namespace kimera
