#pragma once
#include <kimera_dsg_visualizer/LayerVisualizerConfig.h>
#include <kimera_dsg_visualizer/VisualizerConfig.h>
#include <kimera_dsg/node_attributes.h>

namespace kimera {

using LayerConfig = kimera_dsg_visualizer::LayerVisualizerConfig;
using VisualizerConfig = kimera_dsg_visualizer::VisualizerConfig;
using NodeColor = SemanticNodeAttributes::ColorVector;

inline double getZOffset(const LayerConfig& config,
                         const VisualizerConfig& visualizer_config) {
  if (visualizer_config.collapse_layers) {
    return 0.0;
  }

  return config.z_offset_scale * visualizer_config.layer_z_step;
}

VisualizerConfig getVisualizerConfig(const std::string& visualizer_namespace);

LayerConfig getLayerConfig(const std::string& layer_namespace);

} // namespace kimera
