#pragma once
#include "kimera_dsg_visualizer/visualizer_types.h"

#include <kimera_dsg/dynamic_scene_graph.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace kimera {

using ColorFunction = std::function<NodeColor(const SceneGraphNode&)>;

visualization_msgs::Marker makeDeleteMarker(const std_msgs::Header& header,
                                            size_t id,
                                            const std::string& ns);

visualization_msgs::Marker makeBoundingBoxMarker(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphNode& node,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeTextMarker(const std_msgs::Header& header,
                                          const LayerConfig& config,
                                          const SceneGraphNode& node,
                                          const VisualizerConfig& visualizer_config,
                                          const std::string& ns);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const ColormapConfig& colors);

visualization_msgs::Marker makeCentroidMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const std::string& ns,
    const ColorFunction& color_func);

visualization_msgs::MarkerArray makeGraphEdgeMarkers(
    const std_msgs::Header& header,
    const SceneGraph& scene_graph,
    const std::map<LayerId, LayerConfig>& configs,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

visualization_msgs::Marker makeMeshEdgesMarker(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const VisualizerConfig& visualizer_config,
    const DynamicSceneGraph& graph,
    const SceneGraphLayer& layer,
    const std::string& ns);

visualization_msgs::Marker makeLayerEdgeMarkers(
    const std_msgs::Header& header,
    const LayerConfig& config,
    const SceneGraphLayer& layer,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeDynamicCentroidMarkers(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const LayerConfig& layer_config,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeDynamicEdgeMarkers(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const LayerConfig& layer_config,
    const VisualizerConfig& visualizer_config,
    const NodeColor& color,
    const std::string& ns);

visualization_msgs::Marker makeDynamicLabelMarker(
    const std_msgs::Header& header,
    const DynamicLayerConfig& config,
    const DynamicSceneGraphLayer& layer,
    const LayerConfig& layer_config,
    const VisualizerConfig& visualizer_config,
    const std::string& ns);

}  // namespace kimera
