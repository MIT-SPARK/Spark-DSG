#include "kimera_dsg_visualizer/scene_graph_visualizer.h"
#include "kimera_dsg_visualizer/visualizer_utils.h"

#include <tf2_eigen/tf2_eigen.h>

namespace kimera {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using Node = SceneGraph::Node;

namespace {

void clearPrevMarkers(const std_msgs::Header& header,
                      const std::set<NodeId>& curr_nodes,
                      const std::string& ns,
                      std::set<NodeId>& prev_nodes,
                      MarkerArray& msg) {
  for (const auto& node : prev_nodes) {
    if (curr_nodes.count(node)) {
      continue;
    }

    Marker marker = makeDeleteMarker(header, node, ns);
    msg.markers.push_back(marker);
  }

  prev_nodes = curr_nodes;
}

}  // namespace

SceneGraphVisualizer::SceneGraphVisualizer(const ros::NodeHandle& nh,
                                           const SceneGraph::LayerIds& layer_ids)
    : nh_(nh),
      need_redraw_(false),
      world_frame_("world"),
      visualizer_ns_(nh.resolveName("config")),
      visualizer_layer_ns_(nh.resolveName("config/layer")) {
  nh_.param("world_frame", world_frame_, world_frame_);
  nh_.param("visualizer_ns", visualizer_ns_, visualizer_ns_);
  nh_.param("visualizer_layer_ns", visualizer_layer_ns_, visualizer_layer_ns_);

  dsg_pub_ = nh_.advertise<MarkerArray>("dsg_markers", 1, true);

  setupConfigs(layer_ids);

  for (const auto& id : layer_ids) {
    prev_labels_[id] = {};
    curr_labels_[id] = {};
    prev_bboxes_[id] = {};
    curr_bboxes_[id] = {};
  }
}

void SceneGraphVisualizer::start() {
  double visualizer_loop_period = 1.0e-1;
  nh_.param("visualizer_loop_period", visualizer_loop_period, visualizer_loop_period);
  visualizer_loop_timer_ =
      nh_.createWallTimer(ros::WallDuration(visualizer_loop_period),
                          &SceneGraphVisualizer::displayLoop,
                          this);
}

void SceneGraphVisualizer::reset() {
  if (scene_graph_) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = world_frame_;

    MarkerArray msg;
    resetImpl(header, msg);

    if (!msg.markers.empty()) {
      dsg_pub_.publish(msg);
    }
  }

  scene_graph_.reset();
}

bool SceneGraphVisualizer::redraw() {
  if (!scene_graph_) {
    return false;
  }

  need_redraw_ |= hasConfigChanged();

  if (!need_redraw_) {
    return false;
  }
  need_redraw_ = false;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = world_frame_;

  MarkerArray msg;
  redrawImpl(header, msg);

  if (!msg.markers.empty()) {
    dsg_pub_.publish(msg);
  }

  clearConfigChangeFlags();
  return true;
}

void SceneGraphVisualizer::setGraph(const DynamicSceneGraph::Ptr& scene_graph) {
  if (scene_graph == nullptr) {
    ROS_ERROR("Request to visualize invalid scene graph! Ignoring");
    return;
  }

  reset();

  scene_graph_ = scene_graph;
  need_redraw_ = true;
}

void SceneGraphVisualizer::resetImpl(const std_msgs::Header& header, MarkerArray& msg) {
  auto to_delete = published_multimarkers_;
  for (const auto& ns : to_delete) {
    deleteMultiMarker(header, ns, msg);
  }

  for (const auto& id_layer_pair : scene_graph_->layers()) {
    const LayerId layer_id = id_layer_pair.first;
    const std::string label_ns = getLayerLabelNamespace(layer_id);
    const std::string bbox_ns = getLayerBboxNamespace(layer_id);
    clearPrevMarkers(header, {}, label_ns, prev_labels_.at(layer_id), msg);
    clearPrevMarkers(header, {}, bbox_ns, prev_bboxes_.at(layer_id), msg);
  }

  for (auto& label_set : prev_labels_) {
    label_set.second.clear();
  }
  for (auto& label_set : curr_labels_) {
    label_set.second.clear();
  }
  for (auto& bbox_set : prev_bboxes_) {
    bbox_set.second.clear();
  }
  for (auto& bbox_set : curr_bboxes_) {
    bbox_set.second.clear();
  }
}

bool SceneGraphVisualizer::hasConfigChanged() const {
  bool has_changed = false;

  has_changed |= visualizer_config_->hasChange();
  has_changed |= places_colormap_->hasChange();
  for (const auto& id_manager_pair : layer_configs_) {
    has_changed |= id_manager_pair.second->hasChange();
  }

  return has_changed;
}

void SceneGraphVisualizer::clearConfigChangeFlags() {
  visualizer_config_->clearChangeFlag();
  places_colormap_->clearChangeFlag();
  for (auto& id_manager_pair : layer_configs_) {
    id_manager_pair.second->clearChangeFlag();
  }
}

void SceneGraphVisualizer::redrawImpl(const std_msgs::Header& header,
                                      MarkerArray& msg) {
  for (const auto& id_layer_pair : scene_graph_->layers()) {
    LayerConfig config = layer_configs_.at(id_layer_pair.first)->get();
    const SceneGraphLayer& layer = *(id_layer_pair.second);

    if (!config.visualize) {
      deleteLayer(header, layer, msg);
    } else {
      drawLayer(header, layer, config, msg);
    }
  }

  drawLayerMeshEdges(header, mesh_edge_source_layer_, mesh_edge_ns_, msg);

  std::map<LayerId, LayerConfig> all_configs;
  for (const auto& id_manager_pair : layer_configs_) {
    all_configs[id_manager_pair.first] = id_manager_pair.second->get();
  }

  MarkerArray interlayer_edge_markers =
      makeGraphEdgeMarkers(header,
                           *scene_graph_,
                           all_configs,
                           visualizer_config_->get(),
                           interlayer_edge_ns_prefix_);

  std::set<std::string> seen_edge_labels;
  for (const auto& marker : interlayer_edge_markers.markers) {
    addMultiMarkerIfValid(marker, msg);
    seen_edge_labels.insert(marker.ns);
  }

  for (const auto& source_pair : all_configs) {
    for (const auto& target_pair : all_configs) {
      if (source_pair.first == target_pair.first) {
        continue;
      }

      const std::string curr_ns = interlayer_edge_ns_prefix_ +
                                  std::to_string(source_pair.first) + "_" +
                                  std::to_string(target_pair.first);
      if (seen_edge_labels.count(curr_ns)) {
        continue;
      }

      deleteMultiMarker(header, curr_ns, msg);
    }
  }
}

void SceneGraphVisualizer::deleteMultiMarker(const std_msgs::Header& header,
                                             const std::string& ns,
                                             MarkerArray& msg) {
  if (!published_multimarkers_.count(ns)) {
    return;
  }

  Marker delete_marker = makeDeleteMarker(header, 0, ns);
  msg.markers.push_back(delete_marker);

  published_multimarkers_.erase(ns);
}

void SceneGraphVisualizer::addMultiMarkerIfValid(const Marker& marker,
                                                 MarkerArray& msg) {
  if (!marker.points.empty()) {
    msg.markers.push_back(marker);
    published_multimarkers_.insert(marker.ns);
    return;
  }

  deleteMultiMarker(marker.header, marker.ns, msg);
}

void SceneGraphVisualizer::setupConfigs(const SceneGraph::LayerIds& layer_ids) {
  ros::NodeHandle nh("");
  visualizer_config_.reset(new VisualizerConfigManager(
      nh, visualizer_ns_, [](const ros::NodeHandle& nh, auto& config) {
        config = getVisualizerConfig(nh);
      }));

  const std::string colormap_ns = visualizer_ns_ + "/places_colormap";
  places_colormap_.reset(new ColormapConfigManager(
      nh, colormap_ns, [](const ros::NodeHandle& nh, auto& config) {
        config = getColormapConfig(nh);
      }));

  for (const auto& layer : layer_ids) {
    const std::string layer_ns = visualizer_layer_ns_ + std::to_string(layer);
    layer_configs_[layer] = LayerConfigManager::Ptr(new LayerConfigManager(
        nh, layer_ns, [&](const ros::NodeHandle& nh, auto& config) {
          config = getLayerConfig(nh);
        }));
  }
}

void SceneGraphVisualizer::displayLoop(const ros::WallTimerEvent&) { redraw(); }

void SceneGraphVisualizer::deleteLayer(const std_msgs::Header& header,
                                       const SceneGraphLayer& layer,
                                       MarkerArray& msg) {
  deleteMultiMarker(header, getLayerNodeNamespace(layer.id), msg);
  deleteMultiMarker(header, getLayerEdgeNamespace(layer.id), msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);
  for (const auto& node : prev_labels_.at(layer.id)) {
    Marker marker = makeDeleteMarker(header, node, label_ns);
    msg.markers.push_back(marker);
  }
  prev_labels_.at(layer.id).clear();

  const std::string bbox_ns = getLayerBboxNamespace(layer.id);
  for (const auto& node : prev_bboxes_.at(layer.id)) {
    Marker marker = makeDeleteMarker(header, node, bbox_ns);
    msg.markers.push_back(marker);
  }
  prev_bboxes_.at(layer.id).clear();
}

void SceneGraphVisualizer::drawLayer(const std_msgs::Header& header,
                                     const SceneGraphLayer& layer,
                                     const LayerConfig& config,
                                     MarkerArray& msg) {
  const auto& viz_config = visualizer_config_->get();
  const std::string node_ns = getLayerNodeNamespace(layer.id);

  const bool color_by_distance = layer.id == KimeraDsgLayers::PLACES &&
                                 visualizer_config_->get().color_places_by_distance;

  Marker nodes;
  if (color_by_distance) {
    nodes = makeCentroidMarkers(
        header, config, layer, viz_config, node_ns, places_colormap_->get());
  } else if (layer.id == KimeraDsgLayers::PLACES) {
    nodes = makeCentroidMarkers(header,
                                config,
                                layer,
                                viz_config,
                                node_ns,
                                [&](const SceneGraphNode& node) -> NodeColor {
                                  auto parent = node.getParent();
                                  if (!parent) {
                                    return NodeColor::Zero();
                                  }

                                  return scene_graph_->getNode(*parent)
                                      .value()
                                      .get()
                                      .attributes<SemanticNodeAttributes>()
                                      .color;
                                });
  } else {
    nodes = makeCentroidMarkers(header, config, layer, viz_config, node_ns);
  }
  addMultiMarkerIfValid(nodes, msg);

  const std::string edge_ns = getLayerEdgeNamespace(layer.id);
  Marker edges = makeLayerEdgeMarkers(
      header, config, layer, viz_config, NodeColor::Zero(), edge_ns);
  addMultiMarkerIfValid(edges, msg);

  const std::string label_ns = getLayerLabelNamespace(layer.id);
  const std::string bbox_ns = getLayerBboxNamespace(layer.id);

  curr_labels_.at(layer.id).clear();
  curr_bboxes_.at(layer.id).clear();
  for (const auto& id_node_pair : layer.nodes()) {
    const Node& node = *id_node_pair.second;

    if (config.use_label) {
      Marker label = makeTextMarker(header, config, node, viz_config, label_ns);
      msg.markers.push_back(label);
      curr_labels_.at(layer.id).insert(node.id);
    }

    if (config.use_bounding_box) {
      try {
        node.attributes<ObjectNodeAttributes>();
      } catch (const std::bad_cast&) {
        continue;
      }

      Marker bbox = makeBoundingBoxMarker(header, config, node, viz_config, bbox_ns);
      msg.markers.push_back(bbox);
      curr_bboxes_.at(layer.id).insert(node.id);
    }
  }

  clearPrevMarkers(
      header, curr_labels_.at(layer.id), label_ns, prev_labels_.at(layer.id), msg);
  clearPrevMarkers(
      header, curr_bboxes_.at(layer.id), bbox_ns, prev_bboxes_.at(layer.id), msg);
}

void SceneGraphVisualizer::drawLayerMeshEdges(const std_msgs::Header& header,
                                              LayerId layer_id,
                                              const std::string& ns,
                                              MarkerArray& msg) {
  if (!layer_configs_.count(layer_id)) {
    return;
  }

  LayerConfig config = layer_configs_.at(layer_id)->get();

  const auto layer = scene_graph_->getLayer(layer_id);
  if (!layer || !config.visualize) {
    deleteMultiMarker(header, ns, msg);
    return;
  }

  Marker mesh_edges = makeMeshEdgesMarker(
      header, config, visualizer_config_->get(), *scene_graph_, *layer, ns);
  addMultiMarkerIfValid(mesh_edges, msg);
}

}  // namespace kimera
