#include "kimera_dsg_visualizer/dynamic_scene_graph_visualizer.h"
#include "kimera_dsg_visualizer/feature_config.h"
#include "kimera_dsg_visualizer/visualizer_utils.h"

#if ENABLE_PGMO_FEATURES()
#include <kimera_pgmo/utils/CommonFunctions.h>
#endif

#include <mesh_msgs/TriangleMesh.h>
#include <mesh_msgs/TriangleMeshStamped.h>

namespace kimera {

namespace {

void adjustMesh(const LayerConfig& config,
                const VisualizerConfig& visualizer_config,
                mesh_msgs::TriangleMesh& msg) {
  for (auto& point : msg.vertices) {
    point.z += getZOffset(config, visualizer_config);
  }
}

}  // namespace

DynamicSceneGraphVisualizer::DynamicSceneGraphVisualizer(
    const ros::NodeHandle& nh,
    const SceneGraph::LayerIds& layer_ids)
    : SceneGraphVisualizer(nh, layer_ids) {
  wall_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("wall_mesh", 1, true);
  semantic_mesh_pub_ =
      nh_.advertise<mesh_msgs::TriangleMeshStamped>("semantic_mesh", 1, true);
  rgb_mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("rgb_mesh", 1, true);
}

#if ENABLE_PGMO_FEATURES()
void DynamicSceneGraphVisualizer::visualizeMesh(const pcl::PolygonMesh& mesh,
                                                bool rgb_mesh) const {
  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();  // TODO(nathan) unify time
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);

  // TODO(nathan) this could be rethought to not conflict with dsg publisher
  if (rgb_mesh) {
    rgb_mesh_pub_.publish(msg);
  } else {
    semantic_mesh_pub_.publish(msg);
  }
}

#else
void DynamicSceneGraphVisualizer::visualizeMesh(const pcl::PolygonMesh&,
                                                bool rgb_mesh) const {
  ROS_WARN(
      "DSG Visualizer not built with PGMO. You should rebuild the "
      "kimera_dsg_visualizer package with kimera_pgmo installed enable mesh "
      "visualization");
}
#endif

#if ENABLE_PGMO_FEATURES()
void DynamicSceneGraphVisualizer::visualizeDsgMesh() const {
  if (!scene_graph_->hasMesh()) {
    ROS_ERROR("Attempting to visualize unitialized mesh");
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();
  // vertices and meshes are guaranteed to not be null (from hasMesh)
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(*scene_graph_->getMeshVertices(),
                                                       *scene_graph_->getMeshFaces());
  semantic_mesh_pub_.publish(msg);
}
#else
void DynamicSceneGraphVisualizer::visualizeDsgMesh() const {
  ROS_WARN(
      "DSG Visualizer not built with PGMO. You should rebuild the "
      "kimera_dsg_visualizer package with kimera_pgmo installed enable mesh "
      "visualization");
}
#endif

#if ENABLE_PGMO_FEATURES()
void DynamicSceneGraphVisualizer::visualizeWalls(const pcl::PolygonMesh& mesh) const {
  if (mesh.polygons.empty()) {
    return;  // better to not publish an empty marker
  }

  if (!layer_configs_.count(to_underlying(KimeraDsgLayers::PLACES))) {
    ROS_WARN("Failed to find config for places layer");
    return;
  }

  mesh_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = world_frame_;
  msg.header.stamp = ros::Time::now();  // TODO(nathan) unify time
  msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);

  LayerConfig config = layer_configs_.at(to_underlying(KimeraDsgLayers::PLACES));
  adjustMesh(config, visualizer_config_, msg.mesh);

  wall_pub_.publish(msg);
}
#else
void DynamicSceneGraphVisualizer::visualizeWalls(const pcl::PolygonMesh&) const {
  ROS_WARN(
      "DSG Visualizer not built with PGMO. You should rebuild the "
      "kimera_dsg_visualizer package with kimera_pgmo installed enable mesh "
      "visualization");
}
#endif

bool DynamicSceneGraphVisualizer::redraw() {
  if (!SceneGraphVisualizer::redraw()) {
    return false;
  }

  for (const auto& plugin : plugins_) {
    plugin->draw(*scene_graph_);
  }

  if (!scene_graph_->hasMesh()) {
    return false;
  }

  visualizeDsgMesh();
  return true;
}

}  // namespace kimera
