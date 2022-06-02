#include "kimera_dsg/scene_graph_utilities.h"

namespace kimera {

void getAncestorsOfLayer(const DynamicSceneGraph& graph,
                         NodeId parent,
                         LayerKey child_layer,
                         const SgNodeCallback& callback) {
  const auto& node_opt = graph.getNode(parent);
  if (!node_opt) {
    return;
  }

  const SceneGraphNode& node = *node_opt;
  if (node.layer <= child_layer) {
    return;
  }

  for (const auto& child : node.children()) {
    const auto layer_key = *graph.getLayerForNode(child);
    if (layer_key == child_layer) {
      callback(graph, child);
    } else {
      getAncestorsOfLayer(graph, child, child_layer, callback);
    }
  }
}

BoundingBox computeAncestorBoundingBox(const DynamicSceneGraph& graph,
                                       NodeId parent,
                                       LayerId child_layer) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>());

  getAncestorsOfLayer(graph,
                      parent,
                      child_layer,
                      [&points](const DynamicSceneGraph& dsg, const NodeId ancestor) {
                        const Eigen::Vector3d pos = dsg.getPosition(ancestor);
                        points->push_back(pcl::PointXYZ(pos.x(), pos.y(), pos.z()));
                      });

  return BoundingBox::extract(points);
}

}  // namespace kimera
