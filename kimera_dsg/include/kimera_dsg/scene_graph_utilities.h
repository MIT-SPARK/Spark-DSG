#pragma once
#include "kimera_dsg/bounding_box.h"
#include "kimera_dsg/dynamic_scene_graph.h"

namespace kimera {

using SgNodeCallback = std::function<void(const DynamicSceneGraph&, const NodeId)>;

void getAncestorsOfLayer(const DynamicSceneGraph& graph,
                         NodeId parent,
                         LayerKey child_layer,
                         const SgNodeCallback& callback);

BoundingBox computeAncestorBoundingBox(const DynamicSceneGraph& graph,
                                       NodeId parent,
                                       LayerId child_layer = KimeraDsgLayers::PLACES);

}  // namespace kimera
