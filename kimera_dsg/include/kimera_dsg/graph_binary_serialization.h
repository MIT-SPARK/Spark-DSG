#pragma once
#include "kimera_dsg/dynamic_scene_graph.h"

namespace kimera {

void writeGraph(const DynamicSceneGraph& graph, std::vector<uint8_t>& buffer);

DynamicSceneGraph::Ptr readGraph(const std::vector<uint8_t>& buffer);

bool updateGraph(DynamicSceneGraph& graph, const std::vector<uint8_t>& buffer);

}  // namespace kimera
