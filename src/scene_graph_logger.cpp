/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "spark_dsg/scene_graph_logger.h"

#include <filesystem>
#include <fstream>
#include <map>

#include "spark_dsg/dynamic_scene_graph.h"

namespace spark_dsg {

SceneGraphLogger::SceneGraphLogger() {}

SceneGraphLogger::~SceneGraphLogger() {}

void SceneGraphLogger::logGraph(const DynamicSceneGraph& graph) {
  // What I want to log: for each layer, the number of active nodes, number of
  // merged node, number of deleted nodes
  for (const auto& [layer_id, layer] : graph.layers_) {
    auto iter = layer_entries_.find(layer_id);
    if (iter == layer_entries_.end()) {
      iter = layer_entries_.emplace(layer_id, std::vector<Entry>()).first;
    }

    auto& entry = iter->second.emplace_back();
    entry.num_edges = layer->numEdges();
    for (const auto& [node_id, node_status] : layer->nodes_status_) {
      switch (node_status) {
        case NodeStatus::NEW:
        case NodeStatus::VISIBLE:
          entry.num_active_nodes++;
          if (graph.getNode(node_id).hasParent()) {
            entry.num_nodes_with_parents++;
          }
          if (graph.getNode(node_id).hasChildren()) {
            entry.num_nodes_with_children++;
          }
          break;
        case NodeStatus::DELETED:
          entry.num_removed_nodes++;
          break;
        case NodeStatus::MERGED:
          entry.num_merged_nodes++;
          break;
        case NodeStatus::NONEXISTENT:
        default:
          break;
      }
    }
  }
}

void SceneGraphLogger::save(const std::string& folder) {
  const std::filesystem::path output_path(folder);
  for (const auto& [layer_id, entries] : layer_entries_) {
    std::string name = "layer_" + std::to_string(layer_id);
    const auto csv_path = output_path / (name + "_statistics.csv");

    std::ofstream file(csv_path, std::ofstream::out);
    file << "nodes_active,nodes_removed,nodes_merged,nodes_w_parents,nodes_w_children,"
            "edges\n";
    for (const auto& entry : entries) {
      file << entry.num_active_nodes << "," << entry.num_removed_nodes << ","
           << entry.num_merged_nodes << "," << entry.num_nodes_with_parents << ","
           << entry.num_nodes_with_children << "," << entry.num_edges << "\n";
    }
  }
}

}  // namespace spark_dsg
