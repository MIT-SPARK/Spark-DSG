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

#include <fstream>
#include <iostream>

#include "spark_dsg/dynamic_scene_graph.h"

namespace spark_dsg {

void writeStatsToCsv(size_t num_active,
                     size_t num_removed,
                     size_t num_merged,
                     size_t total_parents,
                     size_t total_with_children,
                     size_t num_edges,
                     const std::string& csv_file,
                     bool write_header) {
  std::ofstream file;
  if (write_header) {
    file.open(csv_file);
    // file format
    file << "nodes_active,nodes_removed,nodes_merged,nodes_w_parents,nodes_w_"
            "children,edges\n";
  } else {
    file.open(csv_file, std::ofstream::out | std::ofstream::app);
  }
  file << num_active << "," << num_removed << "," << num_merged << "," << total_parents
       << "," << total_with_children << "," << num_edges << "\n";
  file.close();
  return;
}

SceneGraphLogger::SceneGraphLogger() {}

SceneGraphLogger::~SceneGraphLogger() {}

void SceneGraphLogger::logGraph(const DynamicSceneGraph& graph) {
  const auto& name_to_layer = graph.layer_names();
  std::map<LayerId, std::string> layer_names;
  for (const auto& [name, key] : name_to_layer) {
    layer_names.emplace(key.layer, name);
  }

  // What I want to log: for each layer, the number of active nodes, number of
  // merged node, number of deleted nodes
  for (const auto& [layer_id, layer] : graph.layers_) {
    if (layer->numNodes() == 0 && !write_header_) {
      continue;
    }

    size_t num_active_nodes = 0;
    size_t num_removed_nodes = 0;
    size_t num_merged_nodes = 0;
    size_t num_nodes_with_parents = 0;
    size_t num_nodes_with_children = 0;
    for (const auto& [node_id, node_status] : layer->nodes_status_) {
      switch (node_status) {
        case NodeStatus::NEW:
        case NodeStatus::VISIBLE:
          num_active_nodes++;
          if (graph.getNode(node_id).hasParent()) {
            num_nodes_with_parents++;
          }
          if (graph.getNode(node_id).hasChildren()) {
            num_nodes_with_children++;
          }
          break;
        case NodeStatus::DELETED:
          num_removed_nodes++;
          break;
        case NodeStatus::MERGED:
          num_merged_nodes++;
          break;
        case NodeStatus::NONEXISTENT:
        default:
          break;
      }
    }

    size_t num_edges = layer->numEdges();

    auto iter = layer_names.find(layer_id);
    std::string name = iter == layer_names.end() ? ("layer_" + std::to_string(layer_id))
                                                 : iter->second;
    // Write to file
    std::string csv_filename = output_dir_ + "/" + name + "_layer.csv";
    writeStatsToCsv(num_active_nodes,
                    num_removed_nodes,
                    num_merged_nodes,
                    num_nodes_with_parents,
                    num_nodes_with_children,
                    num_edges,
                    csv_filename,
                    write_header_);
  }

  write_header_ = false;
  return;
}

}  // namespace spark_dsg
