#include "kimera_dsg/scene_graph_logger.h"

#include <fstream>
#include <iostream>

namespace kimera {

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
  file << num_active << "," << num_removed << "," << num_merged << ","
       << total_parents << "," << total_with_children << "," << num_edges
       << "\n";
  file.close();
  return;
}

SceneGraphLogger::SceneGraphLogger() {}

SceneGraphLogger::~SceneGraphLogger() {}

void SceneGraphLogger::logGraph(const SceneGraph::Ptr& graph) {
  // What I want to log: for each layer, the number of active nodes, number of
  // merged node, number of deleted nodes
  for (const auto& id_layer : graph->layers_) {
    if (layer_names_.count(id_layer.first) > 0) {
      if (id_layer.second->numNodes() == 0 && !write_header_) {
        continue;
      }
      size_t num_active_nodes = 0;
      size_t num_removed_nodes = 0;
      size_t num_merged_nodes = 0;
      size_t num_nodes_with_parents = 0;
      size_t num_nodes_with_children = 0;
      for (const auto& id_node_status : id_layer.second->nodes_status_) {
        switch (id_node_status.second) {
          case NodeStatus::VISIBLE:
            num_active_nodes++;
            if (graph->getNode(id_node_status.first)
                    .value()
                    .get()
                    .hasParent()) {
              num_nodes_with_parents++;
            }
            if (graph->getNode(id_node_status.first)
                    .value()
                    .get()
                    .hasChildren()) {
              num_nodes_with_children++;
            }
            break;
          case NodeStatus::DELETED:
            num_removed_nodes++;
            break;
          case NodeStatus::MERGED:
            num_merged_nodes++;
            break;
        }
      }
      size_t num_edges = id_layer.second->numEdges();

      // Write to file
      std::string csv_filename =
          output_dir_ + "/" + layer_names_.at(id_layer.first) + "_layer.csv";
      writeStatsToCsv(num_active_nodes,
                      num_removed_nodes,
                      num_merged_nodes,
                      num_nodes_with_parents,
                      num_nodes_with_children,
                      num_edges,
                      csv_filename,
                      write_header_);
    }
  }
  write_header_ = false;
  return;
}

}  // namespace kimera
