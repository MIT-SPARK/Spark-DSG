#pragma once

#include "kimera_dsg/scene_graph.h"
#include "kimera_dsg/scene_graph_layer.h"

#include <string>

namespace kimera {

/**
 * @brief Logging statistics for the scene graph
 */
class SceneGraphLogger {
 public:
  SceneGraphLogger();
  ~SceneGraphLogger();

  inline void setOutputPath(const std::string& folder) { output_dir_ = folder; }

  inline void setLayerName(const LayerId& id, const std::string& name) {
    layer_names_.insert({id, name});
  }

  void logGraph(const SceneGraph::Ptr& graph);

 private:
  std::string output_dir_;
  std::map<LayerId, std::string> layer_names_;
  bool write_header_ = true;
};

}  // namespace kimera
