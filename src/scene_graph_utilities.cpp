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
#include "spark_dsg/scene_graph_utilities.h"

#include "spark_dsg/bounding_box_extraction.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/scene_graph.h"

namespace spark_dsg {

using Callback = std::function<void(const SceneGraph&, const NodeId)>;

void getNodeAncestorsAtDepth(const SceneGraph& graph,
                             NodeId parent,
                             size_t depth,
                             const Callback& callback) {
  if (!depth) {
    return;
  }

  const auto node = graph.findNode(parent);
  if (!node) {
    return;
  }

  for (const auto& child : node->children()) {
    // technically we can't recurse with depth=0 and hit this point
    if (depth == 1) {
      callback(graph, child);
    } else {
      getNodeAncestorsAtDepth(graph, child, depth - 1, callback);
    }
  }
}

struct NodeAdaptor : public bounding_box::PointAdaptor {
  explicit NodeAdaptor(const SceneGraph* graph) : graph(graph) {}

  ~NodeAdaptor() = default;

  size_t size() const override { return nodes.size(); }

  Eigen::Vector3f get(size_t index) const override {
    if (!graph) {
      throw std::runtime_error("invalid graph!");
    }

    return graph->getPosition(nodes.at(index)).cast<float>();
  }

  void add(NodeId node) { nodes.push_back(node); }

  const SceneGraph* graph = nullptr;
  std::vector<NodeId> nodes;
};

BoundingBox computeAncestorBoundingBox(const SceneGraph& graph,
                                       NodeId parent,
                                       size_t depth,
                                       BoundingBox::Type bbox_type) {
  NodeAdaptor adaptor(&graph);
  getNodeAncestorsAtDepth(
      graph, parent, depth, [&adaptor](const SceneGraph&, const NodeId ancestor) {
        adaptor.add(ancestor);
      });

  return bounding_box::extract(adaptor, bbox_type);
}

namespace {

std::string* imageFolder(NodeAttributes& attrs) {
  if (auto agent = dynamic_cast<AgentNodeAttributes*>(&attrs)) {
    return &agent->image_folder;
  }
  if (auto subframe = dynamic_cast<SubKeyframeNodeAttributes*>(&attrs)) {
    return &subframe->image_folder;
  }
  if (auto object = dynamic_cast<KhronosObjectAttributes*>(&attrs)) {
    return &object->image_folder;
  }
  return nullptr;
}

}  // namespace

size_t updateImageFolders(
    SceneGraph& graph, const std::function<std::string(const std::string&)>& update) {
  size_t num_changed = 0;
  for (const auto& layer : graph.all_layers()) {
    for (const auto& node : layer.nodes()) {
      auto folder = imageFolder(node.attributes());
      if (!folder || folder->empty()) {
        continue;
      }

      auto updated = update(*folder);
      if (updated != *folder) {
        *folder = std::move(updated);
        ++num_changed;
      }
    }
  }

  return num_changed;
}

size_t remapImageFolders(SceneGraph& graph,
                         const std::filesystem::path& old_prefix,
                         const std::filesystem::path& new_prefix) {
  if (old_prefix.empty()) {
    return 0;
  }

  const auto old_root = old_prefix.lexically_normal();
  const auto new_root = new_prefix.lexically_normal();
  return updateImageFolders(graph, [&](const std::string& folder) {
    const auto relative = std::filesystem::path(folder).lexically_relative(old_root);
    if (relative.empty() || *relative.begin() == "..") {
      return folder;  // not under old_prefix
    }

    return (new_root / relative).lexically_normal().string();
  });
}

size_t resolveImageFolders(SceneGraph& graph, const std::filesystem::path& root) {
  return updateImageFolders(graph, [&](const std::string& folder) {
    const std::filesystem::path path(folder);
    return path.is_absolute() ? folder : (root / path).lexically_normal().string();
  });
}

}  // namespace spark_dsg
