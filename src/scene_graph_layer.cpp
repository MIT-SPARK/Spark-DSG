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
#include "spark_dsg/scene_graph_layer.h"

#include "spark_dsg/printing.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;

NodeId GraphMergeConfig::getMergedId(NodeId original) const {
  if (!previous_merges) {
    return original;
  }

  auto iter = previous_merges->find(original);
  return iter == previous_merges->end() ? original : iter->second;
}

SceneGraphLayer::SceneGraphLayer(LayerKey layer_id)
    : SceneGraphLayer(layer_id, std::make_shared<GraphImpl>()) {}

SceneGraphLayer::SceneGraphLayer(const std::string& name)
    : id(DsgLayers::nameToLayerId(name).value()) {}

SceneGraphLayer::SceneGraphLayer(LayerKey layer_id, GraphImpl::Ptr impl)
    : id(layer_id), impl_(impl) {}

bool SceneGraphLayer::hasNode(NodeId node_id) const { return impl_->has(node_id); }

NodeStatus SceneGraphLayer::checkNode(NodeId node) const { return impl_->status(node); }

const Node* SceneGraphLayer::findNode(NodeId node) const { return impl_->find(node); }

const Node& SceneGraphLayer::getNode(NodeId node) const { return impl_->get(node); }

bool SceneGraphLayer::emplaceNode(NodeId node,
                                  std::unique_ptr<NodeAttributes>&& attrs) {
  return impl_->emplace(id, node, std::move(attrs));
}

bool SceneGraphLayer::removeNode(NodeId node) { return impl_->remove(node); }

bool SceneGraphLayer::mergeNodes(NodeId node_from, NodeId node_to) {
  return impl_->contract(node_from, node_to);
}

bool SceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return impl_->has(source, target);
}

const Edge* SceneGraphLayer::findEdge(NodeId source, NodeId target) const {
  return impl_->find(source, target);
}

const SceneGraphEdge& SceneGraphLayer::getEdge(NodeId source, NodeId target) const {
  return impl_->get(source, target);
}

bool SceneGraphLayer::insertEdge(NodeId source,
                                 NodeId target,
                                 std::unique_ptr<EdgeAttributes>&& attrs) {
  return impl_->connect(source, target, std::move(attrs));
}

bool SceneGraphLayer::removeEdge(NodeId source, NodeId target) {
  return impl_->remove(source, target);
}

void SceneGraphLayer::mergeLayer(const SceneGraphLayer& other_layer,
                                 const GraphMergeConfig& config,
                                 std::vector<NodeId>*,
                                 const Eigen::Isometry3d* transform_new_nodes) {
  impl_->merge(*other_layer.impl_, config, transform_new_nodes);
}

void SceneGraphLayer::getNewNodes(std::vector<NodeId>& new_nodes,
                                  bool clear_new) const {}

void SceneGraphLayer::getRemovedNodes(std::vector<NodeId>& removed_nodes,
                                      bool clear_removed) const {}

void SceneGraphLayer::getNewEdges(std::vector<EdgeKey>& new_edges,
                                  bool clear_new) const {}

void SceneGraphLayer::getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                                      bool clear_removed) const {}

void SceneGraphLayer::reset() { impl_->clear(); }

SceneGraphLayer::Ptr SceneGraphLayer::clone() const {
  return SceneGraphLayer::Ptr(new SceneGraphLayer(id, impl_->clone()));
}

void SceneGraphLayer::transform(const Eigen::Isometry3d& transform) {
  impl_->transform(transform);
}

size_t SceneGraphLayer::numNodes() const { return impl_->num_nodes(); }

size_t SceneGraphLayer::numEdges() const { return impl_->num_edges(); }

size_t SceneGraphLayer::memoryUsage() const { return impl_->memory_usage(); }

}  // namespace spark_dsg
