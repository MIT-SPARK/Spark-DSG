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
#pragma once

#include "spark_dsg/graph_view.h"

namespace spark_dsg {

GraphView::GraphView(GraphImpl::Ptr graph,
                     const NodePredicate& node_filter,
                     const EdgePredicate& edge_filter)
    : base_(graph), node_filter_(node_filter), edge_filter_(edge_filter) {}

void GraphView::clear() {}

GraphImpl::Ptr GraphView::clone() const {}

void GraphView::transform(const Eigen::Isometry3d& transform) {
  for (const auto& [node_id, node] : base_->nodes_) {
    if (node_filter_(*node)) {
      node->attributes_->transform(transform);
    }
  }
}

bool GraphView::has(NodeId node_id) const {
  const auto node = base_->find(node_id);
  if (!node) {
    return false;
  }

  return node_filter_(*node);
}

bool GraphView::has(NodeId source, NodeId target) const {}

const SceneGraphNode* GraphView::find(NodeId node_id) const {}

const SceneGraphEdge* GraphView::find(NodeId source, NodeId target) const {}

const SceneGraphNode& GraphView::get(NodeId node_id) const {}

const SceneGraphEdge& GraphView::get(NodeId source, NodeId target) const {}

bool GraphView::emplace(LayerKey layer,
                        NodeId node_id,
                        std::unique_ptr<NodeAttributes>&& attrs) {}

bool GraphView::update(LayerKey layer,
                       NodeId node_id,
                       std::unique_ptr<NodeAttributes>&& attrs) {}

bool GraphView::set(NodeId node_id, std::unique_ptr<NodeAttributes>&& attrs) {}

bool GraphView::connect(NodeId source,
                        NodeId target,
                        std::unique_ptr<EdgeAttributes>&& attrs,
                        bool enforce_parent_constraints) {}

bool GraphView::contract(NodeId node_from, NodeId node_to) {}

bool GraphView::remove(NodeId node_id) {}

bool GraphView::remove(NodeId source, NodeId target) {}

}  // namespace spark_dsg
