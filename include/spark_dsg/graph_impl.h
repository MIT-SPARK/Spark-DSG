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
#include <Eigen/Geometry>
#include <map>

#include "spark_dsg/scene_graph_edge.h"
#include "spark_dsg/scene_graph_node.h"

namespace spark_dsg {

//! Current state of a node
enum class NodeStatus { NEW, PRESENT, MERGED, DELETED, NONEXISTENT };

//! Current state of an edge
enum class EdgeStatus { NEW, PRESENT, DELETED, NONEXISTENT };

//! General graph representation
class GraphImpl {
 public:
  //! desired pointer type for the layer
  using Ptr = std::shared_ptr<GraphImpl>;

  GraphImpl();
  virtual ~GraphImpl();

  size_t num_nodes() const;
  size_t num_edges() const;

  bool has(NodeId node_id) const;
  bool has(NodeId source, NodeId target) const;

  NodeStatus status(NodeId node_id) const;
  EdgeStatus status(NodeId source, NodeId target) const;

  const SceneGraphNode* find(NodeId node_id) const;
  const SceneGraphEdge* find(NodeId source, NodeId target) const;

  const SceneGraphNode& get(NodeId node_id) const;
  const SceneGraphEdge& get(NodeId source, NodeId target) const;

  bool emplace(LayerKey layer, NodeId node_id, std::unique_ptr<NodeAttributes>&& attrs);
  bool connect(NodeId source,
               NodeId target,
               std::unique_ptr<EdgeAttributes>&& edge_attributes = nullptr);

  bool remove(NodeId node_id);
  bool remove(NodeId source, NodeId target);

  bool contract(NodeId node_from, NodeId node_to);

  GraphImpl::Ptr clone() const;

  void transform(const Eigen::Isometry3d& transform);

  size_t memoryUsage() const;

  const std::map<NodeId, std::unique_ptr<SceneGraphNode>>& nodes() const {
    return nodes_;
  }

  const std::map<EdgeKey, SceneGraphEdge>& edges() const { return edges_; }

 protected:
  void reset();

  std::map<NodeId, std::unique_ptr<SceneGraphNode>> nodes_;
  std::map<EdgeKey, SceneGraphEdge> edges_;

  mutable std::map<NodeId, NodeStatus> node_status_;
  mutable std::map<EdgeKey, EdgeStatus> edge_status_;
  mutable std::map<EdgeKey, bool> stale_edges_;
};

}  // namespace spark_dsg
