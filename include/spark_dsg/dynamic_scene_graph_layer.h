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
#include "spark_dsg/base_layer.h"
#include "spark_dsg/node_symbol.h"

namespace spark_dsg {

class DynamicSceneGraphLayer : public BaseLayer {
 public:
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<DynamicSceneGraphLayer>;
  //! node container for the layer
  using Nodes = std::vector<SceneGraphNode::Ptr>;
  //! edge container type for the layer
  using Edges = EdgeContainer::Edges;

  friend class DynamicSceneGraph;

  DynamicSceneGraphLayer(LayerId layer, LayerPrefix node_prefix);

  virtual ~DynamicSceneGraphLayer() = default;

  inline size_t numNodes() const { return times_.size(); }

  inline size_t numEdges() const { return edges_.size(); }

  bool hasNode(NodeId node_id) const;

  NodeStatus checkNode(NodeId node_id) const override;

  bool hasNodeByIndex(size_t node_index) const;

  bool hasEdge(NodeId source, NodeId target) const override;

  bool hasEdgeByIndex(size_t source_index, size_t target_index) const;

  const SceneGraphNode* findNode(NodeId node_id) const override;

  const SceneGraphNode* findNodeByIndex(size_t index) const;

  const SceneGraphNode& getNodeByIndex(size_t node_id) const;

  const SceneGraphEdge* findEdge(NodeId source, NodeId target) const override;

  const SceneGraphEdge* findEdgeByIndex(size_t source, size_t target) const;

  const SceneGraphEdge& getEdgeByIndex(size_t source_index, size_t target_index) const;

  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeAttributes::Ptr&& edge_info = nullptr) override;

  bool insertEdgeByIndex(size_t source_index,
                         size_t target_index,
                         EdgeAttributes::Ptr&& edge_info = nullptr);

  bool removeEdge(NodeId source, NodeId target) override;

  bool removeEdgeByIndex(size_t source_index, size_t target_index);

  Eigen::Vector3d getPosition(NodeId node) const;

  Eigen::Vector3d getPositionByIndex(size_t node_index) const;

  const LayerId id;

  const LayerPrefix prefix;

  bool mergeLayer(const DynamicSceneGraphLayer& other,
                  const GraphMergeConfig& config,
                  std::map<NodeId, LayerKey>* layer_lookup = nullptr);

  void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) override;

  void getNewEdges(std::vector<EdgeKey>& new_edges, bool clear_new) override;

  void getRemovedNodes(std::vector<NodeId>& removed_nodes, bool clear_removed) override;

  void getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                       bool clear_removed) override;

 protected:
  bool emplaceNode(std::chrono::nanoseconds timestamp,
                   NodeAttributes::Ptr&& attrs,
                   bool add_edge = true);

  bool emplaceNodeAtIndex(std::chrono::nanoseconds stamp,
                          size_t index,
                          NodeAttributes::Ptr&& attrs);

  bool removeNode(NodeId node) override;

  inline EdgeContainer& edgeContainer() override { return edges_; }

 protected:
  std::set<std::chrono::nanoseconds::rep> times_;

  Nodes nodes_;
  std::map<size_t, NodeStatus> node_status_;
  size_t next_node_;

  //! internal edge container
  EdgeContainer edges_;

 public:
  /**
   * @brief constant node container
   */
  inline const Nodes& nodes() const { return nodes_; }

  /**
   * @brief constant edge container
   */
  inline const Edges& edges() const { return edges_.edges; };
};

}  // namespace spark_dsg
