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
#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/edge_container.h"
#include "spark_dsg/scene_graph_node.h"

namespace spark_dsg {

struct GraphMergeConfig {
  const std::map<NodeId, NodeId>* previous_merges = nullptr;
  const std::map<LayerId, bool>* update_layer_attributes = nullptr;
  bool update_dynamic_attributes = true;
  bool update_archived_attributes = false;
  bool clear_removed = false;
  bool enforce_parent_constraints = true;

  NodeId getMergedId(NodeId original) const {
    if (!previous_merges) {
      return original;
    }

    auto iter = previous_merges->find(original);
    return iter == previous_merges->end() ? original : iter->second;
  }
};

class BaseLayer {
 public:
  friend class DynamicSceneGraph;

  virtual ~BaseLayer() = default;

  virtual bool hasEdge(NodeId source, NodeId target) const = 0;

  virtual bool removeEdge(NodeId source, NodeId target) = 0;

  virtual bool insertEdge(NodeId source, NodeId target, EdgeAttributes::Ptr&& info) = 0;

  virtual NodeStatus checkNode(NodeId node_id) const = 0;

  virtual const SceneGraphNode* findNode(NodeId node) const = 0;

  virtual const SceneGraphEdge* findEdge(NodeId source, NodeId target) const = 0;

  virtual const SceneGraphNode& getNode(NodeId node_id) const {
    const auto node = findNode(node_id);
    if (!node) {
      throw std::out_of_range("missing node '" + NodeSymbol(node_id).getLabel() + "'");
    }

    return *node;
  }

  virtual const SceneGraphEdge& getEdge(NodeId source, NodeId target) const {
    const auto edge = findEdge(source, target);
    if (!edge) {
      std::stringstream ss;
      ss << "Missing edge '" << EdgeKey(source, target) << "'";
      throw std::out_of_range(ss.str());
    }

    return *edge;
  }

  /**
   * @brief Get node ids of newly inserted nodes
   */
  virtual void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) = 0;

  /**
   * @brief Get node id of deleted nodes
   */
  virtual void getRemovedNodes(std::vector<NodeId>& removed_nodes,
                               bool clear_removed) = 0;

  /**
   * @brief Get the source and target of newly inserted edges
   */
  virtual void getNewEdges(std::vector<EdgeKey>& new_edges, bool clear_new) = 0;

  /**
   * @brief Get the source and target of deleted edges
   */
  virtual void getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                               bool clear_removed) = 0;

 protected:
  virtual EdgeContainer& edgeContainer() = 0;

  virtual bool removeNode(NodeId node_id) = 0;
};

}  // namespace spark_dsg
