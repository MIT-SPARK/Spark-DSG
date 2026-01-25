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
#include <functional>
#include <unordered_set>

#include "spark_dsg/edge_container.h"
#include "spark_dsg/scene_graph_node.h"

namespace spark_dsg {

/**
 * @brief A layer in the scene graph (which is a graph itself)
 */
class SceneGraphLayer {
 public:
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<SceneGraphLayer>;

  virtual ~SceneGraphLayer() = default;

  /**
   * @brief Check whether the layer has the specified node
   * @param node_id node to check for
   * @returns true if the requested node exists in the layer
   */
  bool hasNode(NodeId node_id) const;

  /**
   * @brief Check the status of a node
   * @param node_id node to check for
   * @returns status of type NodeStatus
   */
  NodeStatus checkNode(NodeId node_id) const;

  /**
   * @brief Get a particular node in the layer
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns Valid pointer to node if it exists, nullptr otherwise
   */
  const SceneGraphNode* findNode(NodeId node_id) const;

  /**
   * @brief Get a particular node in the layer
   * @param node_id node to get
   * @returns Const reference to node (throws otherwise)
   */
  const SceneGraphNode& getNode(NodeId node_id) const;

  /**
   * @brief Check whether the layer has the specificied edge
   * @param source first node to check for
   * @param target second node to check for
   * @returns true if the requested edge exists in the layer
   */
  bool hasEdge(NodeId source, NodeId target) const;

  /**
   * @brief Get a particular edge in the layer
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns Pointer to edge if it exists, nullptr otherwise
   */
  const SceneGraphEdge* findEdge(NodeId source, NodeId target) const;

  /**
   * @brief Get a particular edge in the layer
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns the edge
   */
  const SceneGraphEdge& getEdge(NodeId source, NodeId target) const;

  //! ID of the layer
  const LayerKey id;

  //! @brief constant node container
  // const Nodes& nodes() const;

  //! @brief constant edge container
  const EdgeContainer::Edges& edges();

  //! @brief Number of nodes in the layer
  size_t numNodes() const;

  //! @brief Number of edges in the layer
  size_t numEdges() const;

 private:
  friend class SceneGraph;
  explicit SceneGraphLayer(SceneGraph& graph, LayerKey layer_id);

  SceneGraph& graph_;
  std::unordered_set<NodeId> nodes_;
};

namespace graph_utilities {

template <>
struct graph_traits<SceneGraphLayer> {
  using visitor = const std::function<void(const SceneGraphLayer&, NodeId)>&;
  using node_valid_func = const std::function<bool(const SceneGraphNode&)>&;
  using edge_valid_func = const std::function<bool(const SceneGraphEdge&)>&;

  static std::set<NodeId> neighbors(const SceneGraphLayer& graph, NodeId node);
  static bool contains(const SceneGraphLayer& graph, NodeId node);
  static const SceneGraphLayer::Nodes& nodes(const SceneGraphLayer& graph);
  static const SceneGraphNode& unwrap_node(
      const SceneGraphLayer::Nodes::value_type& container);
  static NodeId unwrap_node_id(const SceneGraphLayer::Nodes::value_type& container);
  static const SceneGraphNode& get_node(const SceneGraphLayer& graph, NodeId node_id);
  static const SceneGraphEdge& get_edge(const SceneGraphLayer& graph,
                                        NodeId source,
                                        NodeId target);
};

}  // namespace graph_utilities
}  // namespace spark_dsg
