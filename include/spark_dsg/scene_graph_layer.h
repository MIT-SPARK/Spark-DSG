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
#include <map>
#include <unordered_set>
#include <vector>

#include "spark_dsg/base_layer.h"
#include "spark_dsg/graph_utilities.h"

namespace spark_dsg {

/**
 * @brief A layer in the scene graph (which is a graph itself)
 *
 * This class handles book-keeping for adding to and removing nodes from a layer
 * as well as adding or removing edges between nodes in a layer (i.e. siblings).
 * It is technically safe to add edges directly in the layer class
 * but it is probably preferable to use the scene graph as much as
 * possible to also handle parent child relationships.
 */
class SceneGraphLayer : public BaseLayer {
 public:
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<SceneGraphLayer>;
  //! node container for the layer
  using Nodes = std::map<NodeId, SceneGraphNode::Ptr>;
  //! type tracking the status of nodes
  using NodeCheckup = std::map<NodeId, NodeStatus>;
  //! edge container type for the layer
  using Edges = EdgeContainer::Edges;
  //! callback function for filtering nodes
  using NodeChecker = std::function<bool(const SceneGraphNode&)>;

  friend class DynamicSceneGraph;
  friend class SceneGraphLogger;

  /**
   * @brief Makes an empty layer with the specified layer id
   * @param layer_id layer id of the layer to be constructed
   */
  explicit SceneGraphLayer(LayerId layer_id);

  virtual ~SceneGraphLayer() = default;

  /**
   * @brief Add an edge to the layer
   *
   * Checks that the edge doesn't already exist and
   * that the source and target already exist
   *
   * @param source start node
   * @param target end node
   * @param edge_info optional edge attributes
   * @returns true if the edge was successfully added
   */
  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeAttributes::Ptr&& edge_attributes = nullptr) override;

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
  NodeStatus checkNode(NodeId node_id) const override;

  /**
   * @brief Check whether the layer has the specificied edge
   * @param source first node to check for
   * @param target second node to check for
   * @returns true if the requested edge exists in the layer
   */
  bool hasEdge(NodeId source, NodeId target) const override;

  /**
   * @brief Get a particular node in the layer
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns a potentially valid node constant reference
   */
  const SceneGraphNode* findNode(NodeId node_id) const override;

  /**
   * @brief Get a particular edge in the layer
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns a potentially valid edge constant reference
   */
  const SceneGraphEdge* findEdge(NodeId source, NodeId target) const override;

  /**
   * @brief remove an edge if it exists
   * @param source source of edge to remove
   * @param target target of edge to remove
   * @returns true if the edge existed prior to removal
   */
  bool removeEdge(NodeId source, NodeId target) override;

  /**
   * @brief remove an edge if it exists
   * @param source source of edge to rewire
   * @param target target of edge to rewire
   * @param new_source source of edge after rewiring
   * @param new_target target of edge after rewiring
   * @returns true if operation successful
   */
  bool rewireEdge(NodeId source, NodeId target, NodeId new_source, NodeId new_target);

  bool mergeLayer(const SceneGraphLayer& other,
                  const GraphMergeConfig& config,
                  std::map<NodeId, LayerKey>* layer_lookup = nullptr);

  /**
   * @brief Number of nodes in the layer
   */
  inline size_t numNodes() const { return nodes_.size(); }

  /**
   * @brief Number of edges in the layer
   */
  inline size_t numEdges() const { return edges_.size(); }

  /**
   * @brief Get the position of a node in the layer with bounds checking
   */
  Eigen::Vector3d getPosition(NodeId node) const;

  /**
   * @brief Get node ids of newly inserted nodes
   */
  void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) override;

  /**
   * @brief Get node id of deleted nodes
   */
  void getRemovedNodes(std::vector<NodeId>& removed_nodes, bool clear_removed) override;

  /**
   * @brief Get node id of deleted nodes
   */
  void getRemovedNodes(std::vector<NodeId>& removed_nodes) const;

  /**
   * @brief Get the source and target of newly inserted edges
   */
  void getNewEdges(std::vector<EdgeKey>& new_edges, bool clear_new) override;

  /**
   * @brief Get the source and target of deleted edges
   */
  void getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                       bool clear_removed) override;

  /**
   * @brief Get copy of the layer
   */
  virtual SceneGraphLayer::Ptr clone(const NodeChecker& is_valid = {}) const;

  /**
   * @brief Get the immediate neighborhood of a node via BFS
   * @param node Node to get the neighborhood of
   * @param num_hops Number of hops (1 = siblings and neighbors of siblings)
   */
  std::unordered_set<NodeId> getNeighborhood(NodeId node, size_t num_hops = 1) const;

  /**
   * @brief Get the immediate neighborhood of a set of nodes via BFS
   * @param nodes Nodes to get the neighborhood of
   * @param num_hops Number of hops (1 = siblings and neighbors of siblings)
   */
  std::unordered_set<NodeId> getNeighborhood(const std::unordered_set<NodeId>& nodes,
                                             size_t num_hops = 1) const;

  //! ID of the layer
  const LayerId id;

 protected:
  void reset();

  void fillNeighborhoodForNode(NodeId node,
                               size_t num_hops,
                               std::unordered_set<NodeId>& result,
                               std::map<NodeId, size_t>& costs) const;

  inline EdgeContainer& edgeContainer() override { return edges_; }

  /**
   * @brief remove a node if it exists
   * @param node_id node to remove
   * @returns true if the node existed prior to removal
   */
  bool removeNode(NodeId node_id) override;

  /**
   * @brief construct and add a node to the layer
   * @param node_id node to create
   * @param attrs node attributes
   * @returns true if emplace into internal map was successful
   */
  bool emplaceNode(NodeId node_id, NodeAttributes::Ptr&& attrs);

  /**
   * @brief add a node to the layer
   *
   * Checks that the layer id matches the current layer, that the node
   * is not null and the node doesn't already exist
   *
   * @param node to add
   * @returns true if the node was added successfully
   */
  bool insertNode(SceneGraphNode::Ptr&& node);

  /**
   * @brief merge a node into the other if both nodes exist
   * @param node_from node to merge into the other and remove
   * @param node_to target node (will still exist after merge)
   * @returns true if operation successful
   */
  bool mergeNodes(NodeId node_from, NodeId node_to);

  virtual void cloneImpl(SceneGraphLayer& other, const NodeChecker& is_valid) const;

  //! internal node container
  Nodes nodes_;
  //! internal node status tracking
  NodeCheckup nodes_status_;
  //! internal edge container
  EdgeContainer edges_;

 public:
  /**
   * @brief constant node container
   */
  inline const Nodes& nodes() const { return nodes_; };

  /**
   * @brief constant edge container
   */
  inline const Edges& edges() const { return edges_.edges; };
};

/**
 * @brief A standalone layer not intendend for use with a scene graph
 *
 * This version of a layer exposes methods for adding and remove nodes, something that
 * is typically done at the graph level for book-keeping.
 */
class IsolatedSceneGraphLayer : public SceneGraphLayer {
 public:
  using Ptr = std::unique_ptr<IsolatedSceneGraphLayer>;
  using SPtr = std::shared_ptr<IsolatedSceneGraphLayer>;

  /**
   * @brief Makes an empty layer with the specified layer id
   * @param layer_id layer id of the layer to be constructed
   */
  explicit IsolatedSceneGraphLayer(LayerId layer_id) : SceneGraphLayer(layer_id) {}

  virtual ~IsolatedSceneGraphLayer() = default;

  virtual SceneGraphLayer::Ptr clone(const NodeChecker& is_valid = {}) const override;

  using SceneGraphLayer::emplaceNode;

  using SceneGraphLayer::insertNode;

  using SceneGraphLayer::removeNode;

  using SceneGraphLayer::mergeNodes;
};

namespace graph_utilities {

template <>
struct graph_traits<SceneGraphLayer> {
  using visitor = const std::function<void(const SceneGraphLayer&, NodeId)>&;
  using node_valid_func = const std::function<bool(const SceneGraphNode&)>&;
  using edge_valid_func = const std::function<bool(const SceneGraphEdge&)>&;

  static inline std::set<NodeId> neighbors(const SceneGraphLayer& graph, NodeId node) {
    return get_node(graph, node).siblings();
  }

  static inline bool contains(const SceneGraphLayer& graph, NodeId node) {
    return graph.hasNode(node);
  }

  static inline const SceneGraphLayer::Nodes& nodes(const SceneGraphLayer& graph) {
    return graph.nodes();
  }

  static inline const SceneGraphNode& unwrap_node(
      const SceneGraphLayer::Nodes::value_type& container) {
    return *container.second;
  }

  static inline NodeId unwrap_node_id(
      const SceneGraphLayer::Nodes::value_type& container) {
    return container.first;
  }

  static inline const SceneGraphNode& get_node(const SceneGraphLayer& graph,
                                               NodeId node_id) {
    return graph.getNode(node_id);
  }

  static inline const SceneGraphEdge& get_edge(const SceneGraphLayer& graph,
                                               NodeId source,
                                               NodeId target) {
    return graph.getEdge(source, target);
  }
};

}  // namespace graph_utilities

}  // namespace spark_dsg
