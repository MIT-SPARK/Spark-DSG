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
#include <functional>
#include <map>

#include "spark_dsg/scene_graph_node.h"

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
class NodeStorage {
 public:
  //!  @brief base node status
  enum class Status { NEW, VISIBLE, MERGED, DELETED, NONEXISTENT };
  //! Base node type
  using Node = SceneGraphNode;
  //! Node container for the layer
  using Nodes = std::map<NodeId, std::unique_ptr<SceneGraphNode>>;
  //! Type tracking the status of nodes
  using NodeCheckup = std::map<NodeId, Status>;
  //! Callback function for filtering nodes
  using NodeChecker = std::function<bool(const SceneGraphNode&)>;

  /**
   * @brief Check the status of a node
   * @param node_id node to check for
   * @returns status of type NodeStatus
   */
  Status check(NodeId node_id) const;

  //! Get number of nodes
  size_t size() const;

  /**
   * @brief Get a particular node
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns Valid pointer to node if it exists, nullptr otherwise
   */
  const Node* find(NodeId node_id) const;

  /**
   * @brief Get a particular node in the layer
   * @param node_id node to get
   * @returns Const reference to node (throws otherwise)
   */
  const Node& at(NodeId node_id) const;

  /**
   * @brief construct and add a node to the storage
   * @param layer_id node is associated with
   * @param node_id node to create
   * @param attrs node attributes
   * @returns true if emplace into internal map was successful
   */
  bool emplace(LayerKey layer, NodeId node_id, std::unique_ptr<NodeAttributes>&& attrs);

  /**
   * @brief remove a node if it exists
   * @param node_id node to remove
   * @returns true if the node existed prior to removal
   */
  bool remove(NodeId node_id);

  //! Get node ids of newly inserted nodes
  void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) const;

  //! Get node id of deleted nodes
  void getRemovedNodes(std::vector<NodeId>& removed_nodes, bool clear_removed) const;

  //! Get copy of the nodes
  virtual std::shared_ptr<NodeStorage> clone(const NodeChecker& is_valid = {}) const;

  //! Rigidly transform nodes
  void transform(const Eigen::Isometry3d& transform);

  //! Get memory usage of the layer in bytes.
  size_t memoryUsage() const;

 protected:
  void reset();

  //! internal node container
  Nodes nodes_;
  //! internal node status tracking
  mutable NodeCheckup nodes_status_;
};

}  // namespace spark_dsg
