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
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/scene_graph_types.h"

#include <Eigen/Dense>

#include <chrono>
#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <optional>

namespace spark_dsg {

/**
 * @brief Base node status.
 *
 * Mostly for keeping history and status of nodes in a graph
 */
enum class NodeStatus { NEW, VISIBLE, MERGED, DELETED, NONEXISTENT };

/**
 * @brief Node in the scene graph
 *
 * Nodes are primarily specialized by their attributes, and not by node type.
 * Every node has a constant iterator over the node's siblings (other nodes
 * connected to it in the same layer) and children (nodes connected to it in a
 * lower layer). Nodes can also have a parent (node with conceptual ownership in
 * a higher layer).
 */
class SceneGraphNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! attribute type of the node
  using Attributes = NodeAttributes;
  using AttributesPtr = std::unique_ptr<Attributes>;
  //! desired pointer type of the node (unique)
  using Ptr = std::unique_ptr<SceneGraphNode>;
  friend class DynamicSceneGraphLayer;
  friend class DynamicSceneGraph;
  friend class SceneGraphLayer;

  /**
   * @brief Make a scene graph node (usually not necessary)
   *
   * It's usually not suggested to use this directly (it's typically
   * less book-keeping to just emplace directly into the scene graph)
   *
   * @param id the id of the node to create
   * @param layer the layer that the node will belong to
   * @param attrs attributes for the node
   */
  SceneGraphNode(NodeId id, LayerId layer, AttributesPtr&& attrs);

  SceneGraphNode(const SceneGraphNode& other) = delete;

  SceneGraphNode& operator=(const SceneGraphNode& other) = delete;

  SceneGraphNode(SceneGraphNode&& other) = default;

  SceneGraphNode& operator=(SceneGraphNode&& other) = default;

  virtual ~SceneGraphNode();

  /**
   * @brief get whether a node has a parent
   * @returns whether or not the node has a parent
   */
  inline bool hasParent() const { return has_parent_; }

  /**
   * @brief get whether a node has any siblings
   * @note this is equivalents to siblings.empty() but has more semantically
   * meaning
   * @returns whether or not the node has any siblings
   */
  inline bool hasSiblings() const { return not siblings_.empty(); }

  /**
   * @brief get whether a node has any children
   * @note this is equivalents to children.empty() but has more semantically
   * meaning
   * @returns whether or not the node has any children
   */
  inline bool hasChildren() const { return not children_.empty(); }

  /**
   * @brief get the parent of the node (if it exists)
   * @returns the id of the parent of the node (if it exists)
   */
  inline std::optional<NodeId> getParent() const {
    if (!has_parent_) {
      return std::nullopt;
    }

    return parent_;
  }

  /**
   * @brief get a reference to the attributes of the node (with an optional
   * template argument to perform a cast to the desired attribute type
   */
  template <typename Derived = NodeAttributes>
  Derived& attributes() const {
    static_assert(std::is_base_of<NodeAttributes, Derived>::value,
                  "attributes can only be downcast to a derived NodeAttributes class");
    return dynamic_cast<Derived&>(*attributes_);
  }

  NodeAttributes* getAttributesPtr() const { return attributes_.get(); }

  //! ID of the node
  const NodeId id;
  //! ID of the layer the node belongs to
  const LayerId layer;

  /**
   * @brief output node information
   * @param out output stream
   * @param node node to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const SceneGraphNode& node);

 protected:
  /**
   * @brief internal function for outputing information to a ostream
   * @param out ostream to output info to
   */
  virtual std::ostream& fill_ostream(std::ostream& out) const;

  /**
   * @brief set the parent for a node
   * @note only for internal (scene-graph) use
   * @param parent_id new parent of node
   */
  inline void setParent(NodeId parent_id) {
    has_parent_ = true;
    parent_ = parent_id;
  }

  /**
   * @brief remove a parent from a node
   * @note only for internal (scene-graph) use
   */
  inline void clearParent() { has_parent_ = false; }

 protected:
  //! pointer to attributes
  AttributesPtr attributes_;

  //! whether or not the node has a parent
  bool has_parent_;
  //! node id of parent (if valid)
  NodeId parent_;
  //! sibling node ids (maintained by layer)
  std::set<NodeId> siblings_;
  //! children node ids (maintained by graph)
  std::set<NodeId> children_;

 public:
  /**
   * @brief constant iterable over the node's sibilings
   */
  inline const std::set<NodeId>& siblings() const { return siblings_; };
  /**
   * @brief constant iterable over the node's children
   */
  inline const std::set<NodeId>& children() const { return children_; };
};

class DynamicSceneGraphNode : public SceneGraphNode {
 public:
  friend class DynamicSceneGraphLayer;
  using Ptr = std::unique_ptr<DynamicSceneGraphNode>;

  DynamicSceneGraphNode(NodeId id,
                        LayerId layer,
                        NodeAttributes::Ptr&& attrs,
                        std::chrono::nanoseconds timestamp)
      : SceneGraphNode(id, layer, std::move(attrs)), timestamp(timestamp) {}

  virtual ~DynamicSceneGraphNode() = default;

  DynamicSceneGraphNode(const DynamicSceneGraphNode& other) = delete;

  DynamicSceneGraphNode& operator=(const DynamicSceneGraphNode& other) = delete;

  const std::chrono::nanoseconds timestamp;

 protected:
  /**
   * @brief internal function for outputing information to a ostream
   * @param out ostream to output info to
   */
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

}  // namespace spark_dsg
