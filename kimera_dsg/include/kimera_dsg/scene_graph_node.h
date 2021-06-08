#pragma once
#include "kimera_dsg/iterable_wrapper.h"
#include "kimera_dsg/scene_graph_types.h"

#include <Eigen/Dense>

#include <memory>
#include <ostream>
#include <set>
#include <sstream>
#include <string>

namespace kimera {

struct NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<NodeAttributes>;

  /**
   * @brief Make a default set of attributes
   */
  NodeAttributes();

  /**
   * @brief Set the node position
   */
  explicit NodeAttributes(const Eigen::Vector3d& position);

  virtual ~NodeAttributes() = default;

  //! Position of the node
  Eigen::Vector3d position;

  /**
   * @brief output information about the node attributes
   * @param out output stream
   * @param attrs attributes to output
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const NodeAttributes& attrs);

 protected:
  /**
   * @brief actually output information to the std::ostream
   */
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

class SceneGraphNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Attributes = NodeAttributes;
  using Ptr = std::unique_ptr<SceneGraphNode>;
  friend class SceneGraphLayer;
  friend class SceneGraph;

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
  SceneGraphNode(NodeId id, LayerId layer, NodeAttributes::Ptr&& attrs);

  // TODO(nathan) the iterator wrappers don't work very well with copying
  SceneGraphNode(const SceneGraphNode& other) = delete;
  SceneGraphNode& operator=(const SceneGraphNode& other) = delete;

  virtual ~SceneGraphNode() = default;

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

  template <typename Derived = NodeAttributes>
  Derived& attributes() const {
    static_assert(
        std::is_base_of<NodeAttributes, Derived>::value,
        "attributes can only be downcast to a derived NodeAttributes class");
    return dynamic_cast<Derived&>(*attributes_);
  }

  //! ID of the node
  const NodeId id;
  //! ID of the layer the node belongs to
  const LayerId layer;
  /**
   * @brief print information about the node
   * @param out output stream
   * @param node node to output
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const SceneGraphNode& node);

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;

  inline void setParent_(NodeId parent_id) {
    // TODO(nathan) consider rejecting if parent already set
    has_parent_ = true;
    parent_ = parent_id;
  }

  // TODO(nathan) rethink parent to avoid an invalid id
  inline void clearParent_() { has_parent_ = false; }

  const Attributes::Ptr attributes_;

  bool has_parent_;
  NodeId parent_;
  std::set<NodeId> siblings_;
  std::set<NodeId> children_;

 public:
  /**
   * @brief constant iterable over the node's sibilings
   * See @IterableWrapper for full details.
   */
  IterableWrapper<std::set<NodeId>> siblings;
  /**
   * @brief constant iterable over the node's children
   * See @IterableWrapper for full details.
   */
  IterableWrapper<std::set<NodeId>> children;
};

class NodeSymbol {
 public:
  /**
   * @brief Make a node symbol in a human readable form
   */
  NodeSymbol(char key, NodeId idx);

  /**
   * @brief Make a node id directly from a node ID
   */
  NodeSymbol(NodeId value);

  /**
   * @brief cast the symobl directly to a node ID
   */
  inline operator NodeId() const { return value_.value; }

  /**
   * @brief Get the index of the node in the specific category
   * @returns x where (_, x) were the arguments to create the symbol
   */
  inline NodeId categoryId() const { return value_.symbol.index; }

  /**
   * @brief Get the category of the node
   * @returns c where (c, _) were the arguments to create the symbol
   */
  inline char category() const { return value_.symbol.key; }

  /**
   * @brief pre-increment the index portion of the symbol
   */
  NodeSymbol& operator++() {
    value_.symbol.index++;
    return *this;
  }

  /**
   * @brief post-increment the index portion of the symbol
   */
  NodeSymbol operator++(int) {
    NodeSymbol old = *this;
    value_.symbol.index++;
    return old;
  }

  inline std::string getLabel() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

 private:
  union {
    NodeId value;
    struct __attribute__((packed)) {
      NodeId index : 56;
      char key : 8;
    } symbol;
  } value_;

  friend std::ostream& operator<<(std::ostream& out, const NodeSymbol& symbol);
};

}  // namespace kimera
