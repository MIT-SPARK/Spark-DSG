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

/**
 * @brief Base node attributes.
 *
 * All nodes have a pointer to node attributes (that contain most of the useful
 * information about the node). As every node has to be spatially consistent
 * with the quantity it represents, every node must have a position.
 */
struct NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired node pointer type
  using Ptr = std::unique_ptr<NodeAttributes>;

  //! Make a default set of attributes
  NodeAttributes();

  //! Set the node position
  explicit NodeAttributes(const Eigen::Vector3d& position);

  virtual ~NodeAttributes() = default;

  //! Position of the node
  Eigen::Vector3d position;

  /**
   * @brief output attribute information
   * @param out output stream
   * @param attrs attributes to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const NodeAttributes& attrs);

 protected:
  //! actually output information to the std::ostream
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

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
  //! desired pointer type of the node (unique)
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

  /**
   * @brief get a reference to the attributes of the node (with an optional
   * template argument to perform a cast to the desired attribute type
   */
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
   * @brief output node information
   * @param out output stream
   * @param node node to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out,
                                  const SceneGraphNode& node);

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
  inline void setParent_(NodeId parent_id) {
    has_parent_ = true;
    parent_ = parent_id;
  }

  /**
   * @brief remove a parent from a node
   * @note only for internal (scene-graph) use
   */
  inline void clearParent_() { has_parent_ = false; }

  //! pointer to attributes
  const Attributes::Ptr attributes_;

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
   * See #kimera::IterableWrapper for full details.
   */
  IterableWrapper<std::set<NodeId>> siblings;
  /**
   * @brief constant iterable over the node's children
   * See #kimera::IterableWrapper for full details.
   */
  IterableWrapper<std::set<NodeId>> children;
};

/**
 * @brief A more human readable way to specify node ids
 *
 * Modeled after gtsam::Symbol, this class uses a character and a index to
 * create a unique key for the node. This is accomplished by setting the first 8
 * MSB to the character and the other 56 LSB to the index (through a union
 * between a NodeId and a bitfield). For the most part, this class is
 * transparent and supports implicit casting to NodeId (i.e. you can use this
 * anywhere you need a NodeId).
 */
class NodeSymbol {
 public:
  //! Make a node symbol in a human readable form
  NodeSymbol(char key, NodeId idx);

  //! Make a node id directly from a node ID
  NodeSymbol(NodeId value);

  //! cast the symobl directly to a node ID
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

  //! pre-increment the index portion of the symbol
  NodeSymbol& operator++() {
    value_.symbol.index++;
    return *this;
  }

  //! post-increment the index portion of the symbol
  NodeSymbol operator++(int) {
    NodeSymbol old = *this;
    value_.symbol.index++;
    return old;
  }

  //! get a string representation of the symbol
  inline std::string getLabel() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  /**
   * @brief output node symbol information
   * @param out output stream
   * @param symbol symbol to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const NodeSymbol& symbol);

 private:
  union {
    NodeId value;
    struct __attribute__((packed)) {
      NodeId index : 56;
      char key : 8;
    } symbol;
  } value_;
};

}  // namespace kimera
