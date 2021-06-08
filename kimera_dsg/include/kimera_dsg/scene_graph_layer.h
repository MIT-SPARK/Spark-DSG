#pragma once
#include "kimera_dsg/iterable_wrapper.h"
#include "kimera_dsg/scene_graph_node.h"

#include <map>
#include <optional>
#include <vector>

namespace kimera {

/**
 * @brief Collection of information for an edge
 * @note most of the fields are unused but will hopefully make using
 *       the DSG classes more flexible later
 */
struct SceneGraphEdgeInfo {
  using Ptr = std::unique_ptr<SceneGraphEdgeInfo>;
  bool directed = false;
  bool weighted = false;
  double weight = 1.0;
};

/**
 * @brief Edge representation
 * @note not designed to be constructable by the user (internal book-keeping
 * done by layer and graph)
 */
struct SceneGraphEdge {
  using Info = SceneGraphEdgeInfo;

  SceneGraphEdge(NodeId source, NodeId target, Info::Ptr&& info)
      : source(source), target(target), info(std::move(info)) {}

  NodeId source;
  NodeId target;
  Info::Ptr info;
};

// TODO(nathan) think about inheritance
/**
 * @brief A layer in the scene graph (which is a graph itself)
 *
 * This class handles book-keeping for adding to and removing nodes from a layer
 * as well as adding or removing edges between nodes in a layer (i.e. siblings).
 * It is technically safe to add edges directly in the layer class
 * but it is probably preferable to use the scene graph as much as
 * possible to also handle parent child relationships.
 */
class SceneGraphLayer {
 public:
  using Ptr = std::unique_ptr<SceneGraphLayer>;
  using Node = SceneGraphNode;
  using Nodes = std::map<NodeId, Node::Ptr>;
  using NodeRef = std::reference_wrapper<const Node>;
  using Edge = SceneGraphEdge;
  // map allows for easier removal than vector
  using Edges = std::map<size_t, Edge>;
  using EdgeInfo = SceneGraphEdge::Info;
  using EdgeRef = std::reference_wrapper<const Edge>;
  using EdgeLookup = std::map<NodeId, std::map<NodeId, size_t>>;
  friend class SceneGraph;

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
   * @param edge_info optional edge attributes (will use
   *        default edge attributes if not supplied)
   * @returns true if the edge was successfully added
   */
  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeInfo::Ptr&& edge_info = nullptr);

  /**
   * @brief Check whether the layer has the specified node
   * @param node_id node to check for
   * @returns true if the requested node exists in the layer
   */
  bool hasNode(NodeId node_id) const;

  /**
   * @brief Check whether the layer has the specificied edge
   * @param source first node to check for
   * @param target second node to check for
   * @returns true if the requested edge exists in the layer
   */
  bool hasEdge(NodeId source, NodeId target) const;

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
  std::optional<NodeRef> getNode(NodeId node_id) const;

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
  std::optional<EdgeRef> getEdge(NodeId source, NodeId target) const;

  /**
   * @brief remove a node if it exists
   * @param node_id node to remove
   * @returns true if the node existed prior to removal
   */
  bool removeNode(NodeId node_id);

  /**
   * @brief remove an edge if it exists
   * @param source source of edge to remove
   * @param target target of edge to remove
   * @returns true if the edge existed prior to removal
   */
  bool removeEdge(NodeId source, NodeId target);

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

  //! ID of the layer
  const LayerId id;

 protected:
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
  bool insertNode(Node::Ptr&& node);

  size_t last_edge_idx_;
  Nodes nodes_;
  Edges edges_;
  EdgeLookup edges_info_;

 public:
  // exposure of iterable members for private containers
  // placing them after the private container members ensures
  // that the containers are initialized before the iterable
  // wrappers
  /**
   * @brief constant iterable over the nodes
   * See @IterableWrapper for full details.
   */
  IterableWrapper<Nodes> nodes;
  /**
   * @brief constant iterable over the edges
   * See #IterableWrapper for full details.
   */
  IterableWrapper<Edges> edges;
};

}  // namespace kimera
