#pragma once
#include "kimera_dsg/graph_utilities.h"
#include "kimera_dsg/node_symbol.h"
#include "kimera_dsg/scene_graph_node.h"

#include <map>
#include <optional>
#include <unordered_set>
#include <vector>

#define REGISTER_EDGE_INFO_TYPE(classname, json_store)                    \
  static_assert(std::is_base_of<SceneGraphEdgeInfo, classname>::value,    \
                "invalid registered derived type of SceneGraphEdgeInfo"); \
  json_store[classname::TYPE_KEY] = #classname

namespace kimera {

/**
 * @brief Collection of information for an edge
 * @note most of the fields are unused but will hopefully make using
 *       the DSG classes more flexible later
 */
struct SceneGraphEdgeInfo {
  //! desired pointer type for the edge attributes
  using Ptr = std::unique_ptr<SceneGraphEdgeInfo>;

  static inline constexpr char TYPE_KEY[] = "type";

  SceneGraphEdgeInfo() : weighted(false), weight(1.0) {}

  explicit SceneGraphEdgeInfo(double weight) : weighted(true), weight(weight) {}

  //! whether or not the edge weight is valid
  bool weighted;
  //! the weight of the edge
  double weight;

  virtual nlohmann::json toJson() const;

  virtual void fillFromJson(const nlohmann::json& record);
};

/**
 * @brief Edge representation
 */
struct SceneGraphEdge {
  //! attributes of the edge
  using Info = SceneGraphEdgeInfo;

  //! construct and edge from some info
  SceneGraphEdge(NodeId source, NodeId target, Info::Ptr&& info)
      : source(source), target(target), info(std::move(info)) {}

  //! start of edge (by convention the parent)
  const NodeId source;
  //! end of edge (by convention the child)
  const NodeId target;
  //! attributes about the edge
  Info::Ptr info;
};

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
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<SceneGraphLayer>;
  //! node type of the layer
  using Node = SceneGraphNode;
  //! node container for the layer
  using Nodes = std::map<NodeId, Node::Ptr>;
  //! type tracking the status of nodes
  using NodeCheckup = std::map<NodeId, NodeStatus>;
  //! node reference for the layer
  using NodeRef = std::reference_wrapper<const Node>;
  //! edge type for the layer
  using Edge = SceneGraphEdge;
  //! edge container type for the layer
  using Edges = std::map<size_t, Edge>;
  //! edge attributes type for the layer
  using EdgeInfo = SceneGraphEdge::Info;
  //! edge reference type for the layer
  using EdgeRef = std::reference_wrapper<const Edge>;
  //! type for a mapping between node id and edge index
  using EdgeLookup = std::map<NodeId, std::map<NodeId, size_t>>;
  friend class SceneGraph;
  friend class DynamicSceneGraph;

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
  bool insertEdge(NodeId source, NodeId target, EdgeInfo::Ptr&& edge_info = nullptr);

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
   * @brief remove an edge if it exists
   * @param source source of edge to remove
   * @param target target of edge to remove
   * @returns true if the edge existed prior to removal
   */
  bool removeEdge(NodeId source, NodeId target);

  /**
   * @brief remove an edge if it exists
   * @param source source of edge to rewire
   * @param target target of edge to rewire
   * @param new_source source of edge after rewiring
   * @param new_target target of edge after rewiring
   * @returns true if operation successful
   */
  bool rewireEdge(NodeId source,
                  NodeId target,
                  NodeId new_source,
                  NodeId new_target);

  /**
   * @brief merge a graph layer with another
   * @param other other graph layer
   * @param layer_lookup update node layer lookup if called in scene graph class
   * @returns true if operation successful
   */
  bool mergeLayer(const SceneGraphLayer& other,
                  std::map<NodeId, LayerId>* layer_lookup = nullptr);

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
   * @brief Get node id of deleted nodes
   */
  void getRemovedNodes(std::vector<NodeId>* removed_nodes) const;

  //! ID of the layer
  const LayerId id;

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

  std::string serializeLayer(const std::unordered_set<NodeId>& nodes) const;

  std::unique_ptr<Edges> deserializeLayer(const std::string& info);

 protected:
  void reset();

  void fillNeighborhoodForNode(NodeId node,
                               size_t num_hops,
                               std::unordered_set<NodeId>& result,
                               std::map<NodeId, size_t>& costs) const;

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

  /**
   * @brief remove a node if it exists
   * @param node_id node to remove
   * @returns true if the node existed prior to removal
   */
  bool removeNode(NodeId node_id);

  /**
   * @brief merge a node into the other if both nodes exist
   * @param node_from node to merge into the other and remove
   * @param node_to target node (will still exist after merge)
   * @returns true if operation successful
   */
  bool mergeNodes(NodeId node_from, NodeId node_to);

  //! internal edge index counter
  size_t last_edge_idx_;
  //! internal node container
  Nodes nodes_;
  //! internal node status tracking
  NodeCheckup nodes_status_;
  //! internal edge container
  Edges edges_;
  //! internal mapping between node id and edge index
  EdgeLookup edges_info_;

 public:
  /**
   * @brief constant node container
   */
  inline const Nodes& nodes() const { return nodes_; };
  /**
   * @brief constant edge container
   */
  inline const Edges& edges() const { return edges_; };
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

  /**
   * @brief Makes an empty layer with the specified layer id
   * @param layer_id layer id of the layer to be constructed
   */
  explicit IsolatedSceneGraphLayer(LayerId layer_id) : SceneGraphLayer(layer_id) {}

  virtual ~IsolatedSceneGraphLayer() = default;

  using SceneGraphLayer::emplaceNode;

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
    return graph.getNode(node)->get().siblings();
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
                                               NodeId node) {
    return graph.getNode(node).value();
  }

  static inline const SceneGraphEdge& get_edge(const SceneGraphLayer& graph,
                                               NodeId source,
                                               NodeId target) {
    return graph.getEdge(source, target).value();
  }
};

}  // namespace graph_utilities

}  // namespace kimera
