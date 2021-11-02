#pragma once
#include "kimera_dsg/attribute_serialization.h"
#include "kimera_dsg/scene_graph_layer.h"

#include <map>
#include <memory>
#include <type_traits>

namespace kimera {

/**
 * @brief A representation of a scene graph
 */
class SceneGraph {
 public:
  //! desired pointer type of the scene graph
  using Ptr = std::shared_ptr<SceneGraph>;
  //! alias to the layer type of the scene graph
  using Layer = SceneGraphLayer;
  //! reference type for a layer in the graph
  using LayerRef = std::reference_wrapper<const Layer>;
  //! container type for the layers of the graph
  using Layers = std::map<LayerId, Layer::Ptr>;
  //! container type for the layer ids
  using LayerIds = std::vector<LayerId>;
  //! alias to layer node type
  using Node = Layer::Node;
  //! alias to layer node reference type
  using NodeRef = Layer::NodeRef;
  //! container used for speeding up the lookup of nodes
  using NodeLayerLookup = std::map<NodeId, LayerId>;
  //! alias to the layer edge type
  using Edge = Layer::Edge;
  //! alias to the layer edge info type
  using EdgeInfo = Layer::EdgeInfo;
  //! alias to the layer edge reference type
  using EdgeRef = Layer::EdgeRef;
  //! alias to the layer edge container type
  using Edges = Layer::Edges;
  //! alias to the container storing node-to-node connections
  using EdgeLookup = Layer::EdgeLookup;

  struct JsonExportConfig {
    std::string edge_key = "edges";
    std::string name_key = "id";
    std::string source_key = "source";
    std::string target_key = "target";
  };

  /**
   * @brief Construct the scene graph (with a default layer factory)
   */
  SceneGraph();

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   */
  explicit SceneGraph(const LayerIds& factory);

  virtual ~SceneGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  virtual void clear();

  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param layer_id layer to add to
   * @param node_id node to create
   * @param attrs node attributes
   * @returns true if the node was added successfully
   */
  bool emplaceNode(LayerId layer_id, NodeId node_id, NodeAttributes::Ptr&& attrs);

  /**
   * @brief add a node to the graph
   *
   * Checks that the layer id matches a current layer, that the node
   * is not null and the node doesn't already exist
   *
   * @param node to add
   * @returns true if the node was added successfully
   */
  virtual bool insertNode(Node::Ptr&& node);

  /**
   * @brief Add an edge to the graph
   *
   * Checks that the edge doesn't already exist and
   * that the source and target already exist. Handles passing
   * the edge to the layer if the edge is a intra-layer edge,
   * and updates parents and children of the respective nodes
   *
   * @param source start node
   * @param target end node
   * @param edge_info optional edge attributes (will use
   *        default edge attributes if not supplied)
   * @returns true if the edge was successfully added
   */
  virtual bool insertEdge(NodeId source,
                          NodeId target,
                          EdgeInfo::Ptr&& edge_info = nullptr);

  /**
   * @brief check if a given layer exists
   * @param layer_id layer to check for
   * @returns true if the given layer exists
   */
  virtual bool hasLayer(LayerId layer_id) const;

  /**
   * @brief check if a given node exists
   * @param node_id node to check for
   * @returns true if the given node exists
   */
  virtual bool hasNode(NodeId node_id) const;

  /**
   * @brief check if a given edge exists
   *
   * This checks either the presence of an
   * edge from source to target or from target
   * to source
   *
   * @param source source id of edge to check for
   * @param target target id of edge to check for
   * @returns true if the given edge exists
   */
  virtual bool hasEdge(NodeId source, NodeId target) const;

  /**
   * @brief Get a layer if the layer exists
   * @returns a potentially valid constant reference to the requested layer
   */
  std::optional<LayerRef> getLayer(LayerId layer_id) const;

  /**
   * @brief Get a particular node in the graph
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns a potentially valid node constant reference
   */
  virtual std::optional<NodeRef> getNode(NodeId node_id) const;

  /**
   * @brief Get a particular edge in the graph
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
  virtual bool removeNode(NodeId node_id);

  /**
   * @brief merge two nodes
   * @param node_from node to remove
   * @param node_to node to merge to
   * @returns true if operation succeeded
   */
  virtual bool mergeNodes(NodeId node_from, NodeId node_to);

  /**
   * @brief remove an edge if it exists
   * @param source source of edge to remove
   * @param target target of edge to remove
   * @returns true if the edge existed prior to removal
   */
  virtual bool removeEdge(NodeId source, NodeId target);

  /**
   * @brief Get the number of layers in the graph
   * @return number of layers in the graph
   */
  virtual size_t numLayers() const;

  /**
   * @brief Get number of nodes in the graph
   * @return number of nodes in the graph
   */
  virtual size_t numNodes() const;

  /**
   * @brief Get whether or not the scene graph is empty
   * @note the scene graph invariants make it so only nodes have to be checked
   * @return true if the scene graph is empty
   */
  inline virtual bool empty() const { return numNodes() == 0; }

  /**
   * @brief Get number of edges in the graph
   * @return number of edges in the graph
   */
  virtual size_t numEdges() const;

  /**
   * @brief Update graph from separate layer
   * @note Will invalidate the layer and edges passed in
   * @param other_layer Layer to update from
   * @param edges Optional edges to add to graph
   * @return Whether the update was successful or not
   */
  bool updateFromLayer(SceneGraphLayer& other_layer,
                       std::unique_ptr<Edges>&& edges = nullptr);

  /**
   * @brief Update graph from another graph
   * @note Will add the nodes and edges not previously added in current graph
   * @param other other graph to update from
   */
  bool mergeGraph(const SceneGraph& other);

  /**
   * @brief Get the position of a node in the layer with bounds checking
   */
  virtual Eigen::Vector3d getPosition(NodeId node) const;

  virtual nlohmann::json toJson(const JsonExportConfig& config) const;

  virtual void fillFromJson(const JsonExportConfig& config,
                            const NodeAttributeFactory& node_attr_factory,
                            const EdgeInfoFactory& edge_info_factory,
                            const nlohmann::json& record);

  // TODO(nathan) add full argument list versions of load and save

  void save(const std::string& filepath, bool force_bson = false) const;

  void load(const std::string& filepath, bool force_bson = false);

 protected:
  //! last edge inserted
  size_t last_edge_idx_;
  //! desired layer hierarchy of the graph
  LayerIds layer_ids_;
  //! map between layer id and allocated layers
  Layers layers_;
  //! inter-layer edges
  Edges inter_layer_edges_;
  //! map between node ids and inter-layer edges indices
  EdgeLookup inter_layer_edges_info_;
  //! map between node id and layer id
  NodeLayerLookup node_layer_lookup_;

 protected:
  bool insertIntralayerEdgeInfo(NodeId source,
                                NodeId target,
                                EdgeInfo::Ptr&& edge_info);

  void initialize_();

  void removeInterLayerEdge(NodeId source, NodeId target);

  void rewireInterLayerEdge_(NodeId source,
                             NodeId target,
                             NodeId new_source,
                             NodeId new_target);

  Node* getParentNode_(NodeId source, NodeId target) const;

  Node* getChildNode_(NodeId source, NodeId target) const;

 public:
  /**
   * @brief constant iterator around the layers
   */
  inline const Layers& layers() const { return layers_; };

  /**
   * @brief constant iterator around the inter-layer edges
   *
   * @note inter-layer edges are edges between nodes in different layers
   */
  inline const Edges& inter_layer_edges() const { return inter_layer_edges_; };
};

/**
 * @brief Return a container of the layer hierarchy from #kimera::KimeraDsgLayers
 */
SceneGraph::LayerIds getDefaultLayerIds();

}  // namespace kimera
