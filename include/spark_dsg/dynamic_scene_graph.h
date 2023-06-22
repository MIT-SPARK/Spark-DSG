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
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <type_traits>

#include "spark_dsg/dynamic_scene_graph_layer.h"
#include "spark_dsg/scene_graph_layer.h"

namespace spark_dsg {

/**
 * @brief Structure capturing node-edge relationship
 */
struct MeshEdge {
  NodeId source_node;
  size_t mesh_vertex;

  /**
   * @brief initialize a mesh edge
   * @param source_node Node in the scene graph
   * @param mesh_vertex Mesh vertex that the edge points to
   */
  MeshEdge(NodeId source_node, size_t mesh_vertex)
      : source_node(source_node), mesh_vertex(mesh_vertex) {}
};

/**
 * @brief Dynamic Scene Graph class
 *
 * Contains an explicit mesh layer
 */
class DynamicSceneGraph {
 public:
  using NodeRef = BaseLayer::NodeRef;
  using DynamicNodeRef = BaseLayer::DynamicNodeRef;
  using EdgeRef = BaseLayer::EdgeRef;

  //! Desired pointer type of the scene graph
  using Ptr = std::shared_ptr<DynamicSceneGraph>;
  //! container type for the layer ids
  using LayerIds = std::vector<LayerId>;
  //! Edge container
  using Edges = EdgeContainer::Edges;
  //! Layer container
  using Layers = std::map<LayerId, SceneGraphLayer::Ptr>;
  //! Dynamic layer container
  using DynamicLayers = std::map<uint32_t, DynamicSceneGraphLayer::Ptr>;
  //! Underlying mesh type for lowest layer
  using Mesh = pcl::PolygonMesh;
  //! Underlying mesh vertex type
  using MeshVertices = pcl::PointCloud<pcl::PointXYZRGBA>;
  //! Underlying mesh triangle type
  using MeshFaces = std::vector<pcl::Vertices>;
  //! Mesh edge container type
  using MeshEdges = std::map<size_t, MeshEdge>;
  //! Callback type
  using LayerVisitor = std::function<void(LayerKey, BaseLayer*)>;

  friend class SceneGraphLogger;

  /**
   * @brief Construct the scene graph (with a default layer factory)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  explicit DynamicSceneGraph(LayerId mesh_layer_id = DsgLayers::MESH);

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   * @param factor List of layer ids (not including the mesh)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  DynamicSceneGraph(const LayerIds& factory, LayerId mesh_layer_id = DsgLayers::MESH);

  /**
   * @brief Default destructor
   */
  virtual ~DynamicSceneGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  void clear();

  /**
   * @brief Add a new dynamic layer to the graph if it doesn't exist already
   *
   * @param layer dynamic layer id
   * @param layer_prefix prefix for the dynamic layer (i.e. id within layer)
   * @return true if the layer was created
   */
  bool createDynamicLayer(LayerId layer, LayerPrefix layer_prefix);

  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param layer_id layer to add to
   * @param node_id node to create
   * @param attrs node attributes
   * @return true if the node was added successfully
   */
  bool emplaceNode(LayerId layer_id, NodeId node_id, NodeAttributes::Ptr&& attrs);

  /**
   * @brief construct and add a dynamic node to the specified layer in the graph
   * @param layer_id layer to add to
   * @param prefix dynamic layer prefix to add to
   * @param timestamp dynamic node timestamp in nanoseconds
   * @param attrs node attributes
   * @param add_edge_to_previous add edge to previous dynamic node (if it exists)
   * @return true if the node was added successfully
   */
  bool emplaceNode(LayerId layer_id,
                   LayerPrefix prefix,
                   std::chrono::nanoseconds timestamp,
                   NodeAttributes::Ptr&& attrs,
                   bool add_edge_to_previous = true);

  /**
   * @brief Add a dynamic node with a specific node id
   * @param layer_id layer to add to
   * @param prev_node_id Node id to insert with
   * @param timestamp dynamic node timestamp
   * @param attrs node attributes to use
   * @returns true if the node was added successfully
   */
  bool emplacePrevDynamicNode(LayerId layer_id,
                              NodeId prev_node_id,
                              std::chrono::nanoseconds timestamp,
                              NodeAttributes::Ptr&& attrs);

  /**
   * @brief add a node to the graph
   *
   * Checks that the layer id matches a current layer, that the node
   * is not null and the node doesn't already exist
   *
   * @param node to add
   * @return true if the node was added successfully
   */
  bool insertNode(SceneGraphNode::Ptr&& node);

  /**
   * @brief add a node to the graph or update an existing node
   *
   * @param layer_id layer to add to
   * @param node_id node to add
   * @param attrs attributes to add
   */
  bool addOrUpdateNode(LayerId layer_id, NodeId node_id, NodeAttributes::Ptr&& attrs);

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
   * @return true if the edge was successfully added
   */
  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeAttributes::Ptr&& edge_info = nullptr,
                  bool force_insert = false);

  /**
   * @brief Add an edge to the graph or update an existing edge
   *
   * @param source edge source id
   * @param target edge target id
   * @param edge_info edge attributes
   */
  bool addOrUpdateEdge(NodeId source, NodeId target, EdgeAttributes::Ptr&& edge_info);

  /**
   * @brief Set the attributes of an existing node
   * @param node Node ID to set the attributes for
   * @param attrs New attributes for the node
   * @return Returns true if update was successful
   */
  bool setNodeAttributes(NodeId node, NodeAttributes::Ptr&& attrs);

  /**
   * @brief Set the attributes of an existing edge
   * @param source Source ID to set the attributes for
   * @param target Target ID to set the attributes for
   * @param attrs New attributes for the edge
   * @return Returns true if update was successful
   */
  bool setEdgeAttributes(NodeId source, NodeId target, EdgeAttributes::Ptr&& attrs);

  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_id Layer id to check
   * @returns Returns true if the layer exists and is valid
   */
  bool hasLayer(LayerId layer_id) const;

  /**
   * @brief Check whether the dynamic layer exists and is valid
   * @param layer layer id to check
   * @param prefix dynamic layer prefix
   * @returns returns true if the layer exists
   */
  bool hasLayer(LayerId layer, LayerPrefix prefix) const;

  /**
   * @brief check if a given node exists
   * @param node_id node to check for
   * @returns true if the given node exists
   */
  bool hasNode(NodeId node_id) const;

  /**
   * @brief Check the status of a node
   * @param node_id node to check for
   * @returns status of type NodeStatus
   */
  NodeStatus checkNode(NodeId node_id) const;

  /**
   * @brief check if a given edge exists
   *
   * This checks either the presence of an edge from source to target or from target
   * to source
   *
   * @param source source id of edge to check for
   * @param target target id of edge to check for
   * @returns true if the given edge exists
   */
  bool hasEdge(NodeId source, NodeId target) const;

  /**
   * @brief Get a layer if the layer exists
   * @param layer_id layer to get
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer doesn't exist
   */
  const SceneGraphLayer& getLayer(LayerId layer_id) const;

  /**
   * @brief Get a dynamic layer if the layer exists
   * @param layer_id layer to get
   * @param prefix layer prefix to get
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer doesn't exist
   */
  const DynamicSceneGraphLayer& getLayer(LayerId layer_id, LayerPrefix prefix) const;

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
  std::optional<NodeRef> getNode(NodeId node_id) const;

  /**
   * @brief Get node layer information (if the node exists)
   * @param node_id Node ID to get
   * @returns A potentially valid Layer ID and prefix
   */
  std::optional<LayerKey> getLayerForNode(NodeId node_id) const;

  /**
   * @brief Get a reference to a dynamic node (if the node exists)
   * @param node_id Dynamic node to get
   * @returns Potentially valid dynamic node reference
   */
  std::optional<DynamicNodeRef> getDynamicNode(NodeId node_id) const;

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
   * @brief Remove a node from the graph
   * @param Node node to remove
   * @returns Returns true if the removal was successful
   */
  bool removeNode(NodeId node);

  /**
   * @brief Remove an edge from the graph
   * @param source Source of edge to remove
   * @param target Target of edge to remove
   * @returns Returns true if the removal was successful
   */
  bool removeEdge(NodeId source, NodeId target);

  /**
   * @brief check if a particular node id is a dynamic node
   * @param source Node to check
   * @returns Return true if the node is a dynamic node
   */
  bool isDynamic(NodeId source) const;

  /**
   * @brief Get the number of layers in the graph
   * @return number of layers in the graph
   */
  size_t numLayers() const;

  /**
   * @brief Get the number of dynamic layers of a specific type
   * @param layer LayerId to count
   * @return number dynamic layers
   */
  size_t numDynamicLayersOfType(LayerId layer) const;

  /**
   * @brief Get the total number of dynamic layers
   * @return number dynamic layers
   */
  size_t numDynamicLayers() const;

  /**
   * @brief Get the total number of nodes in the graph
   * @return The number of nodes in the graph
   */
  size_t numNodes(bool include_mesh = true) const;

  /**
   * @brief Get the number of static nodes in the graph
   * @return The number of static nodes in the graph
   */
  size_t numStaticNodes() const;

  /**
   * @brief Get the number of dynamic nodes in the graph
   * @return The number of dynamic nodes in the graph
   */
  size_t numDynamicNodes() const;

  /**
   * @brief Get number of edges in the graph
   * @return number of edges in the graph
   */
  size_t numEdges(bool include_mesh = true) const;

  /**
   * @brief Get number of static edges in the graph
   * @return number of static edges in the graph
   */
  size_t numStaticEdges() const;

  /**
   * @brief Get number of dynamic edges in the graph
   * @return number of dynamic edges in the graph
   */
  size_t numDynamicEdges() const;

  /**
   * @brief Get whether or not the scene graph is empty
   * @note the scene graph invariants make it so only nodes have to be checked
   * @return true if the scene graph is empty
   */
  bool empty() const;

  /**
   * @brief Get the position of a node in the layer with bounds checking
   */
  Eigen::Vector3d getPosition(NodeId node) const;

  /**
   * @brief Initialize an empty mesh
   */
  void initMesh();

  /**
   * @brief Set mesh components individually
   *
   * This removes any edges that point to vertices that no longer
   * exist (i.e. if the new mesh is smaller than the old mesh)
   *
   * @param vertices Mesh vertices
   * @param faces Mesh triangles
   * @param invalidate_all_edges Clear all existing mesh edges
   */
  void setMesh(const MeshVertices::Ptr& vertices,
               const std::shared_ptr<MeshFaces>& faces,
               bool invalidate_all_edges = false);

  /**
   * @brief Set mesh components directly from a polygon mesh
   * @note doesn't invalidate any edges
   */
  void setMeshDirectly(const pcl::PolygonMesh& mesh);

  /**
   * @brief Check whether the mesh exists and is valid
   * @returns Returns true if the mesh exists and is valid
   */
  bool hasMesh() const;

  /**
   * @brief Check whether the scene graph contains a mesh with actual data
   * @returns Returns true if the mesh has vertices and faces
   */
  bool isMeshEmpty() const;

  /**
   * @brief mesh getter
   * @returns Return scene graph mesh
   */
  pcl::PolygonMesh getMesh() const;

  /**
   * @brief Get mesh vertices pointer (may be invalid)
   * @return Pointer to mesh vertices
   */
  MeshVertices::Ptr getMeshVertices() const;

  /**
   * @brief Get pointer to mesh faces (may be invalid)
   * @returns Pointer to mesh faces
   */
  std::shared_ptr<MeshFaces> getMeshFaces() const;

  /**
   * @brief Get the mesh vertex position (if vaild)
   * @param vertex_id Mesh vertex index
   * @param check_invalid Do bounds checking when getting vertex
   * @returns Potentially valid mesh vertex position
   */
  std::optional<Eigen::Vector3d> getMeshPosition(size_t vertex_id,
                                                 bool check_invalid = true) const;

  /**
   * @brief Add an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @param allow_invalid_mesh Allow edge insertion even if the edge points to an
   * invalid vertice
   * @return Returns true if edge would be valid and was added
   */
  bool insertMeshEdge(NodeId source,
                      size_t mesh_vertex,
                      bool allow_invalid_mesh = false);

  /**
   * @brief check if an edge between a source node and a target mesh vertex exists
   * @param source Source node to check
   * @param mesh_vertex Target mesh vertex to check
   * @returns True if the specified edge exists
   */
  bool hasMeshEdge(NodeId source, size_t mesh_vertex) const;

  /**
   * @brief Get all mesh edges
   * @returns Mesh edges
   */
  const MeshEdges& getMeshEdges() const;

  /**
   * @brief Get mesh vertex indices for a specific node
   * @param node Node to get mesh edges for
   * @returns All mesh vertices the node is connected to
   */
  std::vector<size_t> getMeshConnectionIndices(NodeId node) const;

  /**
   * @brief Remove an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @returns Returns true if edge was removed
   */
  bool removeMeshEdge(NodeId source, size_t mesh_vertex);

  /**
   * @brief Delete all edges associated with a mesh vertex
   * @param index vertex index to delete
   */
  void invalidateMeshVertex(size_t index);

  /**
   * @brief Delete all mesh edges
   */
  void clearMeshEdges();

  /**
   * @brief merge two nodes
   * @param node_from node to remove
   * @param node_to node to merge to
   * @returns true if operation succeeded
   */
  bool mergeNodes(NodeId node_from, NodeId node_to);

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
   * @param previous_merges Prevously merged nodes in current graph
   * @param merge_mesh_edges Allow combining mesh edges
   * @param allow_invalid_mesh Add mesh edges for mesh vertices that don't exist
   * @param clear_mesh_edges Delete all previous mesh edges
   * @param attribute_update_map flags per layer to enable merging attributes
   * @param update_dynamic_attributes update dynamic node attributes from other
   * @param clear_removed Delete any removed nodes in other
   * @returns true if merge was successful
   */
  bool mergeGraph(const DynamicSceneGraph& other,
                  const std::map<NodeId, NodeId>& previous_merges,
                  bool merge_mesh_edges = true,
                  bool allow_invalid_mesh = false,
                  bool clear_mesh_edges = true,
                  std::map<LayerId, bool>* attribute_update_map = nullptr,
                  bool update_dynamic_attributes = true,
                  bool clear_removed = false);

  /**
   * @brief Update graph from another graph
   * @note Will add the nodes and edges not previously added in current graph
   * @param other other graph to update from
   * @param merge_mesh_edges Allow combining mesh edges
   * @param allow_invalid_mesh Add mesh edges for mesh vertices that don't exist
   * @param clear_mesh_edges Delete all previous mesh edges
   * @param attribute_update_map flags per layer to enable merging attributes
   * @param update_dynamic_attributes update dynamic node attributes from other
   * @param clear_removed Delete any removed nodes in other
   * @returns true if merge was successful
   */
  bool mergeGraph(const DynamicSceneGraph& other,
                  bool merge_mesh_edges = true,
                  bool allow_invalid_mesh = false,
                  bool clear_mesh_edges = true,
                  std::map<LayerId, bool>* attribute_update_map = nullptr,
                  bool update_dynamic_attributes = true,
                  bool clear_removed = false);

  /**
   * @brief Get all removed nodes from the graph
   * @param clear_removed Reset removed node tracking
   * @returns List of all removed nodes
   */
  std::vector<NodeId> getRemovedNodes(bool clear_removed = false);

  /**
   * @brief Get all new nodes in the graph
   * @param clear_new Clear new status for all new nodes up until this point
   * @returns list of all new nodes
   */
  std::vector<NodeId> getNewNodes(bool clear_new = false);

  /**
   * @brief Get all removed edges from the graph
   * @param clear_removed Reset removed edge tracking
   * @returns List of all removed edges
   */
  std::vector<EdgeKey> getRemovedEdges(bool clear_removed = false);

  /**
   * @brief Get all new edges in the graph
   * @param clear_new Clear new status for all new edges up until this point
   * @returns list of all new edges
   */
  std::vector<EdgeKey> getNewEdges(bool clear_new = false);

  /**
   * @brief track which edges get used during a serialization update
   */
  void markEdgesAsStale();

  /**
   * @brief remove edges that do not appear in serialization update
   */
  void removeAllStaleEdges();

  /**
   * @brief Clone the scene graph
   * @returns Copy of the scene graph
   */
  DynamicSceneGraph::Ptr clone() const;

  /**
   * @brief Save JSON graph representation to file
   * @param filepath Filepath to save graph to
   * @param include_mesh Optionally encode mesh (defaults to true)
   */
  void save(const std::string& filepath, bool include_mesh = true) const;

  /**
   * @brief Get JSON string representing graph
   * @param include_mesh Optionally encode mesh (defaults to false)
   * @returns JSON string representing graph
   */
  std::string serialize(bool include_mesh = false) const;

  /**
   * @brief parse graph from JSON file
   * @param filepath Path to JSON file to read
   * @returns Resulting parsed scene graph
   */
  static Ptr load(const std::string& filepath);

  /**
   * @brief parse graph from JSON string
   * @param contents JSON string to parse
   * @returns Resulting parsed scene graph
   */
  static Ptr deserialize(const std::string& contents);

  //! mesh layer id
  const LayerId mesh_layer_id;

  //! current static layer ids in the graph
  const LayerIds layer_ids;

 protected:
  BaseLayer& layerFromKey(const LayerKey& key);

  const BaseLayer& layerFromKey(const LayerKey& key) const;

  SceneGraphNode* getNodePtr(NodeId node, const LayerKey& key) const;

  bool hasEdge(NodeId source,
               NodeId target,
               LayerKey* source_key,
               LayerKey* target_key) const;

  bool addAncestry(NodeId source,
                   NodeId target,
                   const LayerKey& source_key,
                   const LayerKey& target_key);

  void removeAncestry(NodeId source,
                      NodeId target,
                      const LayerKey& source_key,
                      const LayerKey& target_key);

  void clearParentAncestry(NodeId source,
                           NodeId target,
                           const LayerKey& source_key,
                           const LayerKey& target_key);

  void removeInterlayerEdge(NodeId source,
                            NodeId target,
                            const LayerKey& source_key,
                            const LayerKey& target_key);

  void removeInterlayerEdge(NodeId n1, NodeId n2);

  void rewireInterlayerEdge(NodeId source, NodeId new_source, NodeId target);

  void removeStaleEdges(EdgeContainer& edges);

  void clearMeshEdgesForNode(NodeId node_id);

  void visitLayers(const LayerVisitor& cb);

 protected:
  Layers layers_;
  std::map<LayerId, DynamicLayers> dynamic_layers_;

  std::map<NodeId, LayerKey> node_lookup_;

  EdgeContainer interlayer_edges_;
  EdgeContainer dynamic_interlayer_edges_;

  MeshVertices::Ptr mesh_vertices_;
  std::shared_ptr<MeshFaces> mesh_faces_;

  size_t next_mesh_edge_idx_;
  MeshEdges mesh_edges_;
  std::map<NodeId, std::map<size_t, size_t>> mesh_edges_node_lookup_;
  std::map<size_t, std::map<NodeId, size_t>> mesh_edges_vertex_lookup_;

 public:
  /**
   * @brief constant iterator around the layers
   */
  inline const Layers& layers() const { return layers_; };

  /**
   * @brief constant iterator over mapping between nodes and layers
   */
  inline const std::map<NodeId, LayerKey>& node_lookup() const { return node_lookup_; }

  /**
   * @brief constant iterator around the inter-layer edges
   *
   * @note inter-layer edges are edges between nodes in different layers
   */
  inline const Edges& interlayer_edges() const { return interlayer_edges_.edges; };

  inline const DynamicLayers& dynamicLayersOfType(LayerId layer_id) const {
    auto iter = dynamic_layers_.find(layer_id);
    if (iter == dynamic_layers_.end()) {
      static DynamicLayers empty;  // avoid invalid reference
      return empty;
    }

    return iter->second;
  }

  inline const std::map<LayerId, DynamicLayers>& dynamicLayers() const {
    return dynamic_layers_;
  }

  /**
   * @brief constant iterator around the dynamic interlayer edges
   *
   * @note dynamic interlayer edges are edges between nodes in different layers where at
   * least one of the nodes is dynamic
   */
  inline const Edges& dynamic_interlayer_edges() const {
    return dynamic_interlayer_edges_.edges;
  };
};

/**
 * @brief Return a container of the layer hierarchy from #spark_dsg::DsgLayers
 */
DynamicSceneGraph::LayerIds getDefaultLayerIds();

}  // namespace spark_dsg
