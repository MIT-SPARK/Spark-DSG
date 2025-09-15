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
#include <Eigen/Core>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <type_traits>

#include "spark_dsg/metadata.h"
#include "spark_dsg/scene_graph_layer.h"
#include "spark_dsg/spark_dsg_fwd.h"

namespace spark_dsg {

struct EdgeLayerInfo {
  LayerKey source;
  LayerKey target;
  bool valid = false;
  bool exists = false;

  bool isSameLayer() const;
};

/**
 * @brief 3D Scene Graph class
 *
 * The scene graph is comprised of multiple (optionally named) layers,
 * where the layers are ordered by numerical ID. Between two layers with different IDs,
 * the greater ID is the ancestor and the lesser ID is the descendant.
 *
 * Layers are also optionally partitioned. Partitions own disjoint sets of nodes,
 * and partition 0 refers to the "primary" partition. All functions default to using the
 * "primary" partition unless otherwise specified.
 *
 * Edges are either intralayer (between nodes in the same partition and layer) or
 * interlayer (between nodes in different partitions or layers). Interlayer edges
 * are managed directly by the scene graph while intralayer edges are managed by
 * the associated partition.
 *
 * SceneGraphNode and SceneGraphEdge consist of topological information about the node
 * or edge and are not mutable. They each also have a pointer to associated attributes.
 * These underlying attributes of the node or edge always are mutable. Note that this
 * has the implication that a const scene graph is one that is topologically constant
 * (instead of the attributes also being constant).
 */
class DynamicSceneGraph {
 public:
  friend class SceneGraphLogger;
  //! Desired pointer type of the scene graph
  using Ptr = std::shared_ptr<DynamicSceneGraph>;
  //! Container type for the layer keys
  using LayerKeys = std::vector<LayerKey>;
  //! Container type for the static layers
  using LayerIds = std::vector<LayerId>;
  //! Container type for the layer name to ID mapping
  using LayerNames = std::map<std::string, LayerKey>;
  //! Edge container
  using Edges = EdgeContainer::Edges;
  //! Scene graph layer
  using Layer = SceneGraphLayer;
  //! Layer container
  using Layers = std::map<LayerId, Layer::Ptr>;
  //! Container for layer partitions
  using Partitions = std::map<PartitionId, Layer::Ptr>;

  /**
   * @brief Construct the scene graph
   * @param empty Whether or not to skip the default layer 2-5 initialization
   */
  explicit DynamicSceneGraph(bool empty = false);
  /**
   * @brief Construct the scene graph (with the provided layers)
   * @param layers List of layer ids
   * @param layer_names Optional names for the layers
   * @note Adds IDs for layers in layer_names that aren't in layers
   */
  DynamicSceneGraph(const LayerKeys& layers, const LayerNames& layer_names = {});
  virtual ~DynamicSceneGraph() = default;

  /**
   * @brief Construct a scene graph
   * @param layers Mapping between layer names and layer IDs
   */
  static DynamicSceneGraph::Ptr fromNames(const LayerNames& layers);

  /**
   * @brief Delete all layers and edges
   * @param include_mesh If true also clear the mesh
   */
  void clear(bool include_mesh = true);

  //! @brief Reset scene graph to use new layers
  void reset(const LayerKeys& layer, const LayerNames& layer_names = {});

  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_id Layer id to check
   * @param partition Partition to check if specified
   * @returns Returns true if the layer exists and is valid
   */
  bool hasLayer(LayerId layer_id, PartitionId partition = 0) const;
  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_name Layer name to check
   * @returns Returns true if the layer exists and is valid
   */
  bool hasLayer(const std::string& layer_name) const;
  /**
   * @brief Attempt to retrieve the specified layer
   * @param layer_id Layer ID to find
   * @param partition Partition to find if specified
   * @returns Returns a valid pointer to the layer if it exists (nullptr otherwise)
   */
  const Layer* findLayer(LayerId layer_id, PartitionId partition = 0) const;
  /**
   * @brief Attempt to retrieve the specified layer
   * @param layer_name Layer name to find
   * @returns Returns a valid pointer to the layer if it exists (nullptr otherwise)
   */
  const Layer* findLayer(const std::string& layer_name) const;
  /**
   * @brief Get a layer if the layer exists
   * @param layer_id layer to get
   * @param partition Partition to get if specified
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer doesn't exist
   */
  const Layer& getLayer(LayerId layer_id, PartitionId partition = 0) const;
  /**
   * @brief Get a layer if the layer exists
   * @param layer_name layer to get
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer name doesn't exist
   */
  const Layer& getLayer(const std::string& layer_name) const;
  /**
   * @brief Add a new layer with an optional name
   * @param layer_id Layer ID
   * @param partition Partition to add
   * @param name Optional layer name
   * @return Layer that was created or existed previously
   */
  const Layer& addLayer(LayerId layer_id,
                        PartitionId partition = 0,
                        const std::string& name = "");

  /**
   * @brief Remove a layer from the graph if it exists
   * @param layer Layer ID
   * @param partition Partition to get if specified
   * @return true if the layer was created
   */
  void removeLayer(LayerId layer, PartitionId partition = 0);

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
   * @brief Get a particular node in the graph
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns The scene graph node
   */
  const SceneGraphNode& getNode(NodeId node_id) const;
  /**
   * @brief Get a particular node in the graph
   * @param node_id node to get
   * @returns Pointer to the node if it exists, nullptr otherwise
   */
  const SceneGraphNode* findNode(NodeId node_id) const;
  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param layer_id layer to add to
   * @param node_id node to create
   * @param attrs node attributes
   * @param partition to add to
   * @return true if the node was added successfully
   */
  bool emplaceNode(LayerId layer_id,
                   NodeId node_id,
                   std::unique_ptr<NodeAttributes>&& attrs,
                   PartitionId partition = 0);
  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param layer layer to add to
   * @param node_id node to create
   * @param attrs node attributes
   * @return true if the node was added successfully
   */
  bool emplaceNode(const std::string& layer,
                   NodeId node_id,
                   std::unique_ptr<NodeAttributes>&& attrs);
  /**
   * @brief construct and add a node to the specified layer in the graph
   * @param node_id node to create
   * @param attrs node attributes
   * @return true if the node was added successfully
   */
  bool emplaceNode(LayerKey layer,
                   NodeId node_id,
                   std::unique_ptr<NodeAttributes>&& attrs);
  /**
   * @brief add a node to the graph or update an existing node
   * @param layer layer to add to
   * @param node_id node to add
   * @param attrs attributes to add
   * @param partition to add to
   * @return true if the node was added or updated successfully
   */
  bool addOrUpdateNode(const std::string& layer,
                       NodeId node_id,
                       std::unique_ptr<NodeAttributes>&& attrs);
  /**
   * @brief add a node to the graph or update an existing node
   * @param layer_id layer to add to
   * @param node_id node to add
   * @param attrs attributes to add
   * @param partition to add to
   * @return true if the node was added or updated successfully
   */
  bool addOrUpdateNode(LayerId layer_id,
                       NodeId node_id,
                       std::unique_ptr<NodeAttributes>&& attrs,
                       PartitionId partition = 0);
  /**
   * @brief Set the attributes of an existing node
   * @param node_id Node to set attributes for
   * @param attrs New attributes for the node
   * @return true if the attributes for the node were set
   */
  bool setNodeAttributes(NodeId node_id, std::unique_ptr<NodeAttributes>&& attrs);
  /**
   * @brief Remove a node from the graph
   * @param Node node to remove
   * @returns Returns true if the removal was successful
   */
  bool removeNode(NodeId node);

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
   * @brief Get a particular edge in the graph
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns a valid edge constant reference
   */
  const SceneGraphEdge& getEdge(NodeId source, NodeId target) const;
  /**
   * @brief Get a particular edge in the graph
   *
   * This can be used to update the edge "info", though
   * information about the edge (i.e. source and target) cannot
   * be modified
   *
   * @param source source of edge to get
   * @param target target of edge to get
   * @returns Pointer to the edge if it exists, nullptr otherwise
   */
  const SceneGraphEdge* findEdge(NodeId source, NodeId target) const;
  /**
   * @brief Add an edge to the graph
   *
   * Checks that the edge doesn't already exist and
   * that the source and target already exist. Handles passing
   * the edge to the layer if the edge is a intralayer edge,
   * and updates parents and children of the respective nodes
   *
   * @param source start node
   * @param target end node
   * @param edge_info optional edge attributes (will use
   *        default edge attributes if not supplied)
   * @param enforce_parent_constraints Ensure that edge is only parent of node
   * @return true if the edge was successfully added
   */
  bool insertEdge(NodeId source,
                  NodeId target,
                  std::unique_ptr<EdgeAttributes>&& edge_info = nullptr,
                  bool enforce_parent_constraints = false);
  /**
   * @brief Add an edge to the graph or update an existing edge
   *
   * @param source edge source id
   * @param target edge target id
   * @param edge_info edge attributes
   * @param enforce_parent_constraints Ensure that edge is only parent of node
   */
  bool addOrUpdateEdge(NodeId source,
                       NodeId target,
                       std::unique_ptr<EdgeAttributes>&& edge_info,
                       bool enforce_parent_constraints = false);
  /**
   * @brief Remove an edge from the graph
   * @param source Source of edge to remove
   * @param target Target of edge to remove
   * @returns Returns true if the removal was successful
   */
  bool removeEdge(NodeId source, NodeId target);

  //! @brief Get the number of layers in the graph
  size_t numLayers() const;

  //! @brief Get the total number of nodes in the graph
  size_t numNodes() const;

  //! @brief Get the total number of nodes in partition 0 in each layer
  size_t numUnpartitionedNodes() const;

  //! @brief Get number of edges in the graph
  size_t numEdges() const;

  //! @brief Get the total number of edges between nodes in partition 0
  size_t numUnpartitionedEdges() const;

  //! @brief Get whether or not the scene graph is empty
  bool empty() const;

  /**
   * @brief Get the 3D position of a node
   * @param node Node ID to retrieve position for
   * @return Position of node
   */
  Eigen::Vector3d getPosition(NodeId node) const;

  /**
   * @brief Merge two nodes
   * @param node_from Node to remove
   * @param node_to Node to merge to
   * @returns True if operation succeeded
   */
  bool mergeNodes(NodeId node_from, NodeId node_to);

  /**
   * @brief Update graph from separate layer
   * @param other_layer Layer to update from
   * @param edges Optional edges to add to graph
   * @return Whether the update was successful or not
   */
  bool updateFromLayer(const SceneGraphLayer& other_layer, const Edges& edges = {});

  /**
   * @brief Update graph from another graph
   * @note Will add the nodes and edges not previously added in current graph
   * @param other other graph to update from
   * @param config Configuration controlling merge behavior
   * @param transform the other graph when merging
   * @returns True if merge was successful
   */
  bool mergeGraph(const DynamicSceneGraph& other,
                  const GraphMergeConfig& config = {},
                  const Eigen::Isometry3d* transform = nullptr);

  /**
   * @brief Get all removed nodes from the graph
   * @param clear_removed Reset removed node tracking
   * @returns List of all removed nodes
   */
  std::vector<NodeId> getRemovedNodes(bool clear_removed = false);

  /**
   * @brief Get all new nodes in the graph
   * @param clear_new Clear new status for all new nodes up until this point
   * @returns List of all new nodes
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
   * @brief Get whether an edge points to a partition
   * @param edge Edge to check
   * @return True if at least one end point is in a partition
   */
  bool edgeToPartition(const SceneGraphEdge& edge) const;

  //! @brief Set all current edges as stale for serialization update tracking
  void markEdgesAsStale();

  //! @brief Remove edges that do not appear in serialization update
  void removeAllStaleEdges();

  //! @brief Make a copy of the scene graph
  DynamicSceneGraph::Ptr clone() const;

  //! @brief Rigidly transform graph
  void transform(const Eigen::Isometry3d& transform);

  /**
   * @brief Save the DSG to file. By default, this will save a binary version of the
   * graph. To save as JSON, specify the filepath with a .json extension.
   * @param filepath Filepath to save graph to.
   * @param include_mesh Optionally encode mesh (defaults to true)
   */
  void save(std::filesystem::path filepath, bool include_mesh = true) const;

  /**
   * @brief Parse graph from binary or JSON file
   * @param filepath Complete path to file to read, including extension.
   * @returns Resulting parsed scene graph
   */
  static Ptr load(std::filesystem::path filepath);

  //! @brief Set the scene graph mesh
  void setMesh(const std::shared_ptr<Mesh>& mesh);

  //! @brief Check if the scene graph has a mesh
  bool hasMesh() const;

  //! @brief Get the mesh associated with the scene graph
  std::shared_ptr<Mesh> mesh() const;

  //! Any extra information about the graph
  Metadata metadata;

 protected:
  Layer& layerFromKey(const LayerKey& key);

  const Layer& layerFromKey(const LayerKey& key) const;

  SceneGraphNode* getNodePtr(NodeId node, const LayerKey& key) const;

  EdgeLayerInfo lookupEdge(NodeId source, NodeId target) const;

  void addAncestry(NodeId source,
                   NodeId target,
                   const LayerKey& source_key,
                   const LayerKey& target_key);

  void removeAncestry(NodeId source,
                      NodeId target,
                      const LayerKey& source_key,
                      const LayerKey& target_key);

  void dropAllParents(NodeId source,
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

  void visitLayers(const std::function<void(LayerKey, Layer&)>& cb);

  void visitLayers(const std::function<void(LayerKey, const Layer&)>& cb) const;

 protected:
  std::set<LayerKey> layer_keys_;
  LayerNames layer_names_;
  std::map<NodeId, LayerKey> node_lookup_;

  Layers layers_;
  std::map<LayerId, Partitions> layer_partitions_;

  EdgeContainer interlayer_edges_;

  std::shared_ptr<Mesh> mesh_;

 public:
  //! @brief Get layer key for a named layer
  std::optional<LayerKey> getLayerKey(const std::string& name) const {
    auto iter = layer_names_.find(name);
    return iter == layer_names_.end() ? std::nullopt
                                      : std::optional<LayerKey>(iter->second);
  }

  //! @brief Current static layer ids in the graph
  std::vector<LayerId> layer_ids() const;
  //! @brief Current layer keys of all layers in the graph
  LayerKeys layer_keys() const;
  //! @brief Current name to layer mapping
  const LayerNames layer_names() const { return layer_names_; }
  //! @brief Constant reference to the layers
  const Layers& layers() const { return layers_; };
  //! @brief Constant reference to the mapping between nodes and layers
  const std::map<NodeId, LayerKey>& node_lookup() const { return node_lookup_; }
  //! @brief Const reference to the interlayer edges
  const Edges& interlayer_edges() const { return interlayer_edges_.edges; };
  //! @brief Constant reference to partitions for a particular layer
  const Partitions& layer_partition(LayerId layer_id) const;
  //! @brief Constant reference to all layer partitions
  const std::map<LayerId, Partitions>& layer_partitions() const {
    return layer_partitions_;
  }
};

}  // namespace spark_dsg
