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
#include <nlohmann/json.hpp>
#include <type_traits>

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
 * @brief Dynamic Scene Graph class
 *
 * Contains an explicit mesh layer
 */
class DynamicSceneGraph {
 public:
  //! Desired pointer type of the scene graph
  using Ptr = std::shared_ptr<DynamicSceneGraph>;
  //! Container type for the layer ids
  using LayerIds = std::vector<LayerId>;
  //! Container type for the layer name to ID mapping
  using LayerNames = std::map<std::string, LayerId>;
  //! Edge container
  using Edges = EdgeContainer::Edges;
  //! Scene graph layer
  using Layer = SceneGraphLayer;
  //! Layer container
  using Layers = std::map<LayerId, Layer::Ptr>;
  //! Container for layer partitions
  using Partitions = std::map<PartitionId, Layer::Ptr>;

  friend class SceneGraphLogger;

  /**
   * @brief Construct the scene graph
   * @param empty Whether or not to skip the default layer 2-5 initialization
   */
  explicit DynamicSceneGraph(bool empty = false);

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   * @param factory List of layer ids
   * @param layer_names Names for each layer ID
   */
  DynamicSceneGraph(const LayerIds& factory, const LayerNames& layer_names = {});

  /**
   * @brief Default destructor
   */
  virtual ~DynamicSceneGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  void clear();

  /**
   * @brief Reset scene graph to use new layer ids
   */
  void reset(const LayerIds& layer_ids);

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
   * @param partition Partition to check if specified
   * @returns Returns true if the layer exists and is valid
   */
  bool hasLayer(const std::string& layer_name, PartitionId partition = 0) const;

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
   * @param partition Partition to find if specified
   * @returns Returns a valid pointer to the layer if it exists (nullptr otherwise)
   */
  const Layer* findLayer(const std::string& layer_name,
                         PartitionId partition = 0) const;

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
   * @param partition Partition to get if specified
   * @returns a constant reference to the requested layer
   * @throws std::out_of_range if the layer name doesn't exist
   */
  const Layer& getLayer(const std::string& layer_name, PartitionId partition = 0) const;

  /**
   * @brief Add a new layer with an optional name
   * @param layer_id Layer ID
   * @param name Optional layer name
   */
  const Layer& addLayer(LayerId layer_id, const std::string& name = "");

  /**
   * @brief Add a new partition to a layer
   * @param layer_id Layer to add partition to
   * @param partition Partition to add
   * @return Layer that was created or existed previously
   */
  const Layer& addLayer(LayerId layer_id, PartitionId partition);

  /**
   * @brief Add a new partition to a layer
   * @param name Layer name to add
   * @param partition Partition to add
   * @return Layer that was created or existed previously
   * @throws std::out_of_range if the layer name doesn't exist
   */
  const Layer& addLayer(const std::string& name, PartitionId partition);

  /**
   * @brief Remove a layer from the graph if it exists
   * @param layer Layer ID
   * @param partition Partition to get if specified
   * @return true if the layer was created
   */
  void removeLayer(LayerId layer, PartitionId partition = 0);

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
   * @param partition to add to
   * @return true if the node was added successfully
   */
  bool emplaceNode(const std::string& layer,
                   NodeId node_id,
                   std::unique_ptr<NodeAttributes>&& attrs,
                   PartitionId partition = 0);

  /**
   * @brief construct and add a node to the specified layer in the graph
   */
  bool emplaceNode(LayerKey layer,
                   NodeId node_id,
                   std::unique_ptr<NodeAttributes>&& attrs);

  /**
   * @brief add a node to the graph or update an existing node
   *
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
   *
   * This can be used to update the node attributes, though
   * information about the node (i.e. siblings, etc) cannot
   * be modified
   *
   * @param node_id node to get
   * @returns a pointer to a potentially valid node
   */
  const SceneGraphNode* findNode(NodeId node_id) const;

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
   * @returns a potentially valid edge constant reference
   */
  const SceneGraphEdge* findEdge(NodeId source, NodeId target) const;

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
   * @brief Get the number of layers in the graph
   * @return number of layers in the graph
   */
  size_t numLayers() const;

  /**
   * @brief Get the total number of nodes in the graph
   * @return The number of nodes in the graph
   */
  size_t numNodes() const;

  /**
   * @brief Get the total number of nodes in the graph
   * @return The number of nodes in the graph
   */
  size_t numUnpartitionedNodes() const;

  /**
   * @brief Get number of edges in the graph
   */
  size_t numEdges() const;

  /**
   * @brief Get the total number of nodes in the graph
   * @return The number of nodes in the graph
   */
  size_t numUnpartitionedEdges() const;

  /**
   * @brief Get whether or not the scene graph is empty
   * @note The scene graph invariants make it so only nodes have to be checked
   * @return True if the scene graph is empty
   */
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
   * @returns True if merge was successful
   */
  bool mergeGraph(const DynamicSceneGraph& other, const GraphMergeConfig& config = {});

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

  /**
   * @brief Track which edges get used during a serialization update
   */
  void markEdgesAsStale();

  /**
   * @brief Remove edges that do not appear in serialization update
   */
  void removeAllStaleEdges();

  /**
   * @brief Clone the scene graph
   * @returns Copy of the scene graph
   */
  DynamicSceneGraph::Ptr clone() const;

  /**
   * @brief Save the DSG to file. By default, this will save a binary version of the
   * graph. To save as JSON, specify the filepath with a .json extension.
   * @param filepath Filepath to save graph to.
   * @param include_mesh Optionally encode mesh (defaults to true)
   */
  void save(std::string filepath, bool include_mesh = true) const;

  /**
   * @brief Parse graph from binary or JSON file
   * @param filepath Complete path to file to read, including extension.
   * @returns Resulting parsed scene graph
   */
  static Ptr load(std::string filepath);

  void setMesh(const std::shared_ptr<Mesh>& mesh);

  bool hasMesh() const;

  std::shared_ptr<Mesh> mesh() const;

  //! Any extra information about the graph
  nlohmann::json metadata;

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
  LayerIds layer_ids_;
  LayerNames layer_names_;
  std::map<NodeId, LayerKey> node_lookup_;

  Layers layers_;
  std::map<LayerId, Partitions> layer_partitions_;

  EdgeContainer interlayer_edges_;

  std::shared_ptr<Mesh> mesh_;

 public:
  //! @brief Get layer key for a named layer
  std::optional<LayerKey> getLayerKey(const std::string& name,
                                      PartitionId partition = 0) const {
    auto iter = layer_names_.find(name);
    return iter == layer_names_.end()
               ? std::nullopt
               : std::optional<LayerKey>({iter->second, partition});
  }

  //! Current static layer ids in the graph
  const LayerIds& layer_ids() const { return layer_ids_; }

  //! Current name to layer mapping
  const LayerNames layer_names() const { return layer_names_; }

  /**
   * @brief constant iterator around the layers
   */
  const Layers& layers() const { return layers_; };

  /**
   * @brief constant iterator over mapping between nodes and layers
   */
  const std::map<NodeId, LayerKey>& node_lookup() const { return node_lookup_; }

  /**
   * @brief constant iterator around the inter-layer edges
   *
   * @note inter-layer edges are edges between nodes in different layers
   */
  const Edges& interlayer_edges() const { return interlayer_edges_.edges; };

  const Partitions& layer_partition(const std::string& name) const {
    auto iter = layer_names_.find(name);
    if (iter == layer_names_.end()) {
      throw std::out_of_range("missing layer '" + name + "'");
    }

    return layer_partition(iter->second);
  }

  const Partitions& layer_partition(LayerId layer_id) const {
    auto iter = layer_partitions_.find(layer_id);
    if (iter == layer_partitions_.end()) {
      static Partitions empty;  // avoid invalid reference
      return empty;
    }

    return iter->second;
  }

  const std::map<LayerId, Partitions>& layer_partitions() const {
    return layer_partitions_;
  }
};

}  // namespace spark_dsg
