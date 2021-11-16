#pragma once
#include "kimera_dsg/dynamic_scene_graph_layer.h"
#include "kimera_dsg/scene_graph.h"

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kimera {

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
class DynamicSceneGraph : public SceneGraph {
 public:
  //! Desired pointer type of the scene graph
  using Ptr = std::shared_ptr<DynamicSceneGraph>;
  //! Container type for the layer ids
  using LayerIds = SceneGraph::LayerIds;
  //! Underlying mesh type for lowest layer
  using Mesh = pcl::PolygonMesh;
  //! Underlying mesh vertex type
  using MeshVertices = pcl::PointCloud<pcl::PointXYZRGBA>;
  //! Underlying mesh triangle type
  using MeshFaces = std::vector<pcl::Vertices>;
  //! Mesh edge container type
  using MeshEdges = std::map<size_t, MeshEdge>;
  //! Dynamic layer type
  using DynamicLayer = DynamicSceneGraphLayer;
  //! Dynamic node reference
  using DynamicNodeRef = DynamicLayer::NodeRef;
  //! Dynamic layer container type
  using DynamicLayers = std::map<char, DynamicLayer::Ptr>;
  //! Dynamic layer reference
  using DynamicLayerRef = std::reference_wrapper<const DynamicLayer>;

  /**
   * @brief Construct the scene graph (with a default layer factory)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  explicit DynamicSceneGraph(LayerId mesh_layer_id = KimeraDsgLayers::MESH);

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   * @param factor List of layer ids (not including the mesh)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  DynamicSceneGraph(const LayerIds& factory,
                    LayerId mesh_layer_id = KimeraDsgLayers::MESH);

  /**
   * @brief Default destructor
   */
  virtual ~DynamicSceneGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  virtual void clear() override;

  inline bool emplaceNode(LayerId layer_id,
                          NodeId node_id,
                          NodeAttributes::Ptr&& attrs) {
    if (dynamic_node_lookup_.count(node_id)) {
      return false;
    }

    return SceneGraph::emplaceNode(layer_id, node_id, std::move(attrs));
  }

  inline bool insertNode(Node::Ptr&& node) override {
    if (!node) {
      return false;
    }

    if (dynamic_node_lookup_.count(node->id)) {
      return false;
    }

    return SceneGraph::insertNode(std::move(node));
  }

  virtual bool insertEdge(NodeId source,
                          NodeId target,
                          EdgeInfo::Ptr&& edge_info = nullptr) override;

  bool insertDynamicEdge(NodeId source,
                         NodeId target,
                         EdgeInfo::Ptr&& edge_info = nullptr);

  virtual bool hasEdge(NodeId source, NodeId target) const override;

  bool createDynamicLayer(LayerId layer, char layer_prefix);

  bool hasDynamicLayer(LayerId layer, char layer_prefix) const;

  size_t numDynamicLayersOfType(LayerId layer) const;

  size_t numDynamicLayers() const;

  std::optional<DynamicLayerRef> getDynamicLayer(LayerId layer_id, char prefix) const;

  bool emplaceDynamicNode(LayerId layer_id,
                          char prefix,
                          std::chrono::nanoseconds timestamp,
                          NodeAttributes::Ptr&& attrs,
                          bool add_edge_to_previous = true);

  virtual bool hasNode(NodeId node_id) const override;

  virtual std::optional<NodeRef> getNode(NodeId node_id) const override;

  std::optional<DynamicNodeRef> getDynamicNode(NodeId node_id) const;

  /**
   * @brief Set mesh components directly from a polygon mesh
   * @note doesn't invalidate any edges
   */
  void setMeshDirectly(const pcl::PolygonMesh& mesh);

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

  inline MeshVertices::Ptr getMeshVertices() const { return mesh_vertices_; }

  inline std::shared_ptr<MeshFaces> getMeshFaces() const { return mesh_faces_; }

  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_id Layer id to check
   * @returns Returns true if the layer exists and is valid
   */
  virtual bool hasLayer(LayerId layer_id) const override;

  /**
   * @brief Check whether the mesh exists and is valid
   * @returns Returns true if the mesh exists and is valid
   */
  inline bool hasMesh() const {
    return mesh_vertices_ != nullptr && mesh_faces_ != nullptr;
  }

  /**
   * @brief Remove a node from the graph
   * @param Node node to remove
   * @returns Returns true if the removal was successful
   */
  virtual bool removeNode(NodeId node) override;

  /**
   * @brief Remove an edge from the graph
   * @param source Source of edge to remove
   * @param target Target of edge to remove
   * @returns Returns true if the removal was successful
   */
  virtual bool removeEdge(NodeId source, NodeId target) override;

  /**
   * @brief Add an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @param allow_invalid_mesh Allow edge insertion even if the edge points to an
   * invalid vertice
   * @returns Returns true if edge would be valid and was added
   */
  bool insertMeshEdge(NodeId source,
                      size_t mesh_vertex,
                      bool allow_invalid_mesh = false);

  /**
   * @brief Remove an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @returns Returns true if edge was removed
   */
  bool removeMeshEdge(NodeId source, size_t mesh_vertex);

  inline bool isDynamic(NodeId source) const {
    return dynamic_node_lookup_.count(source);
  }

  virtual size_t numLayers() const override;

  virtual size_t numNodes() const override;

  size_t numDynamicNodes() const;

  virtual size_t numEdges() const override;

  /**
   * @brief Update graph from another graph
   * @note Will add the nodes and edges not previously added in current graph
   * @param other other graph to update from
   */
  bool mergeGraph(const DynamicSceneGraph& other,
                  bool allow_invalid_mesh = false);

  inline LayerId getMeshLayerId() const { return mesh_layer_id_; }

  std::optional<Eigen::Vector3d> getMeshPosition(size_t vertex_id) const;

  std::vector<size_t> getMeshConnectionIndices(NodeId node) const;

  virtual Eigen::Vector3d getPosition(NodeId node) const override;

  virtual nlohmann::json toJson(const JsonExportConfig& config) const override;

  virtual void fillFromJson(const JsonExportConfig& config,
                            const NodeAttributeFactory& node_attr_factory,
                            const EdgeInfoFactory& edge_info_factory,
                            const nlohmann::json& record) override;

 protected:
  // TODO(nathan) consider making const public
  LayerId mesh_layer_id_;

  bool hasMeshEdge(NodeId source, size_t mesh_vertex) const;

  void clearMeshEdges();

  MeshVertices::Ptr mesh_vertices_;
  std::shared_ptr<MeshFaces> mesh_faces_;

  size_t next_mesh_edge_idx_;
  MeshEdges mesh_edges_;
  std::map<NodeId, std::map<size_t, size_t>> mesh_edges_node_lookup_;
  std::map<size_t, std::map<NodeId, size_t>> mesh_edges_vertex_lookup_;

  std::map<LayerId, DynamicLayers> dynamic_layers_;
  std::map<NodeId, DynamicLayerKey> dynamic_node_lookup_;

  Edges dynamic_interlayer_edges_;
  EdgeLookup dynamic_interlayer_edges_info_;
  size_t next_dynamic_edge_idx_;

 private:
  bool isDynamicEdge(NodeId source, NodeId target) const;

  bool inSameDynamicLayer(NodeId source, NodeId target) const;

  bool hasDynamicEdge(NodeId source, NodeId target) const;

  SceneGraphNode* getNodePtr(NodeId node) const;

 public:
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
    return dynamic_interlayer_edges_;
  };

  /**
   * @brief mesh getter
   */
  inline pcl::PolygonMesh getMesh() const {
    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*mesh_vertices_, mesh.cloud);
    mesh.polygons = *mesh_faces_;
    return mesh;
  };
};

}  // namespace kimera
