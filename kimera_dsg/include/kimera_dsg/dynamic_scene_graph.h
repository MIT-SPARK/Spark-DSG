#pragma once
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
  using LayerIds = std::vector<LayerId>;
  //! Underlying mesh type for lowest layer
  using Mesh = pcl::PolygonMesh;
  //! Mesh edge container type
  using MeshEdges = std::map<size_t, MeshEdge>;

  /**
   * @brief Construct the scene graph (with a default layer factory)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  explicit DynamicSceneGraph(LayerId mesh_layer_id = 0);

  /**
   * @brief Construct the scene graph (with a provided layer factory)
   * @param factor List of layer ids (not including the mesh)
   * @param mesh_layer_id Mesh layer id (must be unique)
   */
  DynamicSceneGraph(const LayerIds& factory, LayerId mesh_layer_id = 0);

  /**
   * @brief Default destructor
   */
  virtual ~DynamicSceneGraph() = default;

  /**
   * @brief Delete all layers and edges
   */
  virtual void clear() override;

  /**
   * @brief Update the pointer to the mesh
   *
   * This removes any edges that point to vertices that no longer
   * exist (i.e. if the new mesh is smaller than the old mesh)
   *
   * @param mesh Mesh to use
   * @param invalidate_all_edges Clear all existing mesh edges
   */
  void setMesh(const Mesh::ConstPtr& mesh, bool invalidate_all_edges = false);

  /**
   * @brief Check whether the layer exists and is valid
   * @param layer_id Layer id to check
   * @returns Returns true if the layer exists and is valid
   */
  virtual bool hasLayer(LayerId layer_id) const override;

  /**
   * @brief Check if a given layer exists
   *
   * Convenience function to handle automatically casting an enum
   * to LayerId
   *
   * @param layer_id Layer to check for
   * @returns Returns true if the given layer exists
   */
  template <typename E, std::enable_if_t<std::is_enum<E>::value, bool> = true>
  bool hasLayer(E e) const {
    static_assert(std::is_same<typename std::underlying_type<E>::type, LayerId>::value,
                  "type passed must be compatible with layer id");
    return hasLayer(static_cast<LayerId>(e));
  }

  /**
   * @brief Remove a node from the graph
   * @param Node node to remove
   * @returns Returns true if the removal was successful
   */
  virtual bool removeNode(NodeId node) override;

  /**
   * @brief Add an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @returns Returns true if edge would be valid and was added
   */
  bool insertMeshEdge(NodeId source, size_t mesh_vertex);

  /**
   * @brief Remove an edge from another node to the mesh
   * @param source Source node id in the scene graph
   * @param mesh_vertex Target mesh vertex index
   * @returns Returns true if edge was removed
   */
  bool removeMeshEdge(NodeId source, size_t mesh_vertex);

  virtual size_t numLayers() const override;

  virtual size_t numNodes() const override;

  virtual size_t numEdges() const override;

  //! Layer id that the mesh uses
  const LayerId mesh_layer_id;

  inline pcl::PolygonMesh::ConstPtr getMesh() const { return mesh_; }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMeshCloudForNode(NodeId node) const;

 protected:
  bool hasMeshEdge(NodeId source, size_t mesh_vertex) const;

  void clearMeshEdges();

  Mesh::ConstPtr mesh_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_vertices_;

  size_t next_mesh_edge_idx_;
  MeshEdges mesh_edges_;
  std::map<NodeId, std::map<size_t, size_t>> mesh_edges_node_lookup_;
  std::map<size_t, std::map<NodeId, size_t>> mesh_edges_vertex_lookup_;
};

}  // namespace kimera
