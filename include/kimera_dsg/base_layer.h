#pragma once
#include "kimera_dsg/edge_attributes.h"
#include "kimera_dsg/edge_container.h"
#include "kimera_dsg/scene_graph_node.h"

#include <optional>

namespace kimera {

class BaseLayer {
 public:
  //! Static node reference
  using NodeRef = std::reference_wrapper<const SceneGraphNode>;
  //! Dynamic node reference
  using DynamicNodeRef = std::reference_wrapper<const DynamicSceneGraphNode>;
  //! alias to the layer edge reference type
  using EdgeRef = std::reference_wrapper<const SceneGraphEdge>;

  friend class DynamicSceneGraph;

  virtual ~BaseLayer() = default;

  virtual bool hasEdge(NodeId source, NodeId target) const = 0;

  virtual bool removeEdge(NodeId source, NodeId target) = 0;

  virtual bool insertEdge(NodeId source, NodeId target, EdgeAttributes::Ptr&& info) = 0;

  virtual std::optional<EdgeRef> getEdge(NodeId source, NodeId target) const = 0;

  /**
   * @brief Get node ids of newly inserted nodes
   */
  virtual void getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) = 0;

  /**
   * @brief Get node id of deleted nodes
   */
  virtual void getRemovedNodes(std::vector<NodeId>& removed_nodes,
                               bool clear_removed) = 0;

  /**
   * @brief Get the source and target of newly inserted edges
   */
  virtual void getNewEdges(std::vector<EdgeKey>& new_edges, bool clear_new) = 0;

  /**
   * @brief Get the source and target of deleted edges
   */
  virtual void getRemovedEdges(std::vector<EdgeKey>& removed_edges,
                               bool clear_removed) = 0;

 protected:
  virtual EdgeContainer& edgeContainer() = 0;

  virtual bool removeNode(NodeId node_id) = 0;
};

}  // namespace kimera
