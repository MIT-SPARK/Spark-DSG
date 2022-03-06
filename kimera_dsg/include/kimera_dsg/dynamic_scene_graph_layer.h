#pragma once
#include "kimera_dsg/base_layer.h"
#include "kimera_dsg/node_symbol.h"

namespace kimera {

class DynamicSceneGraphLayer : public BaseLayer {
 public:
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<DynamicSceneGraphLayer>;
  //! node type of the layer
  using Node = DynamicSceneGraphNode;
  //! node reference type of the layer
  using NodeRef = BaseLayer::DynamicNodeRef;
  //! node container for the layer
  using Nodes = std::vector<Node::Ptr>;
  //! edge type for the layer
  using Edge = SceneGraphEdge;
  //! edge container type for the layer
  using Edges = std::map<size_t, Edge>;

  friend class DynamicSceneGraph;

  DynamicSceneGraphLayer(LayerId layer, LayerPrefix node_prefix);

  virtual ~DynamicSceneGraphLayer() = default;

  inline size_t numNodes() const { return nodes_.size(); }

  inline size_t numEdges() const { return edges_.size(); }

  bool hasNode(NodeId node_id) const;

  bool hasNodeByIndex(size_t node_index) const;

  bool hasEdge(NodeId source, NodeId target) const override;

  bool hasEdgeByIndex(size_t source_index, size_t target_index) const;

  std::optional<NodeRef> getNode(NodeId node_id) const;

  std::optional<NodeRef> getNodeByIndex(size_t node_id) const;

  std::optional<EdgeRef> getEdge(NodeId source, NodeId target) const override;

  std::optional<EdgeRef> getEdgeByIndex(size_t source_index, size_t target_index) const;

  bool insertEdge(NodeId source,
                  NodeId target,
                  EdgeAttributes::Ptr&& edge_info = nullptr) override;

  bool insertEdgeByIndex(size_t source_index,
                         size_t target_index,
                         EdgeAttributes::Ptr&& edge_info = nullptr);

  bool removeEdge(NodeId source, NodeId target) override;

  bool removeEdgeByIndex(size_t source_index, size_t target_index);

  Eigen::Vector3d getPosition(NodeId node) const;

  Eigen::Vector3d getPositionByIndex(size_t node_index) const;

  const LayerId id;

  const LayerPrefix prefix;

  bool mergeLayer(const DynamicSceneGraphLayer& graph_layer,
                  std::map<NodeId, LayerKey>* layer_lookup = nullptr,
                  bool update_attributes = true);

 protected:
  bool emplaceNode(std::chrono::nanoseconds timestamp,
                   NodeAttributes::Ptr&& attrs,
                   bool add_edge = true);

  bool removeNode(NodeId node) override;

 protected:
  std::set<std::chrono::nanoseconds::rep> times_;

  Nodes nodes_;
  std::map<size_t, NodeStatus> node_status_;
  size_t next_node_;

  //! internal edge container
  EdgeContainer edges_;

 public:
  /**
   * @brief constant node container
   */
  inline const Nodes& nodes() const { return nodes_; }

  /**
   * @brief constant edge container
   */
  inline const Edges& edges() const { return edges_.edges; };
};

}  // namespace kimera
