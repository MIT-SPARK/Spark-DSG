#pragma once
#include "kimera_dsg/node_symbol.h"
#include "kimera_dsg/scene_graph_layer.h"
#include "kimera_dsg/scene_graph_node.h"

#include <chrono>

namespace kimera {

class DynamicSceneGraphNode : public SceneGraphNode {
 public:
  friend class DynamicSceneGraphLayer;
  using Ptr = std::unique_ptr<DynamicSceneGraphNode>;

  DynamicSceneGraphNode(NodeId id,
                        LayerId layer,
                        NodeAttributes::Ptr&& attrs,
                        std::chrono::nanoseconds timestamp)
      : SceneGraphNode(id, layer, std::move(attrs)), timestamp(timestamp) {}

  virtual ~DynamicSceneGraphNode() = default;

  DynamicSceneGraphNode(const DynamicSceneGraphNode& other) = delete;
  DynamicSceneGraphNode& operator=(const DynamicSceneGraphNode& other) = delete;

  const std::chrono::nanoseconds timestamp;
};

class DynamicSceneGraphLayer {
 public:
  //! desired pointer type for the layer
  using Ptr = std::unique_ptr<DynamicSceneGraphLayer>;
  //! node type of the layer
  using Node = DynamicSceneGraphNode;
  //! node container for the layer
  using Nodes = std::vector<Node::Ptr>;
  //! node reference type for the layer
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
  friend class DynamicSceneGraph;

  DynamicSceneGraphLayer(LayerId layer, char node_prefix);

  virtual ~DynamicSceneGraphLayer() = default;

  inline size_t numNodes() const { return nodes_.size(); }

  inline size_t numEdges() const { return edges_.size(); }

  bool hasNode(NodeId node_id) const;

  bool hasNodeByIndex(size_t node_index) const;

  bool hasEdge(NodeId source, NodeId target) const;

  bool hasEdgeByIndex(size_t source_index, size_t target_index) const;

  std::optional<NodeRef> getNode(NodeId node_id) const;

  std::optional<NodeRef> getNodeByIndex(size_t node_id) const;

  std::optional<EdgeRef> getEdge(NodeId source, NodeId target) const;

  std::optional<EdgeRef> getEdgeByIndex(size_t source_index, size_t target_index) const;

  bool insertEdge(NodeId source, NodeId target, EdgeInfo::Ptr&& edge_info = nullptr);

  bool insertEdgeByIndex(size_t source_index,
                         size_t target_index,
                         EdgeInfo::Ptr&& edge_info = nullptr);

  Eigen::Vector3d getPosition(NodeId node) const;

  Eigen::Vector3d getPositionByIndex(size_t node_index) const;

  const LayerId id;

  const char prefix;

 protected:
  bool emplaceNode(std::chrono::nanoseconds timestamp,
                   NodeAttributes::Ptr&& attrs,
                   bool add_edge = true);

 protected:
  std::unordered_set<std::chrono::nanoseconds::rep> times_;

  Nodes nodes_;
  NodeSymbol next_node_;

  //! internal edge index counter
  size_t last_edge_idx_;
  //! internal edge container
  Edges edges_;
  //! internal mapping between node id and edge index
  EdgeLookup edges_info_;

 public:
  /**
   * @brief constant node container
   */
  inline const Nodes& nodes() const { return nodes_; }

  /**
   * @brief constant edge container
   */
  inline const Edges& edges() const { return edges_; };
};

}  // namespace kimera
