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

 protected:
  virtual bool removeNode(NodeId node_id) = 0;
};

}  // namespace kimera
