#pragma once
#include "kimera_dsg/edge_attributes.h"
#include "kimera_dsg/scene_graph_types.h"

#include <map>

namespace kimera {

/**
 * @brief Edge representation
 */
struct SceneGraphEdge {
  //! attributes of the edge
  using AttrPtr = std::unique_ptr<EdgeAttributes>;

  //! construct and edge from some info
  SceneGraphEdge(NodeId source, NodeId target, AttrPtr&& info);

  ~SceneGraphEdge();

  //! start of edge (by convention the parent)
  const NodeId source;
  //! end of edge (by convention the child)
  const NodeId target;
  //! attributes about the edge
  AttrPtr info;
};

struct EdgeContainer {
  using Edge = SceneGraphEdge;
  using Edges = std::map<size_t, Edge>;
  using EdgeLookup = std::map<NodeId, std::map<NodeId, size_t>>;

  void insert(NodeId source, NodeId target, EdgeAttributes::Ptr&& edge_info);

  void insert(NodeId source,
              NodeId target,
              EdgeAttributes::Ptr&& edge_info,
              size_t index);

  void remove(NodeId source, NodeId target);

  bool contains(NodeId source, NodeId target) const;

  inline size_t size() const { return edges.size(); }

  void reset();

  inline size_t getIndex(NodeId source, NodeId target) const {
    return edges_info.at(source).at(target);
  }

  inline const Edge& get(size_t index) const { return edges.at(index); }

  inline const Edge& get(NodeId source, NodeId target) const {
    const size_t index = getIndex(source, target);
    return edges.at(index);
  }

  size_t last_idx = 0;
  Edges edges;
  EdgeLookup edges_info;
};

}  // namespace kimera
