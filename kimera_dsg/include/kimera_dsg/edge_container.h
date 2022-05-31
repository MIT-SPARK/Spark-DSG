#pragma once
#include "kimera_dsg/edge_attributes.h"
#include "kimera_dsg/scene_graph_types.h"
#include "kimera_dsg/node_symbol.h"

#include <map>
#include <vector>

namespace kimera {

/**
 * @brief edge status
 *
 * Mostly for keeping history of edges in a graph
 */
enum class EdgeStatus { NEW, VISIBLE, DELETED, MERGED, NONEXISTENT };

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

struct EdgeKey {
  EdgeKey(NodeId k1, NodeId k2) : k1(std::min(k1, k2)), k2(std::max(k1, k2)) {}

  inline bool operator==(const EdgeKey& other) const {
    return k1 == other.k1 && k2 == other.k2;
  }

  inline bool operator<(const EdgeKey& other) const {
    if (k1 == other.k1) {
      return k2 < other.k2;
    }

    return k1 < other.k1;
  }

  NodeId k1;
  NodeId k2;
};

inline std::ostream& operator<<(std::ostream& out, const EdgeKey& key) {
  return out << NodeSymbol(key.k1) << " -> " << NodeSymbol(key.k2);
}

struct EdgeContainer {
  using Edge = SceneGraphEdge;
  using Edges = std::map<EdgeKey, Edge>;
  using EdgeStatusMap = std::map<EdgeKey, EdgeStatus>;

  void insert(NodeId source, NodeId target, EdgeAttributes::Ptr&& edge_info);

  void remove(NodeId source, NodeId target);

  void rewire(NodeId source, NodeId target, NodeId new_source, NodeId new_target);

  bool contains(NodeId source, NodeId target) const;

  inline size_t size() const { return edges.size(); }

  void reset();

  inline const Edge& get(NodeId source, NodeId target) const {
    return edges.at(EdgeKey(source, target));
  }

  EdgeStatus getStatus(NodeId source, NodeId target) const;

  void getRemoved(std::vector<EdgeKey>& removed_edges, bool clear_removed);

  void getNew(std::vector<EdgeKey>& new_edges, bool clear_new);

  Edges edges;
  EdgeStatusMap edge_status;
};

}  // namespace kimera
