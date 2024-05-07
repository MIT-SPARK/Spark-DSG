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
#include <map>
#include <vector>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/scene_graph_types.h"

namespace spark_dsg {

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

  /**
   * @brief get a reference to the attributes of the node (with an optional
   * template argument to perform a cast to the desired attribute type
   */
  template <typename Derived = EdgeAttributes>
  Derived& attributes() const {
    static_assert(std::is_base_of<EdgeAttributes, Derived>::value,
                  "info can only be downcast to a derived EdgeAttributes class");
    return dynamic_cast<Derived&>(*info);
  }
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

  size_t size() const;

  void reset();

  Edge* find(NodeId source, NodeId target);

  const Edge* find(NodeId source, NodeId target) const;

  EdgeStatus getStatus(NodeId source, NodeId target) const;

  void getRemoved(std::vector<EdgeKey>& removed_edges, bool clear_removed);

  void getNew(std::vector<EdgeKey>& new_edges, bool clear_new);

  void setStale();

  Edges edges;
  EdgeStatusMap edge_status;
  mutable std::map<EdgeKey, bool> stale_edges;

 protected:
  Edge* find(const EdgeKey& key) const;
};

}  // namespace spark_dsg
