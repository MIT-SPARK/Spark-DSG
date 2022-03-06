#include "kimera_dsg/edge_container.h"

namespace kimera {

SceneGraphEdge::SceneGraphEdge(NodeId source, NodeId target, AttrPtr&& info)
    : source(source), target(target), info(std::move(info)) {}

SceneGraphEdge::~SceneGraphEdge() = default;

using Edge = EdgeContainer::Edge;

void EdgeContainer::insert(NodeId source,
                           NodeId target,
                           EdgeAttributes::Ptr&& edge_info) {
  ++last_idx;
  insert(source, target, std::move(edge_info), last_idx);
}

void EdgeContainer::insert(NodeId source,
                           NodeId target,
                           EdgeAttributes::Ptr&& edge_info,
                           size_t index) {
  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  edges.emplace(std::piecewise_construct,
                std::forward_as_tuple(index),
                std::forward_as_tuple(source, target, std::move(attrs)));
  edges_info[source][target] = index;
  edges_info[target][source] = index;
}

void EdgeContainer::remove(NodeId source, NodeId target) {
  edges.erase(edges_info.at(source).at(target));

  edges_info.at(source).erase(target);
  if (edges_info.at(source).empty()) {
    edges_info.erase(source);
  }

  edges_info.at(target).erase(source);
  if (edges_info.at(target).empty()) {
    edges_info.erase(target);
  }
}

bool EdgeContainer::contains(NodeId source, NodeId target) const {
  return edges_info.count(source) && edges_info.at(source).count(target);
}

void EdgeContainer::reset() {
  last_idx = 0;
  edges_info.clear();
  edges.clear();
}

}  // namespace kimera
