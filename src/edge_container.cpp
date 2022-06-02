#include "kimera_dsg/edge_container.h"

namespace kimera {

SceneGraphEdge::SceneGraphEdge(NodeId source, NodeId target, AttrPtr&& info)
    : source(source), target(target), info(std::move(info)) {}

SceneGraphEdge::~SceneGraphEdge() = default;

using Edge = EdgeContainer::Edge;

void EdgeContainer::insert(NodeId source,
                           NodeId target,
                           EdgeAttributes::Ptr&& edge_info) {
  auto attrs = (edge_info == nullptr) ? std::make_unique<EdgeAttributes>()
                                      : std::move(edge_info);

  edges.emplace(std::piecewise_construct,
                std::forward_as_tuple(source, target),
                std::forward_as_tuple(source, target, std::move(attrs)));
  edge_status[EdgeKey(source, target)] = EdgeStatus::NEW;
}

void EdgeContainer::remove(NodeId source, NodeId target) {
  const EdgeKey key(source, target);
  edge_status.at(key) = EdgeStatus::DELETED;
  edges.erase(key);
}

bool EdgeContainer::contains(NodeId source, NodeId target) const {
  return edges.count(EdgeKey(source, target));
}

void EdgeContainer::reset() {
  edges.clear();
  edge_status.clear();
}

void EdgeContainer::rewire(NodeId source,
                           NodeId target,
                           NodeId new_source,
                           NodeId new_target) {
  auto attrs = get(source, target).info->clone();
  edge_status.at(EdgeKey(source, target)) = EdgeStatus::MERGED;
  remove(source, target);
  insert(new_source, new_target, std::move(attrs));
}

EdgeStatus EdgeContainer::getStatus(NodeId source, NodeId target) const {
  const auto iter = edge_status.find(EdgeKey(source, target));
  return (iter == edge_status.end()) ? EdgeStatus::NONEXISTENT : iter->second;
}

void EdgeContainer::getNew(std::vector<EdgeKey>& new_edges, bool clear_new) {
  auto iter = edge_status.begin();
  while (iter != edge_status.end()) {
    if (iter->second == EdgeStatus::NEW) {
      new_edges.push_back(iter->first);
      if (clear_new) {
        iter->second = EdgeStatus::VISIBLE;
      }
    }

    ++iter;
  }
}

void EdgeContainer::getRemoved(std::vector<EdgeKey>& removed_edges,
                               bool clear_removed) {
  auto iter = edge_status.begin();
  while (iter != edge_status.end()) {
    if (iter->second != EdgeStatus::DELETED) {
      ++iter;
      continue;
    }

    removed_edges.push_back(iter->first);
    iter = clear_removed ? edge_status.erase(iter) : ++iter;
  }
}

}  // namespace kimera
