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
#include "spark_dsg/node_storage.h"

#include "spark_dsg/node_symbol.h"
#include "spark_dsg/printing.h"

namespace spark_dsg {

auto NodeStorage::check(NodeId node_id) const -> Status {
  auto iter = nodes_status_.find(node_id);
  return iter != nodes_status_.end() ? iter->second : Status::NONEXISTENT;
}

auto NodeStorage::find(NodeId node_id) const -> const Node* {
  auto iter = nodes_.find(node_id);
  return iter == nodes_.end() ? nullptr : iter->second.get();
}

auto NodeStorage::at(NodeId node_id) const -> const Node& {
  const auto node = find(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
  }

  return *node;
}

bool NodeStorage::emplace(LayerKey layer,
                          NodeId node_id,
                          std::unique_ptr<NodeAttributes>&& attrs) {
  nodes_status_[node_id] = Status::NEW;
  return nodes_
      .emplace(node_id, std::make_unique<Node>(node_id, layer, std::move(attrs)))
      .second;
}

bool NodeStorage::remove(NodeId node_id) {
  auto iter = nodes_.find(node_id);
  if (iter == nodes_.end()) {
    return false;
  }

  // remove the actual node
  nodes_.erase(iter);
  nodes_status_[node_id] = Status::DELETED;
  return true;
}

void NodeStorage::getNewNodes(std::vector<NodeId>& new_nodes, bool clear_new) const {
  auto iter = nodes_status_.begin();
  while (iter != nodes_status_.end()) {
    if (iter->second == Status::NEW) {
      new_nodes.push_back(iter->first);
      if (clear_new) {
        iter->second = Status::VISIBLE;
      }
    }

    ++iter;
  }
}

void NodeStorage::getRemovedNodes(std::vector<NodeId>& removed_nodes,
                                  bool clear_removed) const {
  auto iter = nodes_status_.begin();
  while (iter != nodes_status_.end()) {
    if (iter->second != Status::DELETED && iter->second != Status::MERGED) {
      ++iter;
      continue;
    }

    removed_nodes.push_back(iter->first);

    if (clear_removed && iter->second == Status::DELETED) {
      iter = nodes_status_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void NodeStorage::reset() {
  nodes_.clear();
  nodes_status_.clear();
}

std::shared_ptr<NodeStorage> NodeStorage::clone(const NodeChecker& is_valid) const {
  auto new_storage = std::make_shared<NodeStorage>();
  for (const auto& [id, node] : nodes_) {
    if (is_valid && !is_valid(*node)) {
      continue;
    }

    new_storage->emplace(node->layer, id, node->attributes().clone());
  }

  return new_storage;
}

void NodeStorage::transform(const Eigen::Isometry3d& transform) {
  for (auto&& [id, node] : nodes_) {
    node->attributes().transform(transform);
  }
}

size_t NodeStorage::memoryUsage() const {
  size_t total_memory = sizeof(*this);

  // Estimate memory usage of nodes.
  total_memory += nodes_.size() * (sizeof(NodeId) + sizeof(Node::Ptr));
  for (const auto& [node_id, node] : nodes_) {
    total_memory += node->memoryUsage();
  }

  // Estimate memory usage of nodes status map.
  total_memory += nodes_status_.size() * (sizeof(NodeId) + sizeof(Status));
  return total_memory;
}

}  // namespace spark_dsg
