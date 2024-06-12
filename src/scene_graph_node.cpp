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
#include "spark_dsg/scene_graph_node.h"

#include "spark_dsg/node_attributes.h"
#include "spark_dsg/node_symbol.h"

namespace spark_dsg {

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerId layer_id,
                               NodeAttributes::Ptr&& attrs)
    : id(node_id), layer(layer_id), attributes_(std::move(attrs)) {}

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerId layer_id,
                               std::chrono::nanoseconds timestamp,
                               NodeAttributes::Ptr&& attrs)
    : id(node_id),
      layer(layer_id),
      timestamp(timestamp),
      attributes_(std::move(attrs)) {}

SceneGraphNode::~SceneGraphNode() = default;

std::ostream& SceneGraphNode::fill_ostream(std::ostream& out) const {
  out << "Node<id=" << NodeSymbol(id).getLabel() << ", layer=" << layer;
  if (timestamp) {
    out << ", timestamp=" << timestamp->count() << "[ns]";
  }
  out << ">";
  return out;
}

std::ostream& operator<<(std::ostream& out, const SceneGraphNode& node) {
  return node.fill_ostream(out);
}

bool SceneGraphNode::hasParent() const { return parents_.size() == 1; }

bool SceneGraphNode::hasSiblings() const { return not siblings_.empty(); }

bool SceneGraphNode::hasChildren() const { return not children_.empty(); }

std::optional<NodeId> SceneGraphNode::getParent() const {
  if (parents_.size() != 1) {
    return std::nullopt;
  }

  return *parents_.begin();
}

NodeAttributes* SceneGraphNode::getAttributesPtr() const { return attributes_.get(); }

const std::set<NodeId>& SceneGraphNode::siblings() const { return siblings_; };

const std::set<NodeId>& SceneGraphNode::children() const { return children_; };

const std::set<NodeId>& SceneGraphNode::parents() const { return parents_; };

std::vector<NodeId> SceneGraphNode::connections() const {
  // TODO(nathan) this would be better as a custom iterator
  std::vector<NodeId> to_return;
  to_return.insert(to_return.end(), siblings_.begin(), siblings_.end());
  to_return.insert(to_return.end(), children_.begin(), children_.end());
  to_return.insert(to_return.end(), parents_.begin(), parents_.end());
  return to_return;
}

}  // namespace spark_dsg
