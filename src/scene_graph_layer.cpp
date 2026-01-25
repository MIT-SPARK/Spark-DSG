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
#include "spark_dsg/scene_graph_layer.h"

#include <sstream>

#include "spark_dsg/graph_utilities.h"
#include "spark_dsg/node_symbol.h"
#include "spark_dsg/printing.h"
#include "spark_dsg/scene_graph.h"

namespace spark_dsg {

using Node = SceneGraphNode;
using Edge = SceneGraphEdge;

SceneGraphLayer::SceneGraphLayer(SceneGraph& graph, LayerKey layer_id)
    : id(layer_id), graph_(graph) {}

bool SceneGraphLayer::hasNode(NodeId node_id) const { return nodes_.count(node_id); }

NodeStatus SceneGraphLayer::checkNode(NodeId node_id) const {
  return nodes_.count(node_id) ? graph_.checkNode(node_id) : NodeStatus::NONEXISTENT;
}

const Node* SceneGraphLayer::findNode(NodeId node_id) const {
  return nodes_.count(node_id) ? graph_.findNode(node_id) : nullptr;
}

const SceneGraphNode& SceneGraphLayer::getNode(NodeId node_id) const {
  const auto node = findNode(node_id);
  if (!node) {
    throw std::out_of_range("missing node '" + NodeSymbol(node_id).str() + "'");
  }

  return *node;
}

bool SceneGraphLayer::hasEdge(NodeId source, NodeId target) const {
  return edges_.contains(source, target);
}

const Edge* SceneGraphLayer::findEdge(NodeId source, NodeId target) const {
  return edges_.find(source, target);
}

const SceneGraphEdge& SceneGraphLayer::getEdge(NodeId source, NodeId target) const {
  const auto edge = findEdge(source, target);
  if (!edge) {
    std::stringstream ss;
    ss << "Missing edge '" << EdgeKey(source, target) << "'";
    throw std::out_of_range(ss.str());
  }

  return *edge;
}

namespace graph_utilities {

using LayerGraphTraits = graph_traits<SceneGraphLayer>;

std::set<NodeId> LayerGraphTraits::neighbors(const SceneGraphLayer& graph,
                                             NodeId node) {
  return get_node(graph, node).siblings();
}

bool LayerGraphTraits::contains(const SceneGraphLayer& graph, NodeId node) {
  return graph.hasNode(node);
}

const SceneGraphLayer::Nodes& LayerGraphTraits::nodes(const SceneGraphLayer& graph) {
  return graph.nodes();
}

const SceneGraphNode& LayerGraphTraits::unwrap_node(
    const SceneGraphLayer::Nodes::value_type& container) {
  return *container.second;
}

NodeId LayerGraphTraits::unwrap_node_id(
    const SceneGraphLayer::Nodes::value_type& container) {
  return container.first;
}

const SceneGraphNode& LayerGraphTraits::get_node(const SceneGraphLayer& graph,
                                                 NodeId node_id) {
  return graph.getNode(node_id);
}

const SceneGraphEdge& LayerGraphTraits::get_edge(const SceneGraphLayer& graph,
                                                 NodeId source,
                                                 NodeId target) {
  return graph.getEdge(source, target);
}

}  // namespace graph_utilities
}  // namespace spark_dsg
