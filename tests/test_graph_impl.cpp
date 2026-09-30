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
#include <gtest/gtest.h>

#include "spark_dsg/graph_impl.h"

namespace spark_dsg {

// Test that an empty graph has no nodes and edges
TEST(GraphImpl, DefaultInvariants) {
  GraphImpl graph;
  EXPECT_EQ(0u, graph.num_nodes());
  EXPECT_EQ(0u, graph.num_edges());
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(GraphImpl, EmplaceNodeInvariants) {
  GraphImpl graph;
  EXPECT_EQ(0u, graph.num_nodes());
  EXPECT_FALSE(graph.has(0));
  EXPECT_EQ(NodeStatus::NONEXISTENT, graph.status(0));

  EXPECT_TRUE(graph.emplace(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, graph.num_nodes());
  EXPECT_TRUE(graph.has(0));
  EXPECT_EQ(NodeStatus::NEW, graph.status(0));

  auto node_opt = graph.find(0);
  ASSERT_TRUE(node_opt);
  const auto& node = *node_opt;
  EXPECT_EQ(LayerKey(1), node.layer);
  EXPECT_EQ(0u, node.id);

  // we already have this node, so we should fail
  EXPECT_FALSE(graph.emplace(1, 0, std::make_unique<NodeAttributes>()));
}

// Test that we only have edges that we add, and that edges added respect:
//   - That the source and target must exist
//   - That the edge must not already exist
//   - That edges are bidirectional
TEST(GraphImpl, InsertEdgeInvariants) {
  GraphImpl graph;
  EXPECT_EQ(0u, graph.num_edges());
  EXPECT_FALSE(graph.has(0, 1));

  // source node
  EXPECT_TRUE(graph.emplace(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(graph.has(0, 1));
  EXPECT_FALSE(graph.connect(0, 1));

  // target node
  EXPECT_TRUE(graph.emplace(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(graph.has(0, 1));

  // actually add the edge
  EXPECT_TRUE(graph.connect(0, 1));
  EXPECT_TRUE(graph.has(0, 1));
  EXPECT_TRUE(graph.has(1, 0));
  EXPECT_EQ(1u, graph.num_edges());

  // add a duplicate
  EXPECT_FALSE(graph.connect(0, 1));
  EXPECT_TRUE(graph.has(0, 1));
  EXPECT_TRUE(graph.has(1, 0));
  EXPECT_EQ(1u, graph.num_edges());
}

// Test that inserting specific edge attributes works
TEST(GraphImpl, EdgeAttributesCorrect) {
  GraphImpl graph;
  // source and target nodes
  EXPECT_TRUE(graph.emplace(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplace(1, 1, std::make_unique<NodeAttributes>()));

  // actually add the edge
  auto info = std::make_unique<EdgeAttributes>();
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(graph.connect(0, 1, std::move(info)));
  EXPECT_TRUE(graph.has(0, 1));
  EXPECT_EQ(1u, graph.num_edges());

  auto edge_opt = graph.find(0, 1);
  ASSERT_TRUE(edge_opt);
  const auto& edge = *edge_opt;
  EXPECT_EQ(0u, edge.source);
  EXPECT_EQ(1u, edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  auto swapped_edge_opt = graph.find(1, 0);
  ASSERT_TRUE(swapped_edge_opt);

  const auto& swapped_edge = *swapped_edge_opt;
  // note that accessing an edge from the reverse direction
  // that is was added doesn't change the info and keeps the
  // source and target the same as how the edge was added
  EXPECT_EQ(0u, swapped_edge.source);
  EXPECT_EQ(1u, swapped_edge.target);
  ASSERT_TRUE(swapped_edge.info != nullptr);
  EXPECT_TRUE(swapped_edge.info->weighted);
  EXPECT_EQ(0.5, swapped_edge.info->weight);
}

// Test that nodes we see via the public iterator match up with what we added
TEST(GraphImpl, BasicNodeIterationCorrect) {
  GraphImpl graph;
  size_t num_nodes = 5;
  std::set<int64_t> expected_ids;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.emplace(1, i, std::make_unique<NodeAttributes>()));
    expected_ids.insert(i);
  }

  EXPECT_EQ(5u, graph.num_nodes());

  // nodes may be stored unordered in the future
  std::set<int64_t> actual_ids;
  for (const auto& node : graph.nodes()) {
    actual_ids.insert(node.id);
  }

  EXPECT_EQ(expected_ids, actual_ids);
}

// Test that edges we see via the public iterator match up with what we added
TEST(GraphImpl, BasicEdgeIterationCorrect) {
  GraphImpl graph;
  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.emplace(1, i, std::make_unique<NodeAttributes>()));
  }

  std::set<NodeId> expected_targets;
  for (size_t i = 1; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.connect(i - 1, i));
    expected_targets.insert(i);
  }

  // nodes may be stored unordered in the future
  std::set<NodeId> actual_targets;
  for (const auto& edge : graph.edges()) {
    ASSERT_TRUE(edge.info != nullptr);
    EXPECT_EQ(edge.source + 1, edge.target);
    actual_targets.insert(edge.target);
  }

  EXPECT_EQ(expected_targets, actual_targets);
}

// Test that removing a node meets the invariants that we expect
//   - we don't do anything if it doesn't exist
//   - we remove all edges related to the node if it does
TEST(GraphImpl, RemoveNodeSound) {
  GraphImpl graph;

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.remove(0));

  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.emplace(1, i, std::make_unique<NodeAttributes>()));
  }

  for (size_t i = 1; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.connect(0, i));
  }

  EXPECT_EQ(num_nodes, graph.num_nodes());
  EXPECT_EQ(num_nodes - 1, graph.num_edges());
  graph.remove(0);
  EXPECT_EQ(num_nodes - 1, graph.num_nodes());
  EXPECT_EQ(0u, graph.num_edges());
  EXPECT_EQ(NodeStatus::DELETED, graph.status(0));
}

// Test that merging two nodes meets the invariants that we expect
//   - we don't do anything if either nodes doesn't exist
//   - we rewire all edges to the merged nodes if we do
TEST(GraphImpl, ContractCorrect) {
  GraphImpl graph;

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.contract(0, 1));

  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(graph.emplace(1, i, std::make_unique<NodeAttributes>()));
  }

  EXPECT_FALSE(graph.contract(0, 5));

  graph.remove(4);
  EXPECT_FALSE(graph.contract(4, 0));

  for (size_t i = 1; i < 4; ++i) {
    EXPECT_TRUE(graph.connect(0, i));
  }

  ASSERT_TRUE(graph.has(0));
  ASSERT_TRUE(graph.has(1));
  EXPECT_EQ(4u, graph.num_nodes());
  EXPECT_EQ(3u, graph.num_edges());
  EXPECT_EQ(3u, graph.get(0).siblings().size());
  EXPECT_EQ(1u, graph.get(1).siblings().size());

  EXPECT_TRUE(graph.contract(0, 1));
  EXPECT_FALSE(graph.has(0));
  ASSERT_TRUE(graph.has(1));
  EXPECT_EQ(3u, graph.num_nodes());
  EXPECT_EQ(2u, graph.num_edges());
  EXPECT_EQ(2u, graph.get(1).siblings().size());
  EXPECT_EQ(NodeStatus::MERGED, graph.status(0));
  EXPECT_EQ(NodeStatus::DELETED, graph.status(4));
}

// Test that removing a edge does what it should
TEST(GraphImpl, RemoveEdgeCorrect) {
  GraphImpl graph;

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.remove(0, 1));
  EXPECT_TRUE(graph.emplace(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplace(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.connect(0, 1));

  EXPECT_EQ(1u, graph.num_edges());
  EXPECT_TRUE(graph.remove(0, 1));
  EXPECT_EQ(0u, graph.num_edges());

  for (const auto& node : graph.nodes()) {
    EXPECT_FALSE(node.hasSiblings());
  }
}

TEST(GraphImpl, CloneCorrect) {
  GraphImpl graph;

  graph.emplace(1, 0, std::make_unique<NodeAttributes>());
  for (size_t i = 1; i < 5; ++i) {
    graph.emplace(1, i, std::make_unique<NodeAttributes>());
    graph.connect(i - 1, i);
  }

  auto result = graph.clone();
  ASSERT_TRUE(result != nullptr);
  for (const auto& node : graph.nodes()) {
    EXPECT_TRUE(result->has(node.id));
  }

  for (const auto& edge : graph.edges()) {
    EXPECT_TRUE(result->has(edge.source, edge.target));
  }
}

}  // namespace spark_dsg
