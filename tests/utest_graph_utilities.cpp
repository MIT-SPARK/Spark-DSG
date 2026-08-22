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
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph_layer.h>

namespace spark_dsg {

using NodeSet = std::set<NodeId>;
using ResultSet = std::vector<NodeSet>;
using graph_utilities::Components;
using graph_utilities::getConnectedComponents;

template <typename Expected, typename Result>
bool matchesExpected(const Expected& expected, const Result& result) {
  if (expected.size() != result.size()) {
    return false;
  }

  for (const NodeId id : result) {
    if (!expected.count(id)) {
      return false;
    }
  }

  return true;
}

template <typename Expected, typename Result>
bool matchesExpectedSet(const Expected& expected,
                        const std::vector<Result>& result_set) {
  for (const auto& result : result_set) {
    if (matchesExpected(expected, result)) {
      return true;
    }
  }

  return false;
}

struct GraphConfig {
  size_t num_nodes;
  std::vector<std::pair<NodeId, NodeId>> edges;
};

struct ConnectedComponentTestConfig {
  NodeSet query;
  ResultSet expected;
  bool restrict_to_query;
  bool connected_graph;
};

struct ConnectedComponentFixture
    : public testing::TestWithParam<ConnectedComponentTestConfig> {
  ConnectedComponentFixture() : layer(1) {}

  virtual ~ConnectedComponentFixture() = default;

  void SetUp() override {
    for (size_t i = 0; i < 6; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    layer.insertEdge(0, 1);
    layer.insertEdge(0, 2);
    layer.insertEdge(1, 2);
    layer.insertEdge(3, 4);
    layer.insertEdge(3, 5);
    layer.insertEdge(4, 5);
  }

  SceneGraphLayer layer;
};

struct DepthLimitedCCTestConfig {
  NodeSet query;
  ResultSet expected;
  size_t depth;
  GraphConfig graph;
};

struct DepthLimitedCCFixture : public testing::TestWithParam<DepthLimitedCCTestConfig> {
  DepthLimitedCCFixture() : layer(1) {}

  virtual ~DepthLimitedCCFixture() = default;

  void SetUp() override {
    DepthLimitedCCTestConfig config = GetParam();
    for (size_t i = 0; i < config.graph.num_nodes; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    for (const auto& edge : config.graph.edges) {
      layer.insertEdge(edge.first, edge.second);
    }
  }

  SceneGraphLayer layer;
};

struct FilteredCCTestConfig {
  ResultSet expected;
  NodeSet disallowed_nodes;
  NodeSet disallowed_edge_nodes;
  GraphConfig graph;
};

struct FilteredCCFixture : public testing::TestWithParam<FilteredCCTestConfig> {
  FilteredCCFixture() : layer(1) {}

  virtual ~FilteredCCFixture() = default;

  void SetUp() override {
    FilteredCCTestConfig config = GetParam();
    for (size_t i = 0; i < config.graph.num_nodes; ++i) {
      layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    }

    for (const auto& edge : config.graph.edges) {
      layer.insertEdge(edge.first, edge.second);
    }
  }

  SceneGraphLayer layer;
};

const GraphConfig dl_graphs[] = {
    {6, {{0, 1}, {1, 2}, {3, 4}, {3, 5}, {4, 5}}},
    {7, {{0, 1}, {1, 2}, {3, 4}, {3, 5}, {4, 5}}},
    {15,
     {{0, 7},
      {0, 2},
      {0, 5},
      {0, 14},
      {2, 14},
      {3, 13},
      {3, 8},
      {4, 10},
      {4, 13},
      {5, 9},
      {6, 9},
      {9, 12},
      {11, 14},
      {12, 13}}},
};

const ConnectedComponentTestConfig cc_test_cases[] = {
    {NodeSet{0}, ResultSet{{0, 1, 2}}, false, false},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, false, false},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2, 3, 4, 5}}, false, true},
    {NodeSet{0}, ResultSet{{0}}, true, false},
    {NodeSet{0, 3}, ResultSet{{0}, {3}}, true, false},
    {NodeSet{0, 1, 2, 3, 4, 5}, ResultSet{{0, 1, 2}, {3, 4, 5}}, true, false},
    {NodeSet{0, 2, 3, 5}, ResultSet{{0, 2, 3, 5}}, true, true},
    {NodeSet{0, 1, 4, 5}, ResultSet{{0, 1}, {4, 5}}, true, true},
    {NodeSet{0, 1, 2, 3, 4, 5}, ResultSet{{0, 1, 2, 3, 4, 5}}, true, true},
};

const DepthLimitedCCTestConfig dl_test_cases[] = {
    {NodeSet{0}, ResultSet{{0, 1}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{1, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 2, dl_graphs[0]},
    {NodeSet{0, 1, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{0, 2, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[0]},
    {NodeSet{6}, ResultSet{{6}}, 1, dl_graphs[1]},
    {NodeSet{0, 2, 3}, ResultSet{{0, 1, 2}, {3, 4, 5}}, 1, dl_graphs[1]},
    {NodeSet{0, 1, 2, 3, 4, 5, 6},
     ResultSet{{0, 1, 2}, {3, 4, 5}, {6}},
     1,
     dl_graphs[1]},
    {NodeSet{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14},
     ResultSet{{0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}, {1}},
     3,
     dl_graphs[2]},
};

const FilteredCCTestConfig filtered_cc_test_cases[] = {
    {ResultSet{{0, 1, 2}, {3, 4, 5}}, {}, {}, dl_graphs[0]},
    {ResultSet{{0, 1}, {3}, {4, 5}}, {2}, {3}, dl_graphs[0]},
    {ResultSet{{0}, {1}, {2}, {3}, {4}, {5}}, {}, {0, 1, 2, 3, 4, 5}, dl_graphs[0]},
};

TEST(GraphUtilities, NormalEmptyCorrect) {
  SceneGraphLayer layer(1);

  NodeSet query;
  auto result = getConnectedComponents(layer, query, false);
  EXPECT_TRUE(result.empty());

  query.insert(0);
  query.insert(1);
  result = getConnectedComponents(layer, query, false);
  EXPECT_TRUE(result.empty());
}

TEST(GraphUtilities, FilteredEmptyCorrect) {
  SceneGraphLayer layer(1);

  NodeSet query;
  const auto result = getConnectedComponents(
      layer, [](const auto&) { return true; }, [](const auto&) { return true; });
  EXPECT_TRUE(result.empty());
}

TEST(GraphUtilities, ShortestPathCorrect) {
  SceneGraphLayer layer(1);

  {  // empty graph means no result
    const auto result = graph_utilities::shortestPath(layer, 0, 1);
    EXPECT_TRUE(result.empty());
  }

  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 5; ++x) {
      const auto idx = 5 * y + x;
      auto attrs = std::make_unique<NodeAttributes>();
      // force non-uniform cost
      attrs->position << x * x, 1.5 * y * y, 1.0;
      layer.emplaceNode(idx, std::move(attrs));
      if (x % 5) {
        // add edge to previous node as long as not first node in row
        layer.insertEdge(idx, 5 * y + x - 1);
      }

      if (y > 0) {
        // add edge to previous row as long as not first row
        layer.insertEdge(idx, 5 * (y - 1) + x);
      }
    }
  }

  {  // planning to the same node means no result
    const auto result = graph_utilities::shortestPath(layer, 5, 5);
    EXPECT_TRUE(result.empty());
  }

  {  // planning along edge is straightforward
    const auto result = graph_utilities::shortestPath(layer, 0, 4);
    const std::vector<NodeId> expected = {0, 1, 2, 3, 4};
    EXPECT_EQ(result, expected);
  }

  {  // planning corner to corner works as expected
    // non-uniform positions means that y is preferred over x
    const auto result = graph_utilities::shortestPath(layer, 0, 24);
    const std::vector<NodeId> expected = {0, 5, 10, 15, 20, 21, 22, 23, 24};
    EXPECT_EQ(result, expected);
  }
}

TEST_P(ConnectedComponentFixture, ConnectedComponentsCorrect) {
  ConnectedComponentTestConfig info = GetParam();
  if (info.connected_graph) {
    layer.insertEdge(2, 3);
  }

  Components result = getConnectedComponents(layer, info.query, info.restrict_to_query);
  EXPECT_EQ(info.expected.size(), result.size());
  for (const auto& expected : info.expected) {
    EXPECT_TRUE(matchesExpectedSet(expected, result))
        << displayNodeSymbolContainer(expected);
  }
}

TEST_P(DepthLimitedCCFixture, ConnectedComponentsCorrect) {
  DepthLimitedCCTestConfig info = GetParam();

  Components result = getConnectedComponents(layer, info.depth, info.query);

  EXPECT_EQ(info.expected.size(), result.size());
  for (const auto& expected : info.expected) {
    EXPECT_TRUE(matchesExpectedSet(expected, result))
        << displayNodeSymbolContainer(expected);
  }
}

TEST_P(FilteredCCFixture, ConnectedComponentsCorrect) {
  FilteredCCTestConfig info = GetParam();

  const auto result = getConnectedComponents(
      layer,
      [&](const SceneGraphNode& node) { return !info.disallowed_nodes.count(node.id); },
      [&](const SceneGraphEdge& edge) {
        return !info.disallowed_edge_nodes.count(edge.source) &&
               !info.disallowed_edge_nodes.count(edge.target);
      });

  EXPECT_EQ(info.expected.size(), result.size());
  for (const auto& expected : info.expected) {
    EXPECT_TRUE(matchesExpectedSet(expected, result))
        << displayNodeSymbolContainer(expected);
  }
}

INSTANTIATE_TEST_SUITE_P(GraphUtilities,
                         ConnectedComponentFixture,
                         testing::ValuesIn(cc_test_cases));

INSTANTIATE_TEST_SUITE_P(GraphUtilities,
                         DepthLimitedCCFixture,
                         testing::ValuesIn(dl_test_cases));

INSTANTIATE_TEST_SUITE_P(GraphUtilities,
                         FilteredCCFixture,
                         testing::ValuesIn(filtered_cc_test_cases));

}  // namespace spark_dsg
