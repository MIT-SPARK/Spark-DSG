#include "kimera_dsg_tests/type_comparisons.h"

#include <kimera_dsg/scene_graph_utilities.h>

#include <gtest/gtest.h>

namespace kimera {

struct AncestorTestConfig {
  NodeId query;
  LayerId query_layer;
  std::vector<NodeId> expected;
};

std::ostream& operator<<(std::ostream& out, const AncestorTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).getLabel()
      << ", layer: " << config.query_layer
      << ", expected: " << displayNodeSymbolContainer(config.expected) << "}";
  return out;
}

struct BoundingBoxTestConfig {
  NodeId query;
  LayerId query_layer;
  BoundingBox expected;
};

std::ostream& operator<<(std::ostream& out, const BoundingBoxTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).getLabel()
      << ", layer: " << config.query_layer << ", expected: " << config.expected << "}";
  return out;
}

template <typename Config>
struct SgUtilitiesFixture : public testing::TestWithParam<Config> {
  SgUtilitiesFixture() {}

  virtual ~SgUtilitiesFixture() = default;

  void SetUp() override {
    graph.emplaceNode(4, 0, std::make_unique<NodeAttributes>());
    graph.emplaceNode(4, 1, std::make_unique<NodeAttributes>());
    graph.emplaceNode(3, 2, std::make_unique<NodeAttributes>());
    graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>());
    graph.emplaceNode(2, 4, std::make_unique<NodeAttributes>(Eigen::Vector3d(1, 1, 1)));
    graph.emplaceNode(2, 5, std::make_unique<NodeAttributes>(Eigen::Vector3d(2, 2, 2)));
    graph.emplaceNode(2, 6, std::make_unique<NodeAttributes>(Eigen::Vector3d(3, 3, 3)));
    graph.emplaceNode(2, 7, std::make_unique<NodeAttributes>(Eigen::Vector3d(4, 4, 4)));
    graph.insertEdge(0, 2);
    graph.insertEdge(1, 3);
    graph.insertEdge(2, 4);
    graph.insertEdge(2, 5);
    graph.insertEdge(3, 6);
    graph.insertEdge(3, 7);
  }

  DynamicSceneGraph graph;
};

using AncestorTestFixture = SgUtilitiesFixture<AncestorTestConfig>;
using BoundingBoxTestFixture = SgUtilitiesFixture<BoundingBoxTestConfig>;

TEST_P(AncestorTestFixture, ResultCorrect) {
  AncestorTestConfig config = GetParam();

  std::vector<NodeId> ancestors;
  getAncestorsOfLayer(
      graph,
      config.query,
      config.query_layer,
      [&](const DynamicSceneGraph&, const NodeId node) { ancestors.push_back(node); });

  EXPECT_EQ(ancestors, config.expected);
}

const AncestorTestConfig ancestor_test_cases[] = {
    {0, 4, {}},
    {0, 3, {2}},
    {0, 2, {4, 5}},
    {1, 4, {}},
    {1, 3, {3}},
    {1, 2, {6, 7}},
    {2, 3, {}},
    {2, 2, {4, 5}},
    {3, 3, {}},
    {3, 2, {6, 7}},
};

INSTANTIATE_TEST_CASE_P(GetAncestors,
                        AncestorTestFixture,
                        testing::ValuesIn(ancestor_test_cases));

TEST_P(BoundingBoxTestFixture, BoundingBoxCorrect) {
  const BoundingBoxTestConfig config = GetParam();
  const auto bbox = computeAncestorBoundingBox(graph, config.query, config.query_layer);
  EXPECT_EQ(bbox, config.expected);
}

const BoundingBoxTestConfig bbox_test_cases[] = {
    {0, 4, {}},
    {1, 4, {}},
    {2, 3, {}},
    {3, 3, {}},
    {0, 2, {{1, 1, 1}, {2, 2, 2}}},
    {1, 2, {{3, 3, 3}, {4, 4, 4}}},
};

INSTANTIATE_TEST_CASE_P(GetChildBoundingBox,
                        BoundingBoxTestFixture,
                        testing::ValuesIn(bbox_test_cases));

}  // namespace kimera
