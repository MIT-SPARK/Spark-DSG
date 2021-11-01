#include <gtest/gtest.h>
#include <kimera_dsg/dynamic_scene_graph_layer.h>

using kimera::DynamicLayerKey;
using kimera::DynamicSceneGraphLayer;
using kimera::NodeAttributes;
using kimera::NodeId;
using kimera::NodeSymbol;
using Node = kimera::DynamicSceneGraphLayer::Node;
using NodeRef = kimera::DynamicSceneGraphLayer::NodeRef;
using Edge = kimera::DynamicSceneGraphLayer::Edge;
using Edges = kimera::DynamicSceneGraphLayer::Edges;
using EdgeInfo = kimera::DynamicSceneGraphLayer::EdgeInfo;
using EdgeRef = kimera::DynamicSceneGraphLayer::EdgeRef;

class TestableDynamicLayer : public DynamicSceneGraphLayer {
 public:
  TestableDynamicLayer(kimera::LayerId id, char prefix)
      : DynamicSceneGraphLayer(id, prefix) {}
  using DynamicSceneGraphLayer::emplaceNode;
  // using DynamicSceneGraphLayer::removeNode;
};

// Test that an empty layer has no nodes and edges
TEST(DynamicSceneGraphLayerTests, DefaultLayerInvariants) {
  TestableDynamicLayer layer(1, 'a');
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_EQ(0u, layer.numEdges());
}

// Test that we have nodes we add, and that duplicate times get rejected
TEST(DynamicSceneGraphLayerTests, EmplaceNodeInvariants) {
  const char layer_prefix = 'a';
  TestableDynamicLayer layer(1, layer_prefix);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_FALSE(layer.hasNodeByIndex(0));
  EXPECT_FALSE(layer.hasNode(NodeSymbol(layer_prefix, 0)));

  EXPECT_TRUE(
      layer.emplaceNode(std::chrono::seconds(1), std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(NodeSymbol(layer_prefix, 0)));
  EXPECT_FALSE(layer.hasNode(NodeSymbol(layer_prefix + 1, 0)));
  EXPECT_TRUE(layer.hasNodeByIndex(0));

  EXPECT_FALSE(layer.getNodeByIndex(NodeSymbol(layer_prefix + 1, 0)));
  EXPECT_TRUE(layer.getNodeByIndex(0));
  std::optional<NodeRef> node_opt = layer.getNode(NodeSymbol(layer_prefix, 0));
  ASSERT_TRUE(node_opt);
  const Node& node = *node_opt;
  EXPECT_EQ(1u, node.layer);
  EXPECT_EQ(NodeSymbol(layer_prefix, 0), node.id);
  EXPECT_NEAR(1.0, std::chrono::duration<double>(node.timestamp).count(), 1.0e-9);

  EXPECT_FALSE(
      layer.emplaceNode(std::chrono::seconds(1), std::make_unique<NodeAttributes>()));

  EXPECT_TRUE(
      layer.emplaceNode(std::chrono::seconds(2), std::make_unique<NodeAttributes>()));

  EXPECT_EQ(2u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(NodeSymbol(layer_prefix, 1)));
  EXPECT_TRUE(layer.hasNodeByIndex(1));
}

// Test that we only have edges that we add, and that edges added respect:
//   - That the source and target must exist
//   - That the edge must not already exist
//   - That edges are bidirectional
TEST(DynamicSceneGraphLayerTests, InsertEdgeInvariants) {
  const char prefix = 'a';

  TestableDynamicLayer layer(1, prefix);
  EXPECT_EQ(0u, layer.numEdges());
  EXPECT_FALSE(layer.hasEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_FALSE(layer.hasEdgeByIndex(0, 1));

  // source node
  EXPECT_TRUE(
      layer.emplaceNode(std::chrono::seconds(1), std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(layer.hasEdgeByIndex(0, 1));
  EXPECT_FALSE(layer.insertEdgeByIndex(0, 1));
  EXPECT_FALSE(layer.insertEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_EQ(0u, layer.numEdges());

  // target node (produces an edge)
  EXPECT_TRUE(
      layer.emplaceNode(std::chrono::seconds(2), std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(layer.hasEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_TRUE(layer.hasEdgeByIndex(0, 1));
  EXPECT_TRUE(layer.hasEdge(NodeSymbol(prefix, 1), NodeSymbol(prefix, 0)));
  EXPECT_TRUE(layer.hasEdgeByIndex(1, 0));
  EXPECT_FALSE(layer.hasEdge(NodeSymbol('b', 0), NodeSymbol('b', 1)));
  EXPECT_EQ(1u, layer.numEdges());

  // adding a duplicate fails
  EXPECT_FALSE(layer.insertEdgeByIndex(0, 1));
  EXPECT_FALSE(layer.insertEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_EQ(1u, layer.numEdges());

  // adding a node without an automatic edge doesn't produce an edge
  EXPECT_TRUE(layer.emplaceNode(
      std::chrono::seconds(3), std::make_unique<NodeAttributes>(), false));
  EXPECT_FALSE(layer.hasEdgeByIndex(1, 2));
  EXPECT_FALSE(layer.hasEdgeByIndex(2, 1));
  EXPECT_EQ(1u, layer.numEdges());

  // manually adding an edge by symbol works
  EXPECT_TRUE(layer.insertEdge(NodeSymbol(prefix, 1), NodeSymbol(prefix, 2)));
  EXPECT_TRUE(layer.hasEdgeByIndex(1, 2));
  EXPECT_TRUE(layer.hasEdgeByIndex(2, 1));
  EXPECT_EQ(2u, layer.numEdges());

  EXPECT_TRUE(layer.insertEdgeByIndex(0, 2));
  EXPECT_TRUE(layer.hasEdgeByIndex(0, 2));
  EXPECT_TRUE(layer.hasEdgeByIndex(2, 0));
  EXPECT_EQ(3u, layer.numEdges());
}

// Test that inserting specific edge attributes works
TEST(DynamicSceneGraphLayerTests, EdgeAttributesCorrect) {
  TestableDynamicLayer layer(1, 'a');
  // source and target nodes
  EXPECT_TRUE(layer.emplaceNode(
      std::chrono::seconds(1), std::make_unique<NodeAttributes>(), false));
  EXPECT_TRUE(layer.emplaceNode(
      std::chrono::seconds(2), std::make_unique<NodeAttributes>(), false));

  // actually add the edge
  EdgeInfo::Ptr info = std::make_unique<EdgeInfo>();
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(layer.insertEdgeByIndex(0, 1, std::move(info)));
  EXPECT_TRUE(layer.hasEdgeByIndex(0, 1));
  EXPECT_EQ(1u, layer.numEdges());

  EXPECT_TRUE(layer.getEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  std::optional<EdgeRef> edge_opt = layer.getEdgeByIndex(0, 1);
  ASSERT_TRUE(edge_opt);
  const Edge& edge = *edge_opt;
  EXPECT_EQ(NodeSymbol('a', 0), edge.source);
  EXPECT_EQ(NodeSymbol('a', 1), edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  std::optional<EdgeRef> swapped_edge_opt = layer.getEdgeByIndex(1, 0);
  ASSERT_TRUE(swapped_edge_opt);

  const Edge& swapped_edge = *swapped_edge_opt;
  // note that accessing an edge from the reverse direction
  // that is was added doesn't change the info and keeps the
  // source and target the same as how the edge was added
  EXPECT_EQ(NodeSymbol('a', 0), swapped_edge.source);
  EXPECT_EQ(NodeSymbol('a', 1), swapped_edge.target);
  ASSERT_TRUE(swapped_edge.info != nullptr);
  EXPECT_TRUE(swapped_edge.info->weighted);
  EXPECT_EQ(0.5, swapped_edge.info->weight);
}

// Test that nodes we see via the public iterator match up with what we added
TEST(DynamicSceneGraphLayerTests, BasicNodeIterationCorrect) {
  // no prefix (for ease of testing
  TestableDynamicLayer layer(1, 0);
  size_t num_nodes = 5;
  std::set<int64_t> expected_ids;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(
        layer.emplaceNode(std::chrono::seconds(i), std::make_unique<NodeAttributes>()));
    expected_ids.insert(i);
  }
  EXPECT_EQ(5u, layer.numNodes());

  // nodes may be stored unordered in the future
  std::set<int64_t> actual_ids;
  for (const auto& node : layer.nodes()) {
    ASSERT_TRUE(node != nullptr);
    actual_ids.insert(node->id);
  }

  EXPECT_EQ(expected_ids, actual_ids);
}

// Test that edges we see via the public iterator match up with what we added
TEST(DynamicSceneGraphLayerTests, BasicEdgeIterationCorrect) {
  // no prefix for ease of testing
  TestableDynamicLayer layer(1, 0);
  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(
        layer.emplaceNode(std::chrono::seconds(i), std::make_unique<NodeAttributes>()));
  }

  std::set<NodeId> expected_targets{1, 2, 3, 4};

  // nodes may be stored unordered in the future
  std::set<NodeId> actual_targets;
  for (const auto& id_edge_pair : layer.edges()) {
    const Edge& edge = id_edge_pair.second;
    ASSERT_TRUE(edge.info != nullptr);
    EXPECT_EQ(edge.source + 1, edge.target);
    actual_targets.insert(edge.target);
  }

  EXPECT_EQ(expected_targets, actual_targets);
}

TEST(DynamicSceneGraphLayerTests, getPositionCorrect) {
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>(expected);

  TestableDynamicLayer layer(1, 0);
  layer.emplaceNode(std::chrono::seconds(1), std::move(attrs));

  Eigen::Vector3d result = layer.getPosition(NodeSymbol(0, 0));
  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
  EXPECT_EQ(expected(2), result(2));

  result = layer.getPositionByIndex(0);
  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
  EXPECT_EQ(expected(2), result(2));

  try {
    layer.getPosition(NodeSymbol(0, 5));
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }

  try {
    layer.getPositionByIndex(1);
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }
}

// Test that rewiring an edge does what it should
TEST(DynamicSceneGraphLayerTests, MergeLayerCorrect) {
  TestableDynamicLayer layer_1(1, 0);
  TestableDynamicLayer layer_2(1, 0);

  for (size_t i = 0; i < 3; ++i) {
    Eigen::Vector3d node_pos;
    node_pos << static_cast<double>(i), 0.0, 0.0;
    EXPECT_TRUE(layer_1.emplaceNode(
        std::chrono::seconds(i), std::make_unique<NodeAttributes>(node_pos)));
  }

  for (size_t i = 0; i < 5; ++i) {
    Eigen::Vector3d node_pos;
    node_pos << static_cast<double>(i + 10), 0.0, 0.0;
    EXPECT_TRUE(layer_2.emplaceNode(
        std::chrono::seconds(i), std::make_unique<NodeAttributes>(node_pos)));
  }

  std::map<NodeId, DynamicLayerKey> node_to_layer;
  layer_1.mergeLayer(layer_2, &node_to_layer);

  EXPECT_EQ(2u, node_to_layer.size());
  EXPECT_EQ(5u, layer_1.numNodes());
  EXPECT_EQ(4u, layer_1.numEdges());

  for (size_t i = 0; i < 5; i++) {
    Eigen::Vector3d result = layer_1.getPosition(i);
    EXPECT_EQ(static_cast<double>(i), result(0));
    EXPECT_EQ(0.0, result(1));
    EXPECT_EQ(0.0, result(2));
    if (i > 2) {
      EXPECT_EQ(1u, node_to_layer.at(i).type);
      EXPECT_EQ(0, node_to_layer.at(i).prefix);
    }
  }
}
