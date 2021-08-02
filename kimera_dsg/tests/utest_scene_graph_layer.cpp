#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph_layer.h>

using kimera::NodeAttributes;
using kimera::NodeId;
using kimera::NodeSymbol;
using kimera::SceneGraphLayer;
using Node = kimera::SceneGraphLayer::Node;
using NodeRef = kimera::SceneGraphLayer::NodeRef;
using Edge = kimera::SceneGraphLayer::Edge;
using EdgeInfo = kimera::SceneGraphLayer::EdgeInfo;
using EdgeRef = kimera::SceneGraphLayer::EdgeRef;
using NodeSet = std::unordered_set<NodeId>;

class TestableSceneGraphLayer : public SceneGraphLayer {
 public:
  TestableSceneGraphLayer(kimera::LayerId id) : SceneGraphLayer(id) {}
  using SceneGraphLayer::emplaceNode;
  using SceneGraphLayer::insertNode;
  using SceneGraphLayer::removeNode;
};

// Test that an empty layer has no nodes and edges
TEST(SceneGraphLayerTests, DefaultLayerInvariants) {
  TestableSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_EQ(0u, layer.numEdges());
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(SceneGraphLayerTests, EmplaceNodeInvariants) {
  TestableSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_FALSE(layer.hasNode(0));

  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(0));

  std::optional<NodeRef> node_opt = layer.getNode(0);
  ASSERT_TRUE(node_opt);
  const Node& node = *node_opt;
  EXPECT_EQ(1u, node.layer);
  EXPECT_EQ(0u, node.id);

  // we already have this node, so we should fail
  EXPECT_FALSE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(SceneGraphLayerTests, InsertNodeInvariants) {
  TestableSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_FALSE(layer.hasNode(0));

  Node::Ptr valid_node =
      std::make_unique<Node>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_TRUE(layer.insertNode(std::move(valid_node)));
  EXPECT_EQ(1u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(0));

  // we already have this node, so we should fail
  Node::Ptr repeat_node =
      std::make_unique<Node>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(layer.insertNode(std::move(repeat_node)));

  // invalid layers should also get rejected
  Node::Ptr invalid_node =
      std::make_unique<Node>(1, 0, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(layer.insertNode(std::move(invalid_node)));
  EXPECT_EQ(1u, layer.numNodes());

  // null nodes should also get rejected
  Node::Ptr null_node(nullptr);
  EXPECT_FALSE(layer.insertNode(std::move(null_node)));
  EXPECT_EQ(1u, layer.numNodes());
}

// Test that we only have edges that we add, and that edges added respect:
//   - That the source and target must exist
//   - That the edge must not already exist
//   - That edges are bidirectional
TEST(SceneGraphLayerTests, InsertEdgeInvariants) {
  TestableSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numEdges());
  EXPECT_FALSE(layer.hasEdge(0, 1));

  // source node
  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(layer.hasEdge(0, 1));
  EXPECT_FALSE(layer.insertEdge(0, 1));

  // target node
  EXPECT_TRUE(layer.emplaceNode(1, std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(layer.hasEdge(0, 1));

  // actually add the edge
  EXPECT_TRUE(layer.insertEdge(0, 1));
  EXPECT_TRUE(layer.hasEdge(0, 1));
  EXPECT_TRUE(layer.hasEdge(1, 0));
  EXPECT_EQ(1u, layer.numEdges());

  // add a duplicate
  EXPECT_FALSE(layer.insertEdge(0, 1));
  EXPECT_TRUE(layer.hasEdge(0, 1));
  EXPECT_TRUE(layer.hasEdge(1, 0));
  EXPECT_EQ(1u, layer.numEdges());
}

// Test that inserting specific edge attributes works
TEST(SceneGraphLayerTests, EdgeAttributesCorrect) {
  TestableSceneGraphLayer layer(1);
  // source and target nodes
  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(layer.emplaceNode(1, std::make_unique<NodeAttributes>()));

  // actually add the edge
  EdgeInfo::Ptr info = std::make_unique<EdgeInfo>();
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(layer.insertEdge(0, 1, std::move(info)));
  EXPECT_TRUE(layer.hasEdge(0, 1));
  EXPECT_EQ(1u, layer.numEdges());

  std::optional<EdgeRef> edge_opt = layer.getEdge(0, 1);
  ASSERT_TRUE(edge_opt);
  const Edge& edge = *edge_opt;
  EXPECT_EQ(0u, edge.source);
  EXPECT_EQ(1u, edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  std::optional<EdgeRef> swapped_edge_opt = layer.getEdge(1, 0);
  ASSERT_TRUE(swapped_edge_opt);

  const Edge& swapped_edge = *swapped_edge_opt;
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
TEST(SceneGraphLayerTests, BasicNodeIterationCorrect) {
  TestableSceneGraphLayer layer(1);
  size_t num_nodes = 5;
  std::set<int64_t> expected_ids;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.emplaceNode(i, std::make_unique<NodeAttributes>()));
    expected_ids.insert(i);
  }
  EXPECT_EQ(5u, layer.numNodes());

  // nodes may be stored unordered in the future
  std::set<int64_t> actual_ids;
  for (const auto& id_node_pair : layer.nodes()) {
    ASSERT_TRUE(id_node_pair.second != nullptr);
    EXPECT_EQ(id_node_pair.first, id_node_pair.second->id);
    actual_ids.insert(id_node_pair.second->id);
  }

  EXPECT_EQ(expected_ids, actual_ids);
}

// Test that edges we see via the public iterator match up with what we added
TEST(SceneGraphLayerTests, BasicEdgeIterationCorrect) {
  TestableSceneGraphLayer layer(1);
  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.emplaceNode(i, std::make_unique<NodeAttributes>()));
  }

  std::set<NodeId> expected_targets;
  for (size_t i = 1; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.insertEdge(i - 1, i));
    expected_targets.insert(i);
  }

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

// Test that removing a node meets the invariants that we expect
//   - we don't do anything if it doesn't exist
//   - we remove all edges related to the node if it does
TEST(SceneGraphLayerTests, RemoveNodeSound) {
  TestableSceneGraphLayer layer(1);

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(layer.removeNode(0));

  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.emplaceNode(i, std::make_unique<NodeAttributes>()));
  }

  for (size_t i = 1; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.insertEdge(0, i));
  }

  EXPECT_EQ(num_nodes, layer.numNodes());
  EXPECT_EQ(num_nodes - 1, layer.numEdges());
  layer.removeNode(0);
  EXPECT_EQ(num_nodes - 1, layer.numNodes());
  EXPECT_EQ(0u, layer.numEdges());
}

// Test that removing a edge does what it should
TEST(SceneGraphLayerTests, RemoveEdgeCorrect) {
  TestableSceneGraphLayer layer(1);

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(layer.removeEdge(0, 1));
  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(layer.emplaceNode(1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(layer.insertEdge(0, 1));

  EXPECT_EQ(1u, layer.numEdges());
  EXPECT_TRUE(layer.removeEdge(0, 1));
  EXPECT_EQ(0u, layer.numEdges());

  for (const auto& node_id_pair : layer.nodes()) {
    EXPECT_FALSE(node_id_pair.second->hasSiblings());
  }
}

TEST(SceneGraphLayerTests, getPositionCorrect) {
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>(expected);

  TestableSceneGraphLayer layer(1);
  layer.emplaceNode(NodeSymbol('x', 0), std::move(attrs));

  Eigen::Vector3d result = layer.getPosition(NodeSymbol('x', 0));
  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
  EXPECT_EQ(expected(2), result(2));

  try {
    layer.getPosition(NodeSymbol('x', 5));
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }
}

TEST(SceneGraphLayerTests, GetNeighborhoodCorrect) {
  TestableSceneGraphLayer layer(1);

  layer.emplaceNode(0, std::make_unique<NodeAttributes>());
  for (size_t i = 1; i < 7; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    layer.insertEdge(i - 1, i);
  }

  {  // one hop at start of chain
    NodeSet expected{0, 1};
    NodeSet result = layer.getNeighborhood(0);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }

  {  // full chain (3 hops at middle of chain)
    NodeSet expected{0, 1, 2, 3, 4, 5, 6};
    NodeSet result = layer.getNeighborhood(3, 3);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }
}

TEST(SceneGraphLayerTests, GetNeighborhoodFromSetCorrect) {
  TestableSceneGraphLayer layer(1);

  layer.emplaceNode(0, std::make_unique<NodeAttributes>());
  for (size_t i = 1; i < 7; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    layer.insertEdge(i - 1, i);
  }

  {  // single node
    NodeSet query{0};
    NodeSet expected{0, 1};
    NodeSet result = layer.getNeighborhood(query);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }

  {  // one hop at start and end of chain
    NodeSet query{0, 6};
    NodeSet expected{0, 1, 5, 6};
    NodeSet result = layer.getNeighborhood(query);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }

  {  // full chain (3 hops at ends of chain)
    NodeSet query{0, 6};
    NodeSet expected{0, 1, 2, 3, 4, 5, 6};
    NodeSet result = layer.getNeighborhood(query, 3);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }

  {  // chain with a loop and 2 hops (original implementation short-circuited visiting
     // too early)
    layer.insertEdge(0, 6);
    NodeSet query{0, 6};
    NodeSet expected{0, 1, 2, 4, 5, 6};
    NodeSet result = layer.getNeighborhood(query, 2);
    EXPECT_EQ(expected.size(), result.size());
    for (const auto node : result) {
      EXPECT_TRUE(expected.count(node)) << "Missing " << node;
    }
  }
}
