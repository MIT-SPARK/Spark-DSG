#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph_layer.h>

namespace kimera {

using Node = SceneGraphLayer::Node;
using NodeRef = SceneGraphLayer::NodeRef;
using Edge = SceneGraphLayer::Edge;
using Edges = SceneGraphLayer::Edges;
using EdgeRef = SceneGraphLayer::EdgeRef;
using NodeSet = std::unordered_set<NodeId>;

// Test that an empty layer has no nodes and edges
TEST(SceneGraphLayerTests, DefaultLayerInvariants) {
  IsolatedSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_EQ(0u, layer.numEdges());
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(SceneGraphLayerTests, EmplaceNodeInvariants) {
  IsolatedSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_FALSE(layer.hasNode(0));
  EXPECT_EQ(NodeStatus::NONEXISTENT, layer.checkNode(0));

  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(0));
  EXPECT_EQ(NodeStatus::VISIBLE, layer.checkNode(0));

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
  IsolatedSceneGraphLayer layer(1);
  EXPECT_EQ(0u, layer.numNodes());
  EXPECT_FALSE(layer.hasNode(0));
  EXPECT_EQ(NodeStatus::NONEXISTENT, layer.checkNode(0));

  Node::Ptr valid_node =
      std::make_unique<Node>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_TRUE(layer.insertNode(std::move(valid_node)));
  EXPECT_EQ(1u, layer.numNodes());
  EXPECT_TRUE(layer.hasNode(0));
  EXPECT_EQ(NodeStatus::VISIBLE, layer.checkNode(0));

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
  IsolatedSceneGraphLayer layer(1);
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
  IsolatedSceneGraphLayer layer(1);
  // source and target nodes
  EXPECT_TRUE(layer.emplaceNode(0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(layer.emplaceNode(1, std::make_unique<NodeAttributes>()));

  // actually add the edge
  auto info = std::make_unique<EdgeAttributes>();
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
  IsolatedSceneGraphLayer layer(1);
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
  IsolatedSceneGraphLayer layer(1);
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
  IsolatedSceneGraphLayer layer(1);

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
  EXPECT_EQ(NodeStatus::DELETED, layer.checkNode(0));
}

// Test that merging two nodes meets the invariants that we expect
//   - we don't do anything if either nodes doesn't exist
//   - we rewire all edges to the merged nodes if we do
TEST(SceneGraphLayerTests, MergeNodesCorrect) {
  IsolatedSceneGraphLayer layer(1);

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(layer.mergeNodes(0, 1));

  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.emplaceNode(i, std::make_unique<NodeAttributes>()));
  }
  EXPECT_FALSE(layer.mergeNodes(0, 5));
  layer.removeNode(4);
  EXPECT_FALSE(layer.mergeNodes(4, 0));

  for (size_t i = 1; i < 4; ++i) {
    EXPECT_TRUE(layer.insertEdge(0, i));
  }

  const Node& node0 = *layer.getNode(0);
  const Node& node1 = *layer.getNode(1);

  EXPECT_EQ(4u, layer.numNodes());
  EXPECT_EQ(3u, layer.numEdges());
  EXPECT_EQ(3u, node0.siblings().size());
  EXPECT_EQ(1u, node1.siblings().size());
  layer.mergeNodes(0, 1);
  EXPECT_EQ(3u, layer.numNodes());
  EXPECT_EQ(2u, layer.numEdges());
  EXPECT_EQ(2u, node1.siblings().size());
  EXPECT_EQ(NodeStatus::MERGED, layer.checkNode(0));
  EXPECT_EQ(NodeStatus::DELETED, layer.checkNode(4));
}

// Test that removing a edge does what it should
TEST(SceneGraphLayerTests, RemoveEdgeCorrect) {
  IsolatedSceneGraphLayer layer(1);

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

// Test that rewiring an edge does what it should
TEST(SceneGraphLayerTests, RewireEdgeCorrect) {
  IsolatedSceneGraphLayer layer(1);

  size_t num_nodes = 5;
  for (size_t i = 0; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.emplaceNode(i, std::make_unique<NodeAttributes>()));
  }

  for (size_t i = 1; i < num_nodes; ++i) {
    EXPECT_TRUE(layer.insertEdge(i - 1, i));
  }
  EXPECT_EQ(4u, layer.numEdges());

  // we can't rewire an edge that doesn't exist
  EXPECT_FALSE(layer.rewireEdge(4, 5, 0, 1));
  // we can't rewire an edge to itself
  EXPECT_FALSE(layer.rewireEdge(0, 1, 0, 1));
  // if the new edge to be rewired is the same, simply remove
  layer.rewireEdge(2, 3, 2, 2);
  EXPECT_EQ(3u, layer.numEdges());
  // if the new edge to be rewired already exists, simply remove
  layer.rewireEdge(1, 2, 1, 0);
  EXPECT_EQ(2u, layer.numEdges());
  // try rewiring edge
  layer.rewireEdge(3, 4, 3, 0);
  EXPECT_EQ(2u, layer.numEdges());

  EXPECT_TRUE(layer.hasEdge(0, 3));
  EXPECT_TRUE(layer.hasEdge(0, 1));
  EXPECT_FALSE(layer.hasEdge(3, 4));
}

// Test that rewiring an edge does what it should
TEST(SceneGraphLayerTests, MergeLayerCorrect) {
  IsolatedSceneGraphLayer layer_1(1);
  IsolatedSceneGraphLayer layer_2(1);

  for (size_t i = 0; i < 3; ++i) {
    Eigen::Vector3d node_pos;
    node_pos << static_cast<double>(i), 0.0, 0.0;
    EXPECT_TRUE(layer_1.emplaceNode(i, std::make_unique<NodeAttributes>(node_pos)));
  }
  for (size_t i = 1; i < 3; ++i) {
    EXPECT_TRUE(layer_1.insertEdge(i - 1, i));
  }

  for (size_t i = 0; i < 5; ++i) {
    Eigen::Vector3d node_pos;
    node_pos << static_cast<double>(i + 10), 0.0, 0.0;
    EXPECT_TRUE(layer_2.emplaceNode(i, std::make_unique<NodeAttributes>(node_pos)));
  }
  for (size_t i = 1; i < 5; ++i) {
    EXPECT_TRUE(layer_2.insertEdge(i - 1, i));
  }

  std::map<NodeId, LayerKey> node_to_layer;
  layer_1.mergeLayer(layer_2, &node_to_layer);

  EXPECT_EQ(2u, node_to_layer.size());
  EXPECT_EQ(5u, layer_1.numNodes());
  EXPECT_EQ(4u, layer_1.numEdges());

  for (size_t i = 0; i < 5; i++) {
    Eigen::Vector3d result = layer_1.getPosition(i);
    EXPECT_EQ(static_cast<double>(i), result(0));
    EXPECT_EQ(0.0, result(1));
    EXPECT_EQ(0.0, result(2));
    EXPECT_EQ(NodeStatus::VISIBLE, layer_1.checkNode(i));
    if (i > 2) {
      EXPECT_EQ(LayerKey(1), node_to_layer.at(i));
    }
  }
}

TEST(SceneGraphLayerTests, getPositionCorrect) {
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>(expected);

  IsolatedSceneGraphLayer layer(1);
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
  IsolatedSceneGraphLayer layer(1);

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
  IsolatedSceneGraphLayer layer(1);

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

TEST(SceneGraphLayerTests, SerializeDeserializeCorrect) {
  IsolatedSceneGraphLayer layer(1);

  layer.emplaceNode(0, std::make_unique<NodeAttributes>());
  for (size_t i = 1; i < 7; ++i) {
    layer.emplaceNode(i, std::make_unique<NodeAttributes>());
    layer.insertEdge(i - 1, i);
  }

  IsolatedSceneGraphLayer deserialized_layer(1);

  {  // no nodes -> empty result on deserialization
    std::unordered_set<NodeId> nodes;
    std::string contents = layer.serializeLayer(nodes);
    std::unique_ptr<Edges> edges = deserialized_layer.deserializeLayer(contents);
    ASSERT_TRUE(edges != nullptr);
    EXPECT_TRUE(edges->empty());
    EXPECT_EQ(0u, deserialized_layer.numNodes());
    EXPECT_EQ(0u, deserialized_layer.numEdges());
  }

  {  // first node -> 1 edge
    std::unordered_set<NodeId> nodes{0};
    std::string contents = layer.serializeLayer(nodes);
    std::unique_ptr<Edges> edges = deserialized_layer.deserializeLayer(contents);
    ASSERT_TRUE(edges != nullptr);
    EXPECT_EQ(1u, edges->size());
    EXPECT_EQ(1u, deserialized_layer.numNodes());
    EXPECT_EQ(0u, deserialized_layer.numEdges());
    ASSERT_TRUE(edges->count(0));
    EXPECT_EQ(0u, edges->at(0).source);
    EXPECT_EQ(1u, edges->at(0).target);
  }

  {  // second node -> 2 edges
    std::unordered_set<NodeId> nodes{1};
    std::string contents = layer.serializeLayer(nodes);
    std::unique_ptr<Edges> edges = deserialized_layer.deserializeLayer(contents);
    ASSERT_TRUE(edges != nullptr);
    EXPECT_EQ(2u, edges->size());
    EXPECT_EQ(1u, deserialized_layer.numNodes());
    EXPECT_EQ(0u, deserialized_layer.numEdges());
  }

  {  // all nodes -> 12 edges
    std::unordered_set<NodeId> nodes{0, 1, 2, 3, 4, 5, 6};
    std::string contents = layer.serializeLayer(nodes);
    std::unique_ptr<Edges> edges = deserialized_layer.deserializeLayer(contents);
    ASSERT_TRUE(edges != nullptr);
    EXPECT_EQ(12u, edges->size());
    EXPECT_EQ(7u, deserialized_layer.numNodes());
    EXPECT_EQ(0u, deserialized_layer.numEdges());
  }
}

}  // namespace kimera
