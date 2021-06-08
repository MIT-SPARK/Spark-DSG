#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph.h>

using kimera::NodeAttributes;
using kimera::NodeId;
using kimera::NodeSymbol;
using kimera::SceneGraph;
using kimera::SceneGraphLayer;
using NodeRef = kimera::SceneGraph::NodeRef;
using Node = kimera::SceneGraph::Node;
using Edge = kimera::SceneGraph::Edge;
using EdgeInfo = kimera::SceneGraph::EdgeInfo;
using EdgeRef = kimera::SceneGraph::EdgeRef;

TEST(SceneGraphTests, DefaultConstructorInvariants) {
  SceneGraph graph;
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(4u, graph.numLayers());
}

TEST(SceneGraphTests, CustomLayerInvariants) {
  SceneGraph::LayerIds layer_ids{1, 2, 3, 4, 5};
  SceneGraph graph(layer_ids);
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(layer_ids.size(), graph.numLayers());
}

TEST(SceneGraphTests, EmptyLayerFails) {
  SceneGraph::LayerIds layer_ids;
  try {
    SceneGraph graph(layer_ids);
    FAIL();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(SceneGraphTests, EmplaceNodeInvariants) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);
  EXPECT_EQ(0u, graph.numNodes());

  ASSERT_TRUE(graph.getLayer(1));
  ASSERT_TRUE(graph.getLayer(2));
  const SceneGraphLayer& layer1 = *(graph.getLayer(1));
  const SceneGraphLayer& layer2 = *(graph.getLayer(2));

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, layer1.numNodes());
  EXPECT_EQ(0u, layer2.numNodes());
  EXPECT_TRUE(graph.hasNode(0));
  EXPECT_TRUE(layer1.hasNode(0));
  EXPECT_FALSE(layer2.hasNode(0));

  std::optional<NodeRef> node_opt = layer1.getNode(0);
  ASSERT_TRUE(node_opt);
  const Node& node = *node_opt;
  EXPECT_EQ(1u, node.layer);
  EXPECT_EQ(0u, node.id);

  // we already have this node, so we should fail
  EXPECT_FALSE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(SceneGraphTests, InsertNodeInvariants) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);
  EXPECT_EQ(0u, graph.numNodes());

  ASSERT_TRUE(graph.getLayer(1));
  ASSERT_TRUE(graph.getLayer(2));
  const SceneGraphLayer& layer1 = *(graph.getLayer(1));
  const SceneGraphLayer& layer2 = *(graph.getLayer(2));

  Node::Ptr valid_node =
      std::make_unique<Node>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_TRUE(graph.insertNode(std::move(valid_node)));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, layer1.numNodes());
  EXPECT_EQ(0u, layer2.numNodes());
  EXPECT_TRUE(graph.hasNode(0));
  EXPECT_TRUE(layer1.hasNode(0));
  EXPECT_FALSE(layer2.hasNode(0));

  // we already have this node, so we should fail
  Node::Ptr repeat_node =
      std::make_unique<Node>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(graph.insertNode(std::move(repeat_node)));

  // invalid layers (0) should also get rejected
  Node::Ptr invalid_node =
      std::make_unique<Node>(1, 0, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(graph.insertNode(std::move(invalid_node)));

  // null nodes should also get rejected
  Node::Ptr null_node(nullptr);
  EXPECT_FALSE(graph.insertNode(std::move(null_node)));
}

// Test that we only have edges that we add, and that edges added respect:
//   - That the source and target must exist
//   - That the edge must not already exist
//   - That edges are bidirectional
//   - That the edge must be within a layer or between adjacent layers
TEST(SceneGraphTests, InsertEdgeInvariants) {
  SceneGraph::LayerIds layer_ids{1, 2, 3};
  SceneGraph graph(layer_ids);
  EXPECT_EQ(0u, graph.numEdges());

  ASSERT_TRUE(graph.getLayer(1));
  ASSERT_TRUE(graph.getLayer(2));
  ASSERT_TRUE(graph.getLayer(3));
  const SceneGraphLayer& layer1 = *(graph.getLayer(1));
  const SceneGraphLayer& layer2 = *(graph.getLayer(2));
  const SceneGraphLayer& layer3 = *(graph.getLayer(3));

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>()));

  // add an intra-layer edge
  EXPECT_TRUE(graph.insertEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_EQ(1u, layer1.numEdges());
  EXPECT_EQ(0u, layer2.numEdges());
  EXPECT_EQ(0u, layer3.numEdges());
  EXPECT_TRUE(graph.hasEdge(0, 1));
  EXPECT_TRUE(graph.hasEdge(1, 0));
  EXPECT_TRUE(layer1.hasEdge(0, 1));
  EXPECT_TRUE(layer1.hasEdge(1, 0));

  // add a duplicate
  EXPECT_FALSE(graph.insertEdge(1, 0));
  EXPECT_EQ(1u, graph.numEdges());

  // add an intra-layer edge
  EXPECT_TRUE(graph.insertEdge(0, 2));
  EXPECT_EQ(2u, graph.numEdges());
  EXPECT_EQ(1u, layer1.numEdges());
  EXPECT_EQ(0u, layer2.numEdges());
  EXPECT_EQ(0u, layer3.numEdges());
  EXPECT_TRUE(graph.hasEdge(0, 2));
  EXPECT_TRUE(graph.hasEdge(2, 0));

  // add an improper edge (0 already has a parent)
  EXPECT_FALSE(graph.insertEdge(0, 3));

  // add an edge between non-existant nodes
  EXPECT_FALSE(graph.insertEdge(0, 5));
  EXPECT_FALSE(graph.insertEdge(7, 0));
}

// Test that inserting specific edge attributes works for a inter-layer edge
TEST(SceneGraphTests, EdgeAttributesCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  // actually add the edge
  EdgeInfo::Ptr info = std::make_unique<EdgeInfo>();
  info->directed = true;
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(graph.insertEdge(0, 1, std::move(info)));
  EXPECT_TRUE(graph.hasEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());

  std::optional<EdgeRef> edge_opt = graph.getEdge(0, 1);
  ASSERT_TRUE(edge_opt);
  const Edge& edge = *edge_opt;
  EXPECT_EQ(0u, edge.source);
  EXPECT_EQ(1u, edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->directed);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  std::optional<EdgeRef> swapped_edge_opt = graph.getEdge(1, 0);
  ASSERT_TRUE(swapped_edge_opt);

  const Edge& swapped_edge = *swapped_edge_opt;
  // note that accessing an edge from the reverse direction
  // that is was added doesn't change the info and keeps the
  // source and target the same as how the edge was added
  EXPECT_EQ(0u, swapped_edge.source);
  EXPECT_EQ(1u, swapped_edge.target);
  ASSERT_TRUE(swapped_edge.info != nullptr);
  EXPECT_TRUE(swapped_edge.info->directed);
  EXPECT_TRUE(swapped_edge.info->weighted);
  EXPECT_EQ(0.5, swapped_edge.info->weight);
}

// Test that removing a node meets the invariants that we expect
//   - we don't do anything if it doesn't exist
//   - we remove all edges related to the node if it does
TEST(SceneGraphTests, RemoveNodeSound) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.removeNode(0));

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 3, std::make_unique<NodeAttributes>()));
  graph.insertEdge(0, 1);
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 3);
  graph.insertEdge(0, 2);

  EXPECT_EQ(4u, graph.numNodes());
  EXPECT_EQ(4u, graph.numEdges());
  graph.removeNode(0);
  EXPECT_EQ(3u, graph.numNodes());
  EXPECT_EQ(2u, graph.numEdges());
}

// Test that removing a edge does what it should
TEST(SceneGraphTests, RemoveEdgeCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.removeEdge(0, 1));

  std::vector<NodeId> nodes{0, 1, 2};
  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>()));

  // add and remove an intra-layer edge
  EXPECT_TRUE(graph.insertEdge(0, 2));
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.removeEdge(0, 2));
  EXPECT_EQ(0u, graph.numEdges());

  for (const auto& node_id : nodes) {
    ASSERT_TRUE(graph.getNode(node_id));
    const Node& node = *(graph.getNode(node_id));
    EXPECT_FALSE(node.hasSiblings());
    EXPECT_FALSE(node.hasParent());
    EXPECT_FALSE(node.hasChildren());
  }

  // add and remove an inter-layer edge
  EXPECT_TRUE(graph.insertEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.removeEdge(0, 1));
  EXPECT_EQ(0u, graph.numEdges());

  for (const auto& node_id : nodes) {
    ASSERT_TRUE(graph.getNode(node_id));
    const Node& node = *(graph.getNode(node_id));
    EXPECT_FALSE(node.hasSiblings());
    EXPECT_FALSE(node.hasParent());
    EXPECT_FALSE(node.hasChildren());
  }
}

TEST(SceneGraphTests, getPositionCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>(expected);

  graph.emplaceNode(1, NodeSymbol('x', 0), std::move(attrs));

  Eigen::Vector3d result = graph.getPosition(NodeSymbol('x', 0));
  EXPECT_EQ(expected(0), result(0));
  EXPECT_EQ(expected(1), result(1));
  EXPECT_EQ(expected(2), result(2));

  try {
    graph.getPosition(NodeSymbol('x', 5));
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }
}
