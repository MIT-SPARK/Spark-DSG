#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph.h>

using kimera::IsolatedSceneGraphLayer;
using kimera::NodeAttributes;
using kimera::NodeId;
using kimera::NodeSymbol;
using kimera::SceneGraph;
using kimera::SceneGraphLayer;
using NodeRef = kimera::SceneGraph::NodeRef;
using Node = kimera::SceneGraph::Node;
using Edge = kimera::SceneGraph::Edge;
using Edges = kimera::SceneGraph::Edges;
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

// Test that merging two nodes meets the invariants that we expect
TEST(SceneGraphTests, MergeNodesCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2, 3};
  SceneGraph graph(layer_ids);

  // we can't merge two nodes that don't exist
  EXPECT_FALSE(graph.mergeNodes(0, 1));

  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 4, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 5, std::make_unique<NodeAttributes>()));
  graph.insertEdge(0, 1);
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 3);
  graph.insertEdge(0, 2);
  graph.insertEdge(2, 5);

  // we can't merge a node with itself
  EXPECT_FALSE(graph.mergeNodes(0, 0));

  // we can't merge nodes on different layers
  EXPECT_FALSE(graph.mergeNodes(0, 1));

  const Node& node0 = graph.getNode(0).value();
  const Node& node2 = graph.getNode(2).value();
  const Node& node4 = graph.getNode(4).value();
  const Node& node5 = graph.getNode(5).value();

  EXPECT_TRUE(graph.hasNode(5));
  EXPECT_FALSE(node4.hasParent());

  NodeId node5_parent = *(node5.getParent());
  EXPECT_EQ(2u, node5_parent);

  // merge node 5 into node 4
  EXPECT_TRUE(graph.mergeNodes(5, 4));

  EXPECT_FALSE(graph.hasNode(5));
  EXPECT_TRUE(node4.hasParent());

  NodeId node4_parent = *(node4.getParent());
  EXPECT_EQ(2u, node4_parent);

  std::set<NodeId> node2_children = node2.children();
  EXPECT_EQ(1u, node2_children.size());
  EXPECT_EQ(4u, *node2_children.begin());

  // merge node 2 into node 0
  EXPECT_TRUE(graph.mergeNodes(2, 0));

  EXPECT_EQ(3u, graph.numEdges());
  EXPECT_EQ(4u, graph.numNodes());

  node4_parent = *(node4.getParent());
  std::set<NodeId> node0_children = node0.children();

  EXPECT_TRUE(graph.hasEdge(0, 4));
  EXPECT_EQ(0u, node4_parent);
  EXPECT_EQ(4u, *node0_children.begin());
  EXPECT_FALSE(graph.hasNode(2));
}

// Test that removeNode -> !hasNode
TEST(SceneGraphTests, RemoveNodeHasNodeCorrent) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);
  const SceneGraphLayer& layer = graph.getLayer(1).value();

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.removeNode(0));
  EXPECT_FALSE(graph.hasNode(0));

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));

  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_TRUE(graph.hasNode(0));
  EXPECT_EQ(layer.hasNode(0), graph.hasNode(0));
  graph.removeNode(0);
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(0));
  EXPECT_EQ(layer.hasNode(0), graph.hasNode(0));
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

TEST(SceneGraphTests, updateFromInvalidLayerCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  IsolatedSceneGraphLayer separate_layer(5);
  std::unique_ptr<Edges> edges(nullptr);
  EXPECT_FALSE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
}

TEST(SceneGraphTests, updateFromEmptyLayerCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);

  IsolatedSceneGraphLayer separate_layer(1);
  std::unique_ptr<Edges> edges(nullptr);
  EXPECT_TRUE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
}

TEST(SceneGraphTests, updateFromLayerCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph(layer_ids);
  graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 3, std::make_unique<NodeAttributes>());
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 3);

  IsolatedSceneGraphLayer separate_layer(1);
  separate_layer.emplaceNode(1, std::make_unique<NodeAttributes>());
  separate_layer.emplaceNode(5, std::make_unique<NodeAttributes>());
  std::unique_ptr<Edges> edges(new Edges);
  edges->emplace(std::piecewise_construct,
                 std::forward_as_tuple(0),
                 std::forward_as_tuple(5, 2, std::make_unique<EdgeInfo>()));
  edges->emplace(std::piecewise_construct,
                 std::forward_as_tuple(1),
                 std::forward_as_tuple(1, 2, std::make_unique<EdgeInfo>()));

  EXPECT_TRUE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(4u, graph.numNodes());
  EXPECT_EQ(3u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
  EXPECT_TRUE(graph.hasNode(5));
  EXPECT_TRUE(graph.hasEdge(5, 2));
}

TEST(SceneGraphTests, updateFromLayerWithSiblings) {
  SceneGraph::LayerIds layer_ids{1};
  SceneGraph graph(layer_ids);
  graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>());
  graph.insertEdge(1, 2);

  IsolatedSceneGraphLayer separate_layer(1);
  separate_layer.emplaceNode(2, std::make_unique<NodeAttributes>());
  std::unique_ptr<Edges> edges(new Edges);

  EXPECT_TRUE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.hasNode(1));
  EXPECT_TRUE(graph.hasNode(2));
  EXPECT_TRUE(graph.hasEdge(1, 2));

  EXPECT_TRUE(graph.removeNode(2));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_TRUE(graph.hasNode(1));
  EXPECT_FALSE(graph.hasNode(2));
  EXPECT_FALSE(graph.hasEdge(1, 2));
}

TEST(SceneGraphTests, MereGraphCorrect) {
  SceneGraph::LayerIds layer_ids{1, 2};
  SceneGraph graph_1(layer_ids);
  SceneGraph graph_2(layer_ids);

  Eigen::Vector3d pos_1;
  pos_1 << 1.0, 1.0, 1.0;
  EXPECT_TRUE(graph_1.emplaceNode(1, 0, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(2, 1, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(1, 2, std::make_unique<NodeAttributes>(pos_1)));
  graph_1.insertEdge(0, 1);
  graph_1.insertEdge(1, 2);
  graph_1.insertEdge(0, 2);

  Eigen::Vector3d pos_2;
  pos_2 << 2.0, 2.0, 2.0;
  EXPECT_TRUE(graph_2.emplaceNode(1, 0, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(2, 1, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(1, 2, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(2, 3, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(1, 4, std::make_unique<NodeAttributes>(pos_2)));
  graph_2.insertEdge(0, 1);
  graph_2.insertEdge(1, 2);
  graph_2.insertEdge(1, 3);
  graph_2.insertEdge(0, 2);
  graph_2.insertEdge(3, 4);

  EXPECT_TRUE(graph_1.mergeGraph(graph_2));

  EXPECT_EQ(5u, graph_1.numNodes());
  EXPECT_EQ(5u, graph_1.numEdges());

  Eigen::Vector3d result = graph_1.getPosition(4);
  EXPECT_EQ(pos_1(0), result(0));
  EXPECT_EQ(pos_1(1), result(1));
  EXPECT_EQ(pos_1(2), result(2));
}