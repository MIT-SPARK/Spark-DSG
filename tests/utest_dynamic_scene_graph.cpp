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
#include <spark_dsg/dynamic_scene_graph.h>

namespace spark_dsg {

std::shared_ptr<Mesh> makeMesh(size_t num_points) {
  auto mesh = std::make_shared<Mesh>();

  for (size_t i = 0; i < num_points; ++i) {
    mesh->points.push_back(Eigen::Vector3f());
  }

  return mesh;
}

TEST(DynamicSceneGraphTests, DefaultConstructorInvariants) {
  DynamicSceneGraph graph;
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(4u, graph.numLayers());
}

TEST(DynamicSceneGraphTests, CustomLayerInvariants) {
  DynamicSceneGraph graph({1, 2, 3, 4, 5, 6});
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(6u, graph.numLayers());
}

TEST(DynamicSceneGraphTests, EmptyLayerFails) {
  try {
    DynamicSceneGraph graph(std::vector<LayerId>{});
    FAIL();
  } catch (const std::domain_error&) {
    SUCCEED();
  }
}

TEST(DynamicSceneGraphTests, NumNodesAndEdges) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.insertEdge(0, 1));

  auto mesh = makeMesh(5);

  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());
  graph.setMesh(mesh);
  EXPECT_EQ(7u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(DynamicSceneGraphTests, EmplaceNodeInvariants) {
  DynamicSceneGraph graph({1, 2});
  EXPECT_EQ(0u, graph.numNodes());

  const auto& layer1 = graph.getLayer(1);
  const auto& layer2 = graph.getLayer(2);

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, layer1.numNodes());
  EXPECT_EQ(0u, layer2.numNodes());
  EXPECT_TRUE(graph.hasNode(0));
  EXPECT_TRUE(layer1.hasNode(0));
  EXPECT_FALSE(layer2.hasNode(0));

  auto node_opt = layer1.findNode(0);
  ASSERT_TRUE(node_opt);
  const auto& node = *node_opt;
  EXPECT_EQ(1u, node.layer);
  EXPECT_EQ(0u, node.id);

  // we already have this node, so we should fail
  EXPECT_FALSE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));

  // we can still update the node however
  EXPECT_TRUE(graph.addOrUpdateNode(2, 0, std::make_unique<NodeAttributes>()));
}

TEST(DynamicSceneGraphTests, AddAndUpdateNode) {
  DynamicSceneGraph graph({1, 2});
  EXPECT_EQ(0u, graph.numNodes());

  Eigen::Vector3d pos1(1.0, 2.0, 3.0);
  Eigen::Vector3d pos2(2.0, 3.0, 4.0);

  {  // add first node and test invariants
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position = pos1;
    EXPECT_TRUE(graph.addOrUpdateNode(1, 0, std::move(attrs)));

    EXPECT_EQ(1u, graph.numNodes());
    EXPECT_TRUE(graph.hasNode(0));

    auto node_opt = graph.findNode(0);
    ASSERT_TRUE(node_opt);
    const auto& node = *node_opt;
    EXPECT_EQ(1u, node.layer);
    EXPECT_EQ(0u, node.id);
    EXPECT_NEAR((node.attributes().position - pos1).norm(), 0.0, 1.0e-9);
  }

  {  // add updated node attributes and test invariants
    auto attrs = std::make_unique<NodeAttributes>();
    attrs->position = pos2;
    EXPECT_TRUE(graph.addOrUpdateNode(1, 0, std::move(attrs)));

    EXPECT_EQ(1u, graph.numNodes());
    EXPECT_TRUE(graph.hasNode(0));

    auto node_opt = graph.findNode(0);
    ASSERT_TRUE(node_opt);
    const auto& node = *node_opt;
    EXPECT_EQ(1u, node.layer);
    EXPECT_EQ(0u, node.id);
    EXPECT_NEAR((node.attributes().position - pos2).norm(), 0.0, 1.0e-9);
  }
}

// Test that we only have nodes that we add, and we can't add the same node
TEST(DynamicSceneGraphTests, InsertNodeInvariants) {
  DynamicSceneGraph graph({1, 2});
  EXPECT_EQ(0u, graph.numNodes());

  const auto& layer1 = graph.getLayer(1);
  const auto& layer2 = graph.getLayer(2);

  auto valid_node =
      std::make_unique<SceneGraphNode>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_TRUE(graph.insertNode(std::move(valid_node)));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, layer1.numNodes());
  EXPECT_EQ(0u, layer2.numNodes());
  EXPECT_TRUE(graph.hasNode(0));
  EXPECT_TRUE(layer1.hasNode(0));
  EXPECT_FALSE(layer2.hasNode(0));

  // we already have this node, so we should fail
  auto repeat_node =
      std::make_unique<SceneGraphNode>(0, 1, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(graph.insertNode(std::move(repeat_node)));

  // invalid layers (0) should also get rejected
  auto invalid_node =
      std::make_unique<SceneGraphNode>(1, 0, std::make_unique<NodeAttributes>());
  EXPECT_FALSE(graph.insertNode(std::move(invalid_node)));

  // null nodes should also get rejected
  std::unique_ptr<SceneGraphNode> null_node(nullptr);
  EXPECT_FALSE(graph.insertNode(std::move(null_node)));
}

// Test that we only have edges that we add, and that edges added respect:
//   - That the source and target must exist
//   - That the edge must not already exist
//   - That edges are bidirectional
//   - That the edge must be within a layer or between adjacent layers
TEST(DynamicSceneGraphTests, InsertEdgeInvariants) {
  DynamicSceneGraph graph({1, 2, 3});
  EXPECT_EQ(0u, graph.numEdges());

  const auto& layer1 = graph.getLayer(1);
  const auto& layer2 = graph.getLayer(2);
  const auto& layer3 = graph.getLayer(3);

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>()));

  // add an intralayer edge
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

  // add an interlayer edge
  EXPECT_TRUE(graph.insertEdge(0, 2));
  EXPECT_EQ(2u, graph.numEdges());
  EXPECT_EQ(1u, layer1.numEdges());
  EXPECT_EQ(0u, layer2.numEdges());
  EXPECT_EQ(0u, layer3.numEdges());
  EXPECT_TRUE(graph.hasEdge(0, 2));
  EXPECT_TRUE(graph.hasEdge(2, 0));

  // add an improper edge (0 already has a parent) that should work
  EXPECT_TRUE(graph.insertEdge(0, 3));

  // add an edge between non-existant nodes
  EXPECT_FALSE(graph.insertEdge(0, 5));
  EXPECT_FALSE(graph.insertEdge(7, 0));
}

// Test that adding and updating edges works as expected
TEST(DynamicSceneGraphTests, AddOrUpdateEdge) {
  DynamicSceneGraph graph({1, 2, 3});
  EXPECT_EQ(0u, graph.numEdges());

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));

  // add and update an intralayer edge
  EXPECT_TRUE(graph.addOrUpdateEdge(0, 1, std::make_unique<EdgeAttributes>(1.0)));
  EXPECT_EQ(1u, graph.numEdges());
  ASSERT_TRUE(graph.hasEdge(0, 1));
  EXPECT_EQ(graph.getEdge(0, 1).info->weight, 1.0);
  EXPECT_TRUE(graph.addOrUpdateEdge(0, 1, std::make_unique<EdgeAttributes>(2.0)));
  EXPECT_EQ(graph.getEdge(0, 1).info->weight, 2.0);

  // add and update an interlayer edge
  EXPECT_TRUE(graph.addOrUpdateEdge(0, 2, std::make_unique<EdgeAttributes>(1.0)));
  EXPECT_EQ(2u, graph.numEdges());
  ASSERT_TRUE(graph.hasEdge(0, 2));
  EXPECT_EQ(graph.getEdge(0, 2).info->weight, 1.0);
  EXPECT_TRUE(graph.addOrUpdateEdge(0, 2, std::make_unique<EdgeAttributes>(2.0)));
  EXPECT_EQ(graph.getEdge(0, 2).info->weight, 2.0);
}

// Test that we can force switching a parent edge
TEST(DynamicSceneGraphTests, InsertNewParent) {
  DynamicSceneGraph graph({1, 2, 3});
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>()));
  const auto& child = graph.getNode(0);
  const auto& p1 = graph.getNode(2);
  const auto& p2 = graph.getNode(3);

  EXPECT_TRUE(graph.insertEdge(0, 2));
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(0, 2));
  EXPECT_TRUE(graph.hasEdge(2, 0));
  ASSERT_TRUE(child.hasParent());
  EXPECT_EQ(child.getParent().value(), 2u);
  EXPECT_TRUE(p1.children().count(0));
  EXPECT_FALSE(p2.children().count(0));

  EXPECT_TRUE(graph.insertParentEdge(0, 3, nullptr));
  ASSERT_TRUE(child.hasParent());
  EXPECT_EQ(child.getParent().value(), 3u);
  EXPECT_FALSE(p1.children().count(0));
  EXPECT_TRUE(p2.children().count(0));

  // reset the original parent (but flip argument order)
  EXPECT_TRUE(graph.insertParentEdge(2, 0, nullptr));
  ASSERT_TRUE(child.hasParent());
  EXPECT_EQ(child.getParent().value(), 2u);
  EXPECT_TRUE(p1.children().count(0));
  EXPECT_FALSE(p2.children().count(0));
}

// Test that inserting specific edge attributes works for a inter-layer edge
TEST(DynamicSceneGraphTests, EdgeAttributesCorrect) {
  DynamicSceneGraph graph({1, 2});

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  // actually add the edge
  auto info = std::make_unique<EdgeAttributes>();
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(graph.insertEdge(0, 1, std::move(info)));
  EXPECT_TRUE(graph.hasEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());

  auto edge_opt = graph.findEdge(0, 1);
  ASSERT_TRUE(edge_opt);
  const auto& edge = *edge_opt;
  EXPECT_EQ(0u, edge.source);
  EXPECT_EQ(1u, edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  auto swapped_edge_opt = graph.findEdge(1, 0);
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

// Test that removing a node meets the invariants that we expect
//   - we don't do anything if it doesn't exist
//   - we remove all edges related to the node if it does
TEST(DynamicSceneGraphTests, RemoveNodeSound) {
  DynamicSceneGraph graph({1, 2});

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
TEST(DynamicSceneGraphTests, MergeNodesCorrect) {
  DynamicSceneGraph graph({1, 2, 3});

  // we can't merge two nodes that don't exist
  EXPECT_FALSE(graph.mergeNodes(0, 1));

  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 4, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 5, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 6, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 7, std::make_unique<NodeAttributes>()));
  graph.insertEdge(0, 1);
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 3);
  graph.insertEdge(0, 2);
  graph.insertEdge(2, 5);
  graph.insertEdge(7, 3);

  // we can't merge a node with itself
  EXPECT_FALSE(graph.mergeNodes(0, 0));

  // we can't merge nodes on different layers
  EXPECT_FALSE(graph.mergeNodes(0, 1));

  // merge node 5 into node 4
  EXPECT_TRUE(graph.mergeNodes(5, 4));
  EXPECT_EQ(6u, graph.numEdges());
  EXPECT_EQ(7u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(5));
  EXPECT_TRUE(graph.hasEdge(2, 4));
  ASSERT_TRUE(graph.getNode(4).hasParent());
  EXPECT_EQ(2u, graph.getNode(4).getParent().value());
  EXPECT_EQ(std::set<NodeId>{4}, graph.getNode(2).children());

  // merge node 2 into node 0
  EXPECT_TRUE(graph.mergeNodes(2, 0));
  EXPECT_EQ(4u, graph.numEdges());
  EXPECT_EQ(6u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(2));
  EXPECT_TRUE(graph.hasEdge(0, 4));
  ASSERT_TRUE(graph.getNode(4).hasParent());
  EXPECT_EQ(0u, graph.getNode(4).getParent().value());
  EXPECT_EQ(std::set<NodeId>{4}, graph.getNode(0).children());

  // check that we clean up old interlayer edges
  EXPECT_TRUE(graph.mergeNodes(0, 6));
  EXPECT_EQ(4u, graph.numEdges());
  EXPECT_EQ(5u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(0));
  EXPECT_TRUE(graph.hasEdge(6, 4));

  // check that conflicting parents get handled
  EXPECT_TRUE(graph.mergeNodes(6, 7));
  EXPECT_EQ(4u, graph.numEdges());
  EXPECT_EQ(4u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(6));
  EXPECT_TRUE(graph.hasEdge(7, 3));
}

// Test that removeNode -> !hasNode
TEST(DynamicSceneGraphTests, RemoveNodeHasNodeCorrent) {
  DynamicSceneGraph graph({1, 2});
  const auto& layer = graph.getLayer(1);

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
TEST(DynamicSceneGraphTests, RemoveEdgeCorrect) {
  DynamicSceneGraph graph({1, 2});

  // we can't remove a node that doesn't exist
  EXPECT_FALSE(graph.removeEdge(0, 1));

  EXPECT_TRUE(graph.emplaceNode(1, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>()));

  // add and remove an intra-layer edge
  EXPECT_TRUE(graph.insertEdge(0, 2));
  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.removeEdge(0, 2));
  EXPECT_EQ(0u, graph.numEdges());

  std::vector<NodeId> nodes{0, 1, 2};
  for (const auto& node_id : nodes) {
    ASSERT_TRUE(graph.findNode(node_id));
    const auto& node = graph.getNode(node_id);
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
    ASSERT_TRUE(graph.findNode(node_id));
    const auto& node = graph.getNode(node_id);
    EXPECT_FALSE(node.hasSiblings());
    EXPECT_FALSE(node.hasParent());
    EXPECT_FALSE(node.hasChildren());
  }
}

TEST(DynamicSceneGraphTests, InsertDynamicLayerCorrect) {
  DynamicSceneGraph graph;
  EXPECT_EQ(4u, graph.numLayers());

  graph.createDynamicLayer(1, 'a');
  EXPECT_EQ(5u, graph.numLayers());
  EXPECT_EQ(1u, graph.numDynamicLayers());
  EXPECT_EQ(1u, graph.numDynamicLayersOfType(1));
  EXPECT_EQ(0u, graph.numDynamicLayersOfType(2));
  EXPECT_TRUE(graph.hasLayer(1, 'a'));
  EXPECT_FALSE(graph.hasLayer(2, 'a'));
  EXPECT_FALSE(graph.hasLayer(1, 'b'));

  graph.createDynamicLayer(1, 'b');
  EXPECT_EQ(5u, graph.numLayers());
  EXPECT_EQ(2u, graph.numDynamicLayers());
  EXPECT_EQ(2u, graph.numDynamicLayersOfType(1));
  EXPECT_EQ(0u, graph.numDynamicLayersOfType(2));
  EXPECT_TRUE(graph.hasLayer(1, 'a'));
  EXPECT_FALSE(graph.hasLayer(2, 'a'));
  EXPECT_TRUE(graph.hasLayer(1, 'b'));

  // TODO(nathan) we may rule this out: conflicting node symbols
  graph.createDynamicLayer(2, 'a');
  EXPECT_EQ(5u, graph.numLayers());
  EXPECT_EQ(3u, graph.numDynamicLayers());
  EXPECT_EQ(2u, graph.numDynamicLayersOfType(1));
  EXPECT_EQ(1u, graph.numDynamicLayersOfType(2));
  EXPECT_TRUE(graph.hasLayer(1, 'a'));
  EXPECT_TRUE(graph.hasLayer(2, 'a'));
  EXPECT_TRUE(graph.hasLayer(1, 'b'));

  // TODO(nathan) we may rule this out: conflicting node symbols
  graph.createDynamicLayer(7, 'a');
  EXPECT_EQ(6u, graph.numLayers());
  EXPECT_EQ(4u, graph.numDynamicLayers());
  EXPECT_EQ(2u, graph.numDynamicLayersOfType(1));
  EXPECT_EQ(1u, graph.numDynamicLayersOfType(2));
  EXPECT_EQ(1u, graph.numDynamicLayersOfType(7));
  EXPECT_TRUE(graph.hasLayer(1, 'a'));
  EXPECT_TRUE(graph.hasLayer(2, 'a'));
  EXPECT_TRUE(graph.hasLayer(1, 'b'));
  EXPECT_TRUE(graph.hasLayer(7, 'a'));
}

TEST(DynamicSceneGraphTests, EmplaceDynamicNodeCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  EXPECT_EQ(4u, graph.numLayers());

  EXPECT_TRUE(graph.emplaceNode(2, 'a', 1s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(graph.findNode(NodeSymbol('a', 0)));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());

  // repeat dynamic nodes don't work
  EXPECT_FALSE(graph.emplaceNode(2, 'a', 1s, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());

  // static nodes still work
  EXPECT_TRUE(
      graph.emplaceNode(2, NodeSymbol('o', 0), std::make_unique<NodeAttributes>()));
  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());
  EXPECT_TRUE(graph.hasNode(NodeSymbol('o', 0)));
  EXPECT_TRUE(graph.findNode(NodeSymbol('o', 0)));

  // repeat static nodes (of dynamic nodes) don't work
  EXPECT_FALSE(
      graph.emplaceNode(2, NodeSymbol('a', 0), std::make_unique<NodeAttributes>()));
  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());

  // repeat dynamic nodes (of static nodes) don't work
  EXPECT_FALSE(graph.emplaceNode(2, 'o', 2s, std::make_unique<NodeAttributes>()));
  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());
  EXPECT_FALSE(graph.hasLayer(2, 'o'));
}

TEST(DynamicSceneGraphTests, updateFromInvalidLayerCorrect) {
  DynamicSceneGraph graph({1, 2});

  IsolatedSceneGraphLayer separate_layer(5);
  std::unique_ptr<DynamicSceneGraph::Edges> edges(nullptr);
  EXPECT_FALSE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
}

TEST(DynamicSceneGraphTests, updateFromEmptyLayerCorrect) {
  DynamicSceneGraph graph({1, 2});

  IsolatedSceneGraphLayer separate_layer(1);
  std::unique_ptr<DynamicSceneGraph::Edges> edges(nullptr);
  EXPECT_TRUE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
}

TEST(DynamicSceneGraphTests, updateFromLayerCorrect) {
  DynamicSceneGraph graph({1, 2});
  graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 3, std::make_unique<NodeAttributes>());
  graph.insertEdge(1, 2);
  graph.insertEdge(1, 3);

  IsolatedSceneGraphLayer separate_layer(1);
  separate_layer.emplaceNode(1, std::make_unique<NodeAttributes>());
  separate_layer.emplaceNode(5, std::make_unique<NodeAttributes>());
  auto edges = std::make_unique<DynamicSceneGraph::Edges>();
  edges->emplace(std::piecewise_construct,
                 std::forward_as_tuple(5, 2),
                 std::forward_as_tuple(5, 2, std::make_unique<EdgeAttributes>()));
  edges->emplace(std::piecewise_construct,
                 std::forward_as_tuple(1, 2),
                 std::forward_as_tuple(1, 2, std::make_unique<EdgeAttributes>()));

  EXPECT_TRUE(graph.updateFromLayer(separate_layer, std::move(edges)));
  EXPECT_EQ(4u, graph.numNodes());
  EXPECT_EQ(3u, graph.numEdges());
  EXPECT_EQ(0u, separate_layer.numNodes());
  EXPECT_EQ(0u, separate_layer.numEdges());
  EXPECT_TRUE(graph.hasNode(5));
  EXPECT_TRUE(graph.hasEdge(5, 2));
}

TEST(DynamicSceneGraphTests, updateFromLayerWithSiblings) {
  DynamicSceneGraph graph({1});
  graph.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  graph.emplaceNode(1, 2, std::make_unique<NodeAttributes>());
  graph.insertEdge(1, 2);

  IsolatedSceneGraphLayer separate_layer(1);
  separate_layer.emplaceNode(2, std::make_unique<NodeAttributes>());
  auto edges = std::make_unique<DynamicSceneGraph::Edges>();

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

TEST(DynamicSceneGraphTests, MergeGraphCorrect) {
  using namespace std::chrono_literals;
  const Eigen::Vector3d pos_1(1.0, 1.0, 1.0);
  const Eigen::Vector3d pos_2(2.0, 2.0, 2.0);

  DynamicSceneGraph graph_1;
  EXPECT_TRUE(graph_1.emplaceNode(2, 0, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(3, 1, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(3, 2, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(2, 8, std::make_unique<NodeAttributes>(pos_1)));
  EXPECT_TRUE(graph_1.emplaceNode(2, 'a', 0s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_1.insertEdge(0, 1));
  EXPECT_TRUE(graph_1.insertEdge(1, 2));
  EXPECT_TRUE(graph_1.insertEdge(0, 8));
  EXPECT_TRUE(graph_1.insertEdge(1, 8));

  DynamicSceneGraph graph_2;
  EXPECT_TRUE(graph_2.emplaceNode(2, 0, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(3, 1, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(3, 2, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(3, 3, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(4, 4, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(4, 5, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(2, 6, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(2, 7, std::make_unique<NodeAttributes>(pos_2)));
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 0s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.insertEdge(0, 1));
  EXPECT_TRUE(graph_2.insertEdge(1, 2));
  EXPECT_TRUE(graph_2.insertEdge(1, 3));
  EXPECT_TRUE(graph_2.insertEdge(2, 4));
  EXPECT_TRUE(graph_2.insertEdge(3, 4));
  EXPECT_TRUE(graph_2.removeEdge(1, 2));

  EXPECT_EQ(5u, graph_1.numNodes());
  EXPECT_EQ(4u, graph_1.numEdges());
  EXPECT_EQ(9u, graph_2.numNodes());
  EXPECT_EQ(4u, graph_2.numEdges());

  GraphMergeConfig config;
  config.update_archived_attributes = true;
  graph_1.mergeGraph(graph_2, config);

  EXPECT_EQ(10u, graph_1.numNodes());
  EXPECT_EQ(6u, graph_1.numEdges());
  EXPECT_EQ(9u, graph_2.numNodes());
  EXPECT_EQ(4u, graph_2.numEdges());

  // 0 and 1 have no change; they existed already
  EXPECT_LT((graph_1.getPosition(0) - pos_2).norm(), 1.0e-6);
  EXPECT_LT((graph_1.getPosition(1) - pos_2).norm(), 1.0e-6);
  // 2 has a position of 0 and delta of -1
  EXPECT_LT((graph_1.getPosition(2) - pos_2).norm(), 1.0e-6);
  // 3 has a position of 2 and delta of -1
  EXPECT_LT((graph_1.getPosition(3) - pos_2).norm(), 1.0e-6);
  // 4 and 5 have a position of 0 and delta of 0
  EXPECT_LT(graph_1.getPosition(4).norm(), 1.0e-6);
  EXPECT_LT(graph_1.getPosition(5).norm(), 1.0e-6);
  // 6 and 7 have a position of 2 and delta of -1
  EXPECT_LT((graph_1.getPosition(6) - pos_2).norm(), 1.0e-6);
  EXPECT_LT((graph_1.getPosition(7) - pos_2).norm(), 1.0e-6);
  // 8 and a(0) have no change; they existed already
  EXPECT_LT((graph_1.getPosition(8) - pos_1).norm(), 1.0e-6);
  EXPECT_LT(graph_1.getPosition(NodeSymbol('a', 0)).norm(), 1.0e-6);
}

TEST(DynamicSceneGraphTests, MergeDynamicGraphCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph_1;

  DynamicSceneGraph graph_2;
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 0s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 1s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 2s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 3s, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(2, 'a', 4s, std::make_unique<NodeAttributes>()));

  EXPECT_EQ(0u, graph_1.numNodes());
  EXPECT_EQ(0u, graph_1.numEdges());
  EXPECT_EQ(5u, graph_2.numNodes());
  EXPECT_EQ(4u, graph_2.numEdges());

  graph_1.mergeGraph(graph_2);

  EXPECT_EQ(5u, graph_2.numNodes());
  EXPECT_EQ(4u, graph_2.numEdges());
  EXPECT_EQ(graph_1.numNodes(), graph_2.numNodes());
  EXPECT_EQ(graph_1.numEdges(), graph_2.numEdges());

  EXPECT_TRUE(graph_1.hasLayer(DsgLayers::AGENTS, 'a'));
  EXPECT_TRUE(graph_1.hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(graph_1.hasNode(NodeSymbol('a', 1)));
  EXPECT_TRUE(graph_1.hasNode(NodeSymbol('a', 2)));
  EXPECT_TRUE(graph_1.hasNode(NodeSymbol('a', 3)));
  EXPECT_TRUE(graph_1.hasNode(NodeSymbol('a', 4)));
}

TEST(DynamicSceneGraphTests, ClearWithDynamicLayersCorrect) {
  DynamicSceneGraph graph;
  EXPECT_EQ(4u, graph.numLayers());

  EXPECT_TRUE(graph.emplaceNode(
      2, 'a', std::chrono::seconds(1), std::make_unique<NodeAttributes>()));
  EXPECT_EQ(1u, graph.numNodes());
  EXPECT_EQ(1u, graph.numDynamicNodes());

  graph.clear();
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_FALSE(graph.hasNode(NodeSymbol('a', 0)));
}

TEST(DynamicSceneGraphTests, HasDynamicEdgeCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  graph.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_FALSE(graph.hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));
}

TEST(DynamicSceneGraphTests, InsertDynamicEdgeCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>(), false);
  graph.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>(), false);
  graph.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  graph.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>(), false);

  EXPECT_EQ(0u, graph.numEdges());

  EXPECT_TRUE(graph.insertEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_TRUE(graph.insertEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(graph.insertEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));

  EXPECT_EQ(3u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));
}

TEST(DynamicSceneGraphTests, getPositionCorrect) {
  using namespace std::chrono_literals;

  Eigen::Vector3d expected1(1.0, 2.0, 3.0);
  Eigen::Vector3d expected2(2.0, 3.0, 4.0);

  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>(expected1));
  graph.emplaceNode(3, NodeSymbol('x', 0), std::make_unique<NodeAttributes>(expected2));

  // valid dynamic node matches expected
  Eigen::Vector3d result1 = graph.getPosition(NodeSymbol('a', 0));
  EXPECT_EQ(expected1, result1);

  // valid static nodes still also work
  Eigen::Vector3d result2 = graph.getPosition(NodeSymbol('x', 0));
  EXPECT_EQ(expected2, result2);

  // invalid static nodes still cause an exception
  try {
    graph.getPosition(NodeSymbol('x', 5));
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }

  // invalid dynamic nodes cause an exception
  try {
    graph.getPosition(NodeSymbol('a', 3));
    FAIL();
  } catch (const std::out_of_range&) {
    SUCCEED();
  }
}

void testParentRelationship(const DynamicSceneGraph& graph,
                            NodeId parent,
                            NodeId child) {
  const auto& parent_node = graph.getNode(parent);
  const auto& child_node = graph.getNode(child);

  EXPECT_TRUE(parent_node.children().count(child_node.id))
      << NodeSymbol(child).getLabel() << " is not child of "
      << NodeSymbol(parent).getLabel();

  // note that this depends on the parent id not being 0
  EXPECT_EQ(parent_node.id, child_node.getParent().value_or(0))
      << NodeSymbol(parent).getLabel() << " is not parent of "
      << NodeSymbol(child).getLabel();
}

void testSiblingRelationship(const DynamicSceneGraph& graph,
                             NodeId source,
                             NodeId target) {
  const auto& source_node = graph.getNode(source);
  const auto& target_node = graph.getNode(target);

  EXPECT_TRUE(source_node.siblings().count(target_node.id))
      << NodeSymbol(target).getLabel() << " is not a sibling of "
      << NodeSymbol(source).getLabel();

  EXPECT_TRUE(target_node.siblings().count(source_node.id))
      << NodeSymbol(source).getLabel() << " is not a sibling of "
      << NodeSymbol(target).getLabel();
}

TEST(DynamicSceneGraphTests, InsertMixedEdgeCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, NodeSymbol('x', 0), std::make_unique<NodeAttributes>());

  EXPECT_EQ(0u, graph.numEdges());

  EXPECT_TRUE(graph.insertEdge(NodeSymbol('a', 0), NodeSymbol('x', 0)));

  EXPECT_EQ(1u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('a', 0), NodeSymbol('x', 0)));
  testParentRelationship(graph, NodeSymbol('x', 0), NodeSymbol('a', 0));

  graph.emplaceNode(4, 'b', 10ns, std::make_unique<NodeAttributes>());

  EXPECT_TRUE(graph.insertEdge(NodeSymbol('b', 0), NodeSymbol('x', 0)));
  EXPECT_EQ(2u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('b', 0), NodeSymbol('x', 0)));
  testParentRelationship(graph, NodeSymbol('b', 0), NodeSymbol('x', 0));

  graph.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>(), false);
  EXPECT_TRUE(graph.insertEdge(NodeSymbol('b', 0), NodeSymbol('a', 1)));
  EXPECT_EQ(3u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('b', 0), NodeSymbol('a', 1)));
  testParentRelationship(graph, NodeSymbol('b', 0), NodeSymbol('a', 1));

  graph.emplaceNode(2, 'c', 10ns, std::make_unique<NodeAttributes>());
  EXPECT_TRUE(graph.insertEdge(NodeSymbol('c', 0), NodeSymbol('a', 0)));
  EXPECT_EQ(4u, graph.numEdges());
  EXPECT_TRUE(graph.hasEdge(NodeSymbol('c', 0), NodeSymbol('a', 0)));
  testSiblingRelationship(graph, NodeSymbol('c', 0), NodeSymbol('a', 0));
}

TEST(DynamicSceneGraphTests, GetLayerKeyCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, NodeSymbol('x', 0), std::make_unique<NodeAttributes>());

  LayerPrefix prefix('a');
  EXPECT_EQ(*graph.getLayerForNode(NodeSymbol('a', 0)), LayerKey(2, prefix));
  EXPECT_EQ(*graph.getLayerForNode(NodeSymbol('x', 0)), LayerKey(3));

  EXPECT_FALSE(graph.getLayerForNode(NodeSymbol('b', 0)));
}

TEST(DynamicSceneGraphTests, RemovedAndNewNodesCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, NodeSymbol('x', 0), std::make_unique<NodeAttributes>());

  {
    std::vector<NodeId> expected{NodeSymbol('x', 0), NodeSymbol('a', 0)};
    std::vector<NodeId> removed = graph.getNewNodes(true);
    EXPECT_EQ(expected, removed);
  }

  {
    std::vector<NodeId> expected;
    std::vector<NodeId> removed = graph.getNewNodes(true);
    EXPECT_EQ(expected, removed);
  }

  graph.removeNode(NodeSymbol('a', 0));
  graph.removeNode(NodeSymbol('x', 0));

  {
    std::vector<NodeId> expected{NodeSymbol('x', 0), NodeSymbol('a', 0)};
    std::vector<NodeId> removed = graph.getRemovedNodes(true);
    EXPECT_EQ(expected, removed);
  }

  {
    std::vector<NodeId> expected;
    std::vector<NodeId> removed = graph.getRemovedNodes(true);
    EXPECT_EQ(expected, removed);
  }
}

TEST(DynamicSceneGraphTests, RemovedAndNewEdgesCorrect) {
  using namespace std::chrono_literals;
  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 'a', 11ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, "x0"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(4, "y1"_id, std::make_unique<NodeAttributes>());
  graph.insertEdge("x0"_id, "y1"_id);
  graph.insertEdge("a1"_id, "x0"_id);

  {
    std::vector<EdgeKey> new_expected{
        {"a0"_id, "a1"_id}, {"x0"_id, "y1"_id}, {"a1"_id, "x0"_id}};
    std::vector<EdgeKey> new_edges = graph.getNewEdges(false);
    EXPECT_EQ(new_expected, new_edges);

    std::vector<EdgeKey> removed_expected;
    std::vector<EdgeKey> removed_edges = graph.getRemovedEdges(false);
    EXPECT_EQ(removed_expected, removed_edges);
  }

  graph.removeEdge("x0"_id, "y1"_id);

  {
    std::vector<EdgeKey> new_expected{{"a0"_id, "a1"_id}, {"a1"_id, "x0"_id}};
    std::vector<EdgeKey> new_edges = graph.getNewEdges(false);
    EXPECT_EQ(new_expected, new_edges);

    std::vector<EdgeKey> removed_expected{{"x0"_id, "y1"_id}};
    std::vector<EdgeKey> removed_edges = graph.getRemovedEdges(false);
    EXPECT_EQ(removed_expected, removed_edges);
  }
}

TEST(DynamicSceneGraphTests, MergeGraphCorrectWithPrevMerges) {
  // graph 1: merged version of graph 2 (where 2 is merged into 1)
  std::map<NodeId, NodeId> prev_merges{{2, 1}};
  DynamicSceneGraph graph_1;
  EXPECT_TRUE(graph_1.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_1.emplaceNode(3, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_1.emplaceNode(3, 1, std::make_unique<NodeAttributes>()));
  graph_1.mergeNodes(2, 1);

  // graph 2: parent between 0 and 2 (but 2 goes to 1 in graph_1)
  DynamicSceneGraph graph_2;
  EXPECT_TRUE(graph_2.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(3, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.emplaceNode(3, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph_2.insertEdge(0, 2));

  EXPECT_EQ(2u, graph_1.numNodes());
  EXPECT_EQ(0u, graph_1.numEdges());
  EXPECT_EQ(3u, graph_2.numNodes());
  EXPECT_EQ(1u, graph_2.numEdges());

  graph_1.mergeGraph(graph_2);

  EXPECT_EQ(2u, graph_1.numNodes());
  EXPECT_EQ(0u, graph_1.numEdges());
  EXPECT_EQ(3u, graph_2.numNodes());
  EXPECT_EQ(1u, graph_2.numEdges());
  EXPECT_FALSE(graph_1.hasEdge(0, 1));

  GraphMergeConfig config;
  config.previous_merges = &prev_merges;
  graph_1.mergeGraph(graph_2, config);

  EXPECT_EQ(2u, graph_1.numNodes());
  EXPECT_EQ(1u, graph_1.numEdges());
  EXPECT_EQ(3u, graph_2.numNodes());
  EXPECT_EQ(1u, graph_2.numEdges());
  EXPECT_TRUE(graph_1.hasEdge(0, 1));
}

TEST(DynamicSceneGraphTests, CloneCorrect) {
  using namespace std::chrono_literals;

  DynamicSceneGraph graph;
  graph.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(2, 'a', 11ns, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, "x0"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(3, "x1"_id, std::make_unique<NodeAttributes>());
  graph.emplaceNode(4, "y1"_id, std::make_unique<NodeAttributes>());
  graph.insertEdge("x0"_id, "x1"_id);
  graph.insertEdge("x0"_id, "y1"_id);
  graph.insertEdge("a1"_id, "x0"_id);

  auto clone = graph.clone();
  ASSERT_TRUE(clone != nullptr);
  EXPECT_EQ(clone->numNodes(), graph.numNodes());
  EXPECT_EQ(clone->numEdges(), graph.numEdges());
  EXPECT_TRUE(clone->hasNode("a0"_id));
  EXPECT_TRUE(clone->hasNode("a1"_id));
  EXPECT_TRUE(clone->hasNode("x0"_id));
  EXPECT_TRUE(clone->hasNode("x1"_id));
  EXPECT_TRUE(clone->hasNode("y1"_id));
  EXPECT_TRUE(clone->hasEdge("x0"_id, "x1"_id));
  EXPECT_TRUE(clone->hasEdge("x0"_id, "y1"_id));
  EXPECT_TRUE(clone->hasEdge("a1"_id, "x0"_id));
  EXPECT_TRUE(clone->hasEdge("a0"_id, "a1"_id));
}

}  // namespace spark_dsg
