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
#include <spark_dsg/dynamic_scene_graph_layer.h>

namespace spark_dsg {

class TestableDynamicLayer : public DynamicSceneGraphLayer {
 public:
  TestableDynamicLayer(spark_dsg::LayerId id, char prefix)
      : DynamicSceneGraphLayer(id, prefix) {}

  using DynamicSceneGraphLayer::emplaceNode;

  using DynamicSceneGraphLayer::removeNode;
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

  EXPECT_FALSE(layer.findNodeByIndex(NodeSymbol(layer_prefix + 1, 0)));
  EXPECT_TRUE(layer.findNodeByIndex(0));
  auto node_opt = layer.findNode(NodeSymbol(layer_prefix, 0));
  ASSERT_TRUE(node_opt);
  const auto& node = *node_opt;
  EXPECT_EQ(1u, node.layer);
  EXPECT_EQ(NodeSymbol(layer_prefix, 0), node.id);
  EXPECT_TRUE(node.timestamp);
  EXPECT_NEAR(1.0, std::chrono::duration<double>(*node.timestamp).count(), 1.0e-9);

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
  using namespace std::chrono_literals;
  const char prefix = 'a';

  TestableDynamicLayer layer(1, prefix);
  EXPECT_EQ(0u, layer.numEdges());
  EXPECT_FALSE(layer.hasEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_FALSE(layer.hasEdgeByIndex(0, 1));

  // source node
  EXPECT_TRUE(layer.emplaceNode(1s, std::make_unique<NodeAttributes>()));
  EXPECT_FALSE(layer.hasEdgeByIndex(0, 1));
  EXPECT_FALSE(layer.insertEdgeByIndex(0, 1));
  EXPECT_FALSE(layer.insertEdge(NodeSymbol(prefix, 0), NodeSymbol(prefix, 1)));
  EXPECT_EQ(0u, layer.numEdges());

  // target node (produces an edge)
  EXPECT_TRUE(layer.emplaceNode(2s, std::make_unique<NodeAttributes>()));
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
  EXPECT_TRUE(layer.emplaceNode(3s, std::make_unique<NodeAttributes>(), false));
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
  using namespace std::chrono_literals;

  TestableDynamicLayer layer(1, 'a');
  // source and target nodes
  EXPECT_TRUE(layer.emplaceNode(1s, std::make_unique<NodeAttributes>(), false));
  EXPECT_TRUE(layer.emplaceNode(2s, std::make_unique<NodeAttributes>(), false));

  // actually add the edge
  auto info = std::make_unique<EdgeAttributes>();
  info->weighted = true;
  info->weight = 0.5;
  EXPECT_TRUE(layer.insertEdgeByIndex(0, 1, std::move(info)));
  EXPECT_TRUE(layer.hasEdgeByIndex(0, 1));
  EXPECT_EQ(1u, layer.numEdges());

  EXPECT_TRUE(layer.findEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  auto edge_opt = layer.findEdgeByIndex(0, 1);
  ASSERT_TRUE(edge_opt);
  const auto& edge = *edge_opt;
  EXPECT_EQ(NodeSymbol('a', 0), edge.source);
  EXPECT_EQ(NodeSymbol('a', 1), edge.target);
  ASSERT_TRUE(edge.info != nullptr);
  EXPECT_TRUE(edge.info->weighted);
  EXPECT_EQ(0.5, edge.info->weight);

  auto swapped_edge_opt = layer.findEdgeByIndex(1, 0);
  ASSERT_TRUE(swapped_edge_opt);

  const auto& swapped_edge = *swapped_edge_opt;
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

  // nodes may be stored unordered in the future
  std::set<NodeId> actual_targets;
  for (const auto& id_edge_pair : layer.edges()) {
    const auto& edge = id_edge_pair.second;
    ASSERT_TRUE(edge.info != nullptr);
    EXPECT_EQ(edge.source + 1, edge.target);
    actual_targets.insert(edge.target);
  }

  std::set<NodeId> expected_targets{1, 2, 3, 4};
  EXPECT_EQ(expected_targets, actual_targets);
}

TEST(DynamicSceneGraphLayerTests, getPositionCorrect) {
  using namespace std::chrono_literals;
  Eigen::Vector3d expected;
  expected << 1.0, 2.0, 3.0;
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>(expected);

  TestableDynamicLayer layer(1, 0);
  layer.emplaceNode(1s, std::move(attrs));

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
    EXPECT_TRUE(layer_1.emplaceNode(std::chrono::seconds(i),
                                    std::make_unique<NodeAttributes>(node_pos)));
  }

  for (size_t i = 0; i < 5; ++i) {
    Eigen::Vector3d node_pos;
    node_pos << static_cast<double>(i + 10), 0.0, 0.0;
    EXPECT_TRUE(layer_2.emplaceNode(std::chrono::seconds(i),
                                    std::make_unique<NodeAttributes>(node_pos)));
  }

  GraphMergeConfig config;
  std::map<NodeId, LayerKey> node_to_layer;
  layer_1.mergeLayer(layer_2, config, &node_to_layer);

  EXPECT_EQ(2u, node_to_layer.size());
  EXPECT_EQ(5u, layer_1.numNodes());
  EXPECT_EQ(4u, layer_1.numEdges());

  for (size_t i = 0; i < 5; i++) {
    Eigen::Vector3d result = layer_1.getPosition(i);
    EXPECT_EQ(static_cast<double>(i), result(0));
    EXPECT_EQ(0.0, result(1));
    EXPECT_EQ(0.0, result(2));
    if (i > 2) {
      EXPECT_EQ(1u, node_to_layer.at(i).layer);
      EXPECT_EQ(0, node_to_layer.at(i).prefix);
    }
  }
}

TEST(SceneGraphLayerTests, GetRemovedNodes) {
  using namespace std::chrono_literals;
  TestableDynamicLayer layer(1, 'a');
  layer.emplaceNode(1s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(2s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(3s, std::make_unique<NodeAttributes>());

  {
    std::vector<NodeId> removed_nodes;
    layer.getRemovedNodes(removed_nodes, false);
    EXPECT_EQ(std::vector<NodeId>(), removed_nodes);
  }

  layer.removeNode(NodeSymbol('a', 0));

  {
    std::vector<NodeId> removed_nodes;
    layer.getRemovedNodes(removed_nodes, false);

    std::vector<NodeId> expected{NodeSymbol('a', 0)};
    EXPECT_EQ(expected, removed_nodes);
  }

  layer.emplaceNode(1s, std::make_unique<NodeAttributes>());

  {
    std::vector<NodeId> removed_nodes;
    layer.getRemovedNodes(removed_nodes, false);

    std::vector<NodeId> expected{NodeSymbol('a', 0)};
    EXPECT_EQ(expected, removed_nodes);
  }

  {
    std::vector<NodeId> removed_nodes;
    layer.getRemovedNodes(removed_nodes, true);

    std::vector<NodeId> expected{NodeSymbol('a', 0)};
    EXPECT_EQ(expected, removed_nodes);
  }

  {
    std::vector<NodeId> removed_nodes;
    layer.getRemovedNodes(removed_nodes, false);
    EXPECT_EQ(std::vector<NodeId>(), removed_nodes);
  }
}

TEST(DynamicSceneGraphLayerTests, GetNewCorrect) {
  using namespace std::chrono_literals;
  TestableDynamicLayer layer(1, 'a');
  layer.emplaceNode(1s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(2s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(3s, std::make_unique<NodeAttributes>());

  {
    std::vector<NodeId> expected{
        NodeSymbol('a', 0), NodeSymbol('a', 1), NodeSymbol('a', 2)};
    std::vector<NodeId> result;
    layer.getNewNodes(result, true);
    EXPECT_EQ(expected, result);
  }

  {
    std::vector<NodeId> expected;
    std::vector<NodeId> result;
    layer.getNewNodes(result, true);
    EXPECT_EQ(expected, result);
  }
}

TEST(DynamicSceneGraphLayerTests, NewRemovedEdgesCorrect) {
  using namespace std::chrono_literals;
  TestableDynamicLayer layer(1, 0);
  layer.emplaceNode(1s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(2s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(3s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(4s, std::make_unique<NodeAttributes>());
  layer.emplaceNode(5s, std::make_unique<NodeAttributes>());

  {
    std::vector<EdgeKey> expected{{0, 1}, {1, 2}, {2, 3}, {3, 4}};

    std::vector<EdgeKey> new_edges;
    layer.getNewEdges(new_edges, false);
    EXPECT_EQ(new_edges, expected);

    std::vector<EdgeKey> removed;
    layer.getRemovedEdges(removed, false);
    EXPECT_EQ(removed, std::vector<EdgeKey>());
  }

  layer.removeNode(2);
  layer.removeEdge(0, 1);

  {
    std::vector<EdgeKey> new_expected{{1, 3}, {3, 4}};
    std::vector<EdgeKey> removed_expected{{0, 1}, {1, 2}, {2, 3}};

    std::vector<EdgeKey> new_edges;
    layer.getNewEdges(new_edges, false);
    EXPECT_EQ(new_edges, new_expected);

    std::vector<EdgeKey> removed;
    layer.getRemovedEdges(removed, false);
    EXPECT_EQ(removed, removed_expected);
  }
}

}  // namespace spark_dsg
