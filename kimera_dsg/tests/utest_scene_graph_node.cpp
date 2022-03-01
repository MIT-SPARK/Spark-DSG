#include <kimera_dsg/scene_graph_node.h>

#include <gtest/gtest.h>

using namespace kimera;

// Check that a default node has no edges
TEST(SceneGraphNodeTests, DefaultNodeIvariants) {
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>();
  SceneGraphNode a(0, 0, std::move(attrs));
  EXPECT_FALSE(a.hasSiblings());
  EXPECT_FALSE(a.hasChildren());
  EXPECT_FALSE(a.hasParent());
}
