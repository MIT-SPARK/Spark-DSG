#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph_node.h>

using namespace kimera;

// Check that a default node has no edges
TEST(SceneGraphNodeTests, DefaultNodeIvariants) {
  NodeAttributes::Ptr attrs = std::make_unique<NodeAttributes>();
  SceneGraphNode a(0, 0, std::move(attrs));
  EXPECT_FALSE(a.hasSiblings());
  EXPECT_FALSE(a.hasChildren());
  EXPECT_FALSE(a.hasParent());
}

TEST(SceneGraphNodeTests, SymbolCorrect) {
  // test a hardcoded symbol
  NodeSymbol test1(static_cast<char>(0), 5);
  EXPECT_EQ(5u, test1) << "Symbol value: " << static_cast<NodeId>(test1);

  // test the same symbol directly from an int
  test1 = NodeSymbol(5);
  EXPECT_EQ(5u, test1) << "Symbol value: " << static_cast<NodeId>(test1);

  // test something slightly more complicated
  NodeSymbol test2('P', 1);
  NodeId expected = (static_cast<NodeId>('P') << 56) + 1u;
  EXPECT_EQ(expected, static_cast<NodeId>(test2))
      << "Symbol value: " << static_cast<NodeId>(test2);

  // test that pre-increment works as expected
  expected++;
  ++test2;
  EXPECT_EQ(expected, static_cast<NodeId>(test2))
      << "Symbol value: " << static_cast<NodeId>(test2);

  // test that post-increment works as expected
  NodeSymbol test3 = test2++;
  EXPECT_EQ(expected, static_cast<NodeId>(test3))
      << "Symbol value: " << static_cast<NodeId>(test3);
  expected++;
  EXPECT_EQ(expected, static_cast<NodeId>(test2))
      << "Symbol value: " << static_cast<NodeId>(test2);
}
