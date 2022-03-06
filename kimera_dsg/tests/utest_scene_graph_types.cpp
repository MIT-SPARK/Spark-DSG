#include <kimera_dsg/node_symbol.h>
#include <kimera_dsg/scene_graph_types.h>

#include <gtest/gtest.h>

namespace kimera {

TEST(LayerKeyTests, TestEquality) {
  EXPECT_EQ(LayerKey(1), LayerKey(1));
  EXPECT_NE(LayerKey(1), LayerKey(2));
  EXPECT_NE(LayerKey(1), LayerKey(1, 0));
  EXPECT_EQ(LayerKey(2, 0), LayerKey(2, 0));
  EXPECT_NE(LayerKey(2, 0), LayerKey(2, 1));
}

TEST(LayerKeyTests, TestIsParent) {
  LayerKey key1{1};
  LayerKey key2{1};
  LayerKey key3{2};
  LayerKey key4{2, 0};
  LayerKey key5{3, 0};
  LayerKey key6{2, 1};

  // static
  EXPECT_TRUE(key3.isParent(key1));
  EXPECT_FALSE(key1.isParent(key2));
  EXPECT_FALSE(key1.isParent(key3));

  // dynamic
  EXPECT_TRUE(key4.isParent(key1));
  EXPECT_TRUE(key5.isParent(key6));
  EXPECT_FALSE(key6.isParent(key4));
}

TEST(LayerKeyTests, TestKeyTruthValues) {
  EXPECT_FALSE(LayerKey());
  EXPECT_TRUE(LayerKey(1));
  EXPECT_TRUE(LayerKey(2, 0));
  EXPECT_TRUE(LayerKey(0));
}

TEST(LayerPrefixTests, TestMatches) {
  LayerPrefix a('a');
  EXPECT_TRUE(a.matches(NodeSymbol('a', 0)));
  EXPECT_TRUE(a.matches(NodeSymbol('a', 5)));
  EXPECT_FALSE(a.matches(NodeSymbol('b', 5)));
}

TEST(LayerPrefixTests, TestMakeId) {
  LayerPrefix a('a');
  EXPECT_EQ(a.makeId(0), NodeSymbol('a', 0));
  EXPECT_EQ(a.makeId(5), NodeSymbol('a', 5));

  LayerPrefix b('b');
  EXPECT_EQ(b.makeId(5), NodeSymbol('b', 5));
}

TEST(LayerPrefixTests, TestIndex) {
  LayerPrefix a('a');
  EXPECT_EQ(a.index(NodeSymbol('a', 0)), 0u);
  EXPECT_EQ(a.index(NodeSymbol('a', 5)), 5u);

  LayerPrefix b('b', 1);
  EXPECT_EQ(b.index(b.makeId(5)), 5u);
}

TEST(LayerHelperTests, TestToString) {
  EXPECT_EQ("OBJECTS", KimeraDsgLayers::LayerIdToString(KimeraDsgLayers::OBJECTS));
  // layer 2 always maps to OBJECTS
  EXPECT_EQ("OBJECTS", KimeraDsgLayers::LayerIdToString(KimeraDsgLayers::AGENTS));
}

TEST(LayerHelperTests, TestToId) {
  EXPECT_EQ(KimeraDsgLayers::OBJECTS, KimeraDsgLayers::StringToLayerId("OBJECTS"));
  EXPECT_EQ(KimeraDsgLayers::AGENTS, KimeraDsgLayers::StringToLayerId("AGENTS"));
}

}  // namespace kimera
