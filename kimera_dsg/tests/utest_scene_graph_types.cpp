#include <gtest/gtest.h>
#include <kimera_dsg/scene_graph_types.h>

namespace kimera {

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
