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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph.h>
#include <spark_dsg/scene_graph_utilities.h>

namespace spark_dsg {

struct AncestorTestConfig {
  NodeId query;
  size_t depth;
  std::vector<NodeId> expected;
};

std::ostream& operator<<(std::ostream& out, const AncestorTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).str() << ", depth: " << config.depth
      << ", expected: " << displayNodeSymbolContainer(config.expected) << "}";
  return out;
}

struct BoundingBoxTestConfig {
  NodeId query;
  size_t depth;
  BoundingBox expected;
};

std::ostream& operator<<(std::ostream& out, const BoundingBoxTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).str() << ", depth: " << config.depth
      << ", expected: " << config.expected << "}";
  return out;
}

template <typename Config>
struct SgUtilitiesFixture : public testing::TestWithParam<Config> {
  SgUtilitiesFixture() {}

  virtual ~SgUtilitiesFixture() = default;

  void SetUp() override {
    graph.emplaceNode(4, 0, std::make_unique<NodeAttributes>());
    graph.emplaceNode(4, 1, std::make_unique<NodeAttributes>());
    graph.emplaceNode(3, 2, std::make_unique<NodeAttributes>());
    graph.emplaceNode(3, 3, std::make_unique<NodeAttributes>());
    graph.emplaceNode(2, 4, std::make_unique<NodeAttributes>(Eigen::Vector3d(1, 1, 1)));
    graph.emplaceNode(2, 5, std::make_unique<NodeAttributes>(Eigen::Vector3d(2, 2, 2)));
    graph.emplaceNode(2, 6, std::make_unique<NodeAttributes>(Eigen::Vector3d(3, 3, 3)));
    graph.emplaceNode(2, 7, std::make_unique<NodeAttributes>(Eigen::Vector3d(4, 4, 4)));
    graph.insertEdge(0, 2);
    graph.insertEdge(1, 3);
    graph.insertEdge(2, 4);
    graph.insertEdge(2, 5);
    graph.insertEdge(3, 6);
    graph.insertEdge(3, 7);
  }

  SceneGraph graph;
};

using AncestorTestFixture = SgUtilitiesFixture<AncestorTestConfig>;
using BoundingBoxTestFixture = SgUtilitiesFixture<BoundingBoxTestConfig>;

TEST_P(AncestorTestFixture, ResultCorrect) {
  AncestorTestConfig config = GetParam();

  std::vector<NodeId> ancestors;
  getNodeAncestorsAtDepth(
      graph, config.query, config.depth, [&](const SceneGraph&, const NodeId node) {
        ancestors.push_back(node);
      });

  EXPECT_EQ(ancestors, config.expected);
}

const AncestorTestConfig ancestor_test_cases[] = {
    {0, 0, {}},
    {0, 1, {2}},
    {0, 2, {4, 5}},
    {1, 0, {}},
    {1, 1, {3}},
    {1, 2, {6, 7}},
    {2, 0, {}},
    {2, 1, {4, 5}},
    {3, 0, {}},
    {3, 1, {6, 7}},
};

INSTANTIATE_TEST_SUITE_P(GetAncestors,
                         AncestorTestFixture,
                         testing::ValuesIn(ancestor_test_cases));

TEST_P(BoundingBoxTestFixture, BoundingBoxCorrect) {
  const BoundingBoxTestConfig config = GetParam();
  const auto bbox = computeAncestorBoundingBox(graph, config.query, config.depth);
  EXPECT_EQ(bbox, config.expected);
}

const BoundingBoxTestConfig bbox_test_cases[] = {
    {0, 0, {}},
    {1, 0, {}},
    {2, 0, {}},
    {3, 0, {}},
    {0, 2, {{1, 1, 1}, {1.5, 1.5, 1.5}}},
    {1, 2, {{1, 1, 1}, {3.5, 3.5, 3.5}}},
};

INSTANTIATE_TEST_SUITE_P(GetChildBoundingBox,
                         BoundingBoxTestFixture,
                         testing::ValuesIn(bbox_test_cases));

TEST(SceneGraphUtilities, ImageFoldersResolveAndRemap) {
  SceneGraph graph;
  auto agent = std::make_unique<AgentNodeAttributes>();
  agent->image_folder = "agents/agent_1";
  graph.emplaceNode(2, NodeSymbol('a', 0), std::move(agent), 'a');

  auto subframe = std::make_unique<SubKeyframeNodeAttributes>();
  subframe->image_folder = "subkeyframes/subkf_1";
  graph.emplaceNode(2, NodeSymbol('s', 0), std::move(subframe), 's');

  auto object = std::make_unique<KhronosObjectAttributes>();
  object->image_folder = "/old/run/images/O_1";
  graph.emplaceNode(2, NodeSymbol('O', 1), std::move(object));

  auto other = std::make_unique<KhronosObjectAttributes>();
  other->image_folder = "/old/run2/images/O_2";
  graph.emplaceNode(2, NodeSymbol('O', 2), std::move(other));

  graph.emplaceNode(3, NodeSymbol('p', 0), std::make_unique<NodeAttributes>());

  const auto folder = [&](NodeSymbol id) -> std::string {
    const auto& attrs = graph.getNode(id).attributes();
    if (auto agent = dynamic_cast<const AgentNodeAttributes*>(&attrs)) {
      return agent->image_folder;
    }
    if (auto subframe = dynamic_cast<const SubKeyframeNodeAttributes*>(&attrs)) {
      return subframe->image_folder;
    }
    return dynamic_cast<const KhronosObjectAttributes&>(attrs).image_folder;
  };

  // only relative folders change
  EXPECT_EQ(resolveImageFolders(graph, "/new/run/"), 2u);
  EXPECT_EQ(folder(NodeSymbol('a', 0)), "/new/run/agents/agent_1");
  EXPECT_EQ(folder(NodeSymbol('s', 0)), "/new/run/subkeyframes/subkf_1");
  EXPECT_EQ(folder(NodeSymbol('O', 1)), "/old/run/images/O_1");

  // prefixes only match whole path components
  EXPECT_EQ(remapImageFolders(graph, "/old/run", "/moved"), 1u);
  EXPECT_EQ(folder(NodeSymbol('O', 1)), "/moved/images/O_1");
  EXPECT_EQ(folder(NodeSymbol('O', 2)), "/old/run2/images/O_2");
  EXPECT_EQ(remapImageFolders(graph, "/new/run", "/elsewhere/"), 2u);
  EXPECT_EQ(folder(NodeSymbol('a', 0)), "/elsewhere/agents/agent_1");

  // the filesystem root is a valid prefix
  EXPECT_EQ(remapImageFolders(graph, "/", "/mnt"), 4u);
  EXPECT_EQ(folder(NodeSymbol('O', 2)), "/mnt/old/run2/images/O_2");
  EXPECT_EQ(remapImageFolders(graph, "/mnt/old", "/"), 1u);
  EXPECT_EQ(folder(NodeSymbol('O', 2)), "/run2/images/O_2");
}

}  // namespace spark_dsg
