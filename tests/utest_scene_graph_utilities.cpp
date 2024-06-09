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
#include <spark_dsg/scene_graph_utilities.h>

#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

struct AncestorTestConfig {
  NodeId query;
  LayerId query_layer;
  std::vector<NodeId> expected;
};

std::ostream& operator<<(std::ostream& out, const AncestorTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).getLabel()
      << ", layer: " << config.query_layer
      << ", expected: " << displayNodeSymbolContainer(config.expected) << "}";
  return out;
}

struct BoundingBoxTestConfig {
  NodeId query;
  LayerId query_layer;
  BoundingBox expected;
};

std::ostream& operator<<(std::ostream& out, const BoundingBoxTestConfig& config) {
  out << "{query: " << NodeSymbol(config.query).getLabel()
      << ", layer: " << config.query_layer << ", expected: " << config.expected << "}";
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

  DynamicSceneGraph graph;
};

using AncestorTestFixture = SgUtilitiesFixture<AncestorTestConfig>;
using BoundingBoxTestFixture = SgUtilitiesFixture<BoundingBoxTestConfig>;

TEST_P(AncestorTestFixture, ResultCorrect) {
  AncestorTestConfig config = GetParam();

  std::vector<NodeId> ancestors;
  getAncestorsOfLayer(
      graph,
      config.query,
      config.query_layer,
      [&](const DynamicSceneGraph&, const NodeId node) { ancestors.push_back(node); });

  EXPECT_EQ(ancestors, config.expected);
}

const AncestorTestConfig ancestor_test_cases[] = {
    {0, 4, {}},
    {0, 3, {2}},
    {0, 2, {4, 5}},
    {1, 4, {}},
    {1, 3, {3}},
    {1, 2, {6, 7}},
    {2, 3, {}},
    {2, 2, {4, 5}},
    {3, 3, {}},
    {3, 2, {6, 7}},
};

INSTANTIATE_TEST_SUITE_P(GetAncestors,
                         AncestorTestFixture,
                         testing::ValuesIn(ancestor_test_cases));

TEST_P(BoundingBoxTestFixture, BoundingBoxCorrect) {
  const BoundingBoxTestConfig config = GetParam();
  const auto bbox = computeAncestorBoundingBox(graph, config.query, config.query_layer);
  EXPECT_EQ(bbox, config.expected);
}

const BoundingBoxTestConfig bbox_test_cases[] = {
    {0, 4, {}},
    {1, 4, {}},
    {2, 3, {}},
    {3, 3, {}},
    {0, 2, {{1, 1, 1}, {1.5, 1.5, 1.5}}},
    {1, 2, {{1, 1, 1}, {3.5, 3.5, 3.5}}},
};

INSTANTIATE_TEST_SUITE_P(GetChildBoundingBox,
                         BoundingBoxTestFixture,
                         testing::ValuesIn(bbox_test_cases));

}  // namespace spark_dsg
