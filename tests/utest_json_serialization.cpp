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

TEST(SceneGraphSerializationTests, SerializeDsgBasic) {
  DynamicSceneGraph expected({1, 2, 3});
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  const auto output = expected.serializeToJson();

  auto result = DynamicSceneGraph::deserializeFromJson(output);

  EXPECT_EQ(expected.numNodes(), result->numNodes()) << output;
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(1));
  EXPECT_TRUE(result->hasNode(2));
  EXPECT_TRUE(result->hasEdge(0, 1));
  EXPECT_TRUE(result->hasEdge(1, 2));
  EXPECT_EQ(expected.hasLayer(0), result->hasLayer(0));
}

TEST(SceneGraphSerializationTests, SerializeDsgWithNaNs) {
  DynamicSceneGraph expected({1, 2, 3});
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());
  Eigen::Vector3d bad_pos = Eigen::Vector3d::Zero();
  bad_pos(0) = std::numeric_limits<double>::quiet_NaN();
  bad_pos(1) = std::numeric_limits<double>::quiet_NaN();
  bad_pos(2) = std::numeric_limits<double>::quiet_NaN();
  expected.emplaceNode(3, 3, std::make_unique<NodeAttributes>(bad_pos));

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);
  expected.insertEdge(2, 3);

  const std::string output_str = expected.serializeToJson();

  auto result = DynamicSceneGraph::deserializeFromJson(output_str);

  EXPECT_EQ(expected.numNodes(), result->numNodes());
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(1));
  EXPECT_TRUE(result->hasNode(2));
  EXPECT_TRUE(result->hasNode(3));
  EXPECT_TRUE(result->hasEdge(0, 1));
  EXPECT_TRUE(result->hasEdge(1, 2));
  EXPECT_TRUE(result->hasEdge(2, 3));
  EXPECT_EQ(expected.hasLayer(0), result->hasLayer(0));
}

TEST(SceneGraphSerializationTests, SerializeDsgDynamic) {
  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  const auto output = expected.serializeToJson();

  auto result = DynamicSceneGraph::deserializeFromJson(output);

  EXPECT_EQ(expected.numNodes(), result->numNodes()) << output;
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 1)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 3)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_FALSE(result->hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));

  EXPECT_TRUE(result->hasLayer(2, 'a'));
}

TEST(SceneGraphSerializationTests, SerializeDsgMesh) {
  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  auto mesh = std::make_shared<Mesh>();
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->colors.push_back({10, 20, 30, 255});
  mesh->labels.push_back(2);
  mesh->labels.push_back(8);
  mesh->stamps.push_back(0);
  mesh->stamps.push_back(10);
  mesh->stamps.push_back(20);
  mesh->stamps.push_back(30);
  mesh->faces.push_back({{1, 2, 3}});
  expected.setMesh(mesh);

  const auto output = expected.serializeToJson(true);

  auto result = DynamicSceneGraph::deserializeFromJson(output);

  EXPECT_EQ(expected.numNodes(), result->numNodes()) << output;
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 1)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 3)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_FALSE(result->hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));

  EXPECT_TRUE(result->hasLayer(2, 'a'));
  auto result_mesh = result->mesh();
  ASSERT_TRUE(result_mesh);
  EXPECT_EQ(result_mesh->points.size(), 3u);
  EXPECT_EQ(result_mesh->colors.size(), 1u);
  EXPECT_EQ(result_mesh->labels.size(), 2u);
  EXPECT_EQ(result_mesh->stamps.size(), 4u);
  EXPECT_EQ(result_mesh->faces.size(), 1u);
}

}  // namespace spark_dsg
