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

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/labelspace.h"

namespace spark_dsg {

TEST(Labelspace, EmptyCorrect) {
  Labelspace empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_FALSE(empty);

  Labelspace full(std::map<SemanticLabel, std::string>{{0, "wall"}, {1, "floor"}});
  EXPECT_FALSE(full.empty());
  EXPECT_TRUE(full);
}

TEST(Labelspace, InverseCorrect) {
  const std::map<SemanticLabel, std::string> label_to_names{
      {0, "wall"}, {1, "floor"}, {4, "ceiling"}, {20, "lamp"}};
  Labelspace labelspace(label_to_names);

  const std::map<std::string, SemanticLabel> expected{
      {"ceiling", 4}, {"floor", 1}, {"lamp", 20}, {"wall", 0}};
  EXPECT_EQ(labelspace.names_to_labels(), expected);
}

TEST(Labelspace, FromVectorCorrect) {
  const std::vector<std::string> labels{"floor", "ceiling", "wall", "lamp"};
  Labelspace labelspace(labels);

  const std::map<SemanticLabel, std::string> expected{
      {0, "floor"}, {1, "ceiling"}, {2, "wall"}, {3, "lamp"}};
  EXPECT_EQ(labelspace.labels_to_names(), expected);
}

TEST(Labelspace, LookupCorrect) {
  const std::map<SemanticLabel, std::string> label_to_names{
      {0, "wall"}, {1, "floor"}, {4, "ceiling"}, {20, "lamp"}};
  Labelspace labelspace(label_to_names);

  // label-based lookup should work
  EXPECT_EQ(labelspace.getCategory(-1), std::nullopt);
  EXPECT_EQ(labelspace.getCategory(5), std::nullopt);
  EXPECT_EQ(labelspace.getCategory(4), "ceiling");

  // names-based lookup should work
  EXPECT_EQ(labelspace.getLabel("door"), std::nullopt);
  EXPECT_EQ(labelspace.getLabel("wall"), 0u);
  EXPECT_EQ(labelspace.getLabel("lamp"), 20u);

  SemanticNodeAttributes attrs;
  attrs.semantic_label = 1u;
  EXPECT_EQ(labelspace.getCategory(attrs), "floor");
  attrs.semantic_label = 2u;
  EXPECT_EQ(labelspace.getCategory(attrs), "UNKNOWN");
  attrs.semantic_label = 2u;
  EXPECT_EQ(labelspace.getCategory(attrs, "custom"), "custom");
}

TEST(Labelspace, Serialization) {
  const std::map<SemanticLabel, std::string> label_to_names{
      {0, "wall"}, {1, "floor"}, {4, "ceiling"}, {20, "lamp"}};
  Labelspace labelspace(label_to_names);

  DynamicSceneGraph graph;
  labelspace.save(graph, 1, 0);
  EXPECT_FALSE(Labelspace::fromMetadata(graph, 1, 1));
  EXPECT_FALSE(Labelspace::fromMetadata(graph, 2, 0));
  EXPECT_TRUE(Labelspace::fromMetadata(graph, 1, 0));
  EXPECT_EQ(Labelspace::fromMetadata(graph, 1, 0).labels_to_names(), label_to_names);
}

TEST(Labelspace, Clone) {
  const std::map<SemanticLabel, std::string> label_to_names{
      {0, "wall"}, {1, "floor"}, {4, "ceiling"}, {20, "lamp"}};
  Labelspace labelspace(label_to_names);

  DynamicSceneGraph graph;
  labelspace.save(graph, 1, 0);
  EXPECT_FALSE(Labelspace::fromMetadata(graph, 1, 1));
  EXPECT_FALSE(Labelspace::fromMetadata(graph, 2, 0));
  EXPECT_TRUE(Labelspace::fromMetadata(graph, 1, 0));
  EXPECT_EQ(Labelspace::fromMetadata(graph, 1, 0).labels_to_names(), label_to_names);

  const auto copy = graph.clone();
  ASSERT_TRUE(copy != nullptr);
  EXPECT_FALSE(Labelspace::fromMetadata(*copy, 1, 1));
  EXPECT_FALSE(Labelspace::fromMetadata(*copy, 2, 0));
  EXPECT_TRUE(Labelspace::fromMetadata(*copy, 1, 0));
  EXPECT_EQ(Labelspace::fromMetadata(*copy, 1, 0).labels_to_names(), label_to_names);
}

}  // namespace spark_dsg
