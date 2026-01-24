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
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/edge_container.h>
#include <spark_dsg/mesh.h>
#include <spark_dsg/metadata.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph.h>
#include <spark_dsg/scene_graph_layer.h>
#include <spark_dsg/scene_graph_node.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

// TODO(lschmid): These tests are a quick sanity check and might break on different
// platforms or with some future changes.

namespace spark_dsg::tests {

TEST(MemoryUsage, Mesh) {
  Mesh mesh;
  size_t expected_size = sizeof(Mesh);  // 160 bytes
  EXPECT_EQ(mesh.memoryUsage(), expected_size);

  mesh.points.resize(1000);
  expected_size += sizeof(Mesh::Pos) * 1000;  // 12160 bytes
  EXPECT_EQ(mesh.memoryUsage(), expected_size);

  mesh.colors.resize(1000);
  expected_size += sizeof(Color) * 1000;  // 28160 bytes
  EXPECT_EQ(mesh.memoryUsage(), expected_size);
}

TEST(MemoryUsage, Metadata) {
  Metadata metadata;
  size_t expected_size = sizeof(Metadata);  // 16 bytes.
  EXPECT_EQ(metadata.memoryUsage(), expected_size);

  nlohmann::json contents;
  contents["key1"] = "value1";
  contents["key2"] = 42;
  contents["nested"] = {{"nkey1", "nvalue1"}, {"nkey2", 3.14}};
  metadata.set(contents);

  expected_size += contents.dump().size();  // 85 bytes.
  EXPECT_EQ(metadata.memoryUsage(), expected_size);
}

TEST(MemoryUsage, EdgeAttributes) {
  EdgeAttributes attrs;
  size_t expected_size = sizeof(EdgeAttributes);  // 40 bytes.
  EXPECT_EQ(attrs.memoryUsage(), expected_size);

  nlohmann::json meta_contents;
  meta_contents["description"] = "Test edge";
  meta_contents["weight"] = 2.5;
  attrs.metadata.set(meta_contents);

  expected_size += meta_contents.dump().size();  // 80 bytes
  EXPECT_EQ(attrs.memoryUsage(), expected_size);
}

TEST(MemoryUsage, EdgeContainer) {
  EdgeContainer edge_container;
  size_t expected_size = sizeof(EdgeContainer);  // 144 bytes
  EXPECT_EQ(edge_container.memoryUsage(), expected_size);

  // Add some edges.
  const auto edge_attrs = std::make_unique<EdgeAttributes>(1.0);
  for (size_t i = 0; i < 100; ++i) {
    edge_container.insert(i, i + 1, edge_attrs->clone());
  }
  expected_size += 100 * (edge_attrs->memoryUsage() + 60);
  EXPECT_EQ(edge_container.memoryUsage(), expected_size);
}

TEST(MemoryUsage, NodeAttributes) {
  NodeAttributes attrs;
  size_t expected_size = 47;
  EXPECT_EQ(attrs.memoryUsage(), expected_size);

  // Metadata.
  nlohmann::json meta_contents;
  meta_contents["type"] = "Test node";
  meta_contents["value"] = 123;
  attrs.metadata.set(meta_contents);
  expected_size += meta_contents.dump().size() - 2;  // 79 bytes
  EXPECT_EQ(attrs.memoryUsage(), expected_size);

  // Semantic node attributes.
  SemanticNodeAttributes sem_attrs;
  expected_size = 196;
  EXPECT_EQ(sem_attrs.memoryUsage(), expected_size);

  sem_attrs.name = "Test Semantic Node";
  sem_attrs.semantic_feature.resize(128, 64);
  expected_size = 41174;
  EXPECT_EQ(sem_attrs.memoryUsage(), expected_size);

  // Object noed attributes.
  ObjectNodeAttributes obj_attrs;
  expected_size = 243;
  EXPECT_EQ(obj_attrs.memoryUsage(), expected_size);
  obj_attrs.mesh_connections.resize(500);

  // NOTE(lschmid): The serialization uses up an extra byte per number for the type, so
  // a bit more than the true memory size.
  expected_size += 9 * 500;  // 4743 bytes.
  EXPECT_EQ(obj_attrs.memoryUsage(), expected_size);
}

TEST(MemoryUsage, SceneGraphNode) {
  SceneGraphNode node(1, {0, 0}, std::make_unique<NodeAttributes>());
  size_t expected_size = 231;
  EXPECT_EQ(node.memoryUsage(), expected_size);

  // Add attributes
  SceneGraphNode node2(2, {0, 0}, std::make_unique<SemanticNodeAttributes>());
  node2.attributes<SemanticNodeAttributes>().semantic_feature.resize(128, 64);
  expected_size = 41340;
  EXPECT_EQ(node2.memoryUsage(), expected_size);
}

TEST(MemoryUsage, SceneGraphLayer) {
  FAIL();
}

TEST(MemoryUsage, SceneGraph) {
  SceneGraph graph;
  size_t expected_size = 2409;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add a layer.
  graph.addLayer(1, 0, "test_layer");
  expected_size += 458;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add a partitioned layer.
  graph.addLayer(2, 1, "test_layer_partition");
  expected_size += 120;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add some nodes.
  for (size_t i = 0; i < 20; ++i) {
    graph.emplaceNode(1, i, std::make_unique<NodeAttributes>(), 0);
    graph.emplaceNode(2, 20 + i, std::make_unique<NodeAttributes>(), 1);
  }
  expected_size += 6140;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add some intralayer edges.
  for (size_t i = 0; i < 20; ++i) {
    graph.insertEdge(i, (i + 1) % 20);
    graph.insertEdge(20 + i, 20 + (i + 1) % 20);
  }
  expected_size += 40 * 58;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add some interlayer edges.
  for (size_t i = 0; i < 20; ++i) {
    graph.insertEdge(i, 20 + i);
  }
  expected_size += 20 * 108;
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add mesh.
  auto mesh = std::make_unique<Mesh>();
  mesh->points.resize(1000);
  mesh->colors.resize(1000);
  graph.setMesh(std::move(mesh));
  expected_size += 28160;  // Mesh size.
  EXPECT_EQ(graph.memoryUsage(), expected_size);

  // Add metadata.
  nlohmann::json meta_contents;
  meta_contents["creator"] = "unit_test";
  meta_contents["description"] = "Test scene graph";
  graph.metadata.set(meta_contents);
  expected_size += 56;
  EXPECT_EQ(graph.memoryUsage(), expected_size);
}

SceneGraph::Ptr populateSceneGraph(size_t size) {
  auto graph = std::make_shared<SceneGraph>();
  graph->addLayer(1, 0, "test_layer");
  graph->addLayer(2, 1, "test_layer_partition");
  auto attrs = std::make_unique<SemanticNodeAttributes>();
  attrs->semantic_feature.resize(128, 64);
  for (size_t i = 0; i < size; ++i) {
    graph->emplaceNode(1, i, attrs->clone(), 0);
    graph->emplaceNode(2, size + i, attrs->clone(), 1);
  }
  for (size_t i = 0; i < size; ++i) {
    graph->insertEdge(i, (i + 1) % size);
    graph->insertEdge(size + i, size + (i + 1) % size);
  }
  for (size_t i = 0; i < size; ++i) {
    graph->insertEdge(i, size + i);
  }
  auto mesh = std::make_unique<Mesh>();
  mesh->points.resize(10 * size);
  mesh->colors.resize(10 * size);
  graph->setMesh(std::move(mesh));
  return graph;
}

TEST(MemoryUsage, SerializationComparison) {
  for (const auto& size : {10, 100, 1000}) {
    // Serialize to a byte stream and compare with estimated memory usage.
    SCOPED_TRACE("Scene graph size: " + std::to_string(size));
    auto graph = populateSceneGraph(size);
    std::vector<uint8_t> buffer;
    io::binary::writeGraph(*graph, buffer, true);
    const double ratio =
        static_cast<double>(buffer.size()) / static_cast<double>(graph->memoryUsage());
    // NOTE(lschmid): The serialization adds some data-type overhead etc., so should be
    // larger but probably not much more than 2 (empirically rather close to 2).
    EXPECT_LT(ratio, 2.0);
    EXPECT_GT(ratio, 1.0);
  }
}

}  // namespace spark_dsg::tests
