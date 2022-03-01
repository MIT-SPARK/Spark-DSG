#include <kimera_dsg/attribute_serialization.h>
#include <kimera_dsg/attribute_serializer.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph.h>
#include <kimera_dsg/serialization_helpers.h>

#include <gtest/gtest.h>

#include <stdlib.h>
#include <unistd.h>

namespace kimera {

struct TempFile {
  TempFile() {
    char default_path[] = "/tmp/dsgtest.XXXXXX";
    auto fd = mkstemp(default_path);
    if (fd == -1) {
      valid = false;
      perror("mkstemp failed: ");
      return;
    } else {
      valid = true;
    }

    close(fd);

    path = std::string(default_path, sizeof(default_path));
  }

  ~TempFile() {
    if (!valid) {
      return;
    }

    if (remove(path.c_str()) != 0) {
      perror("remove failed: ");
    }
  }

  bool valid;
  std::string path;
};

TEST(SceneGraphSerializationTests, SerializeGraphBasic) {
  SceneGraph::LayerIds layers{1, 2, 3};
  SceneGraph expected(layers);
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  const auto output = expected.serialize();

  SceneGraph result;
  result.deserialize(output);

  EXPECT_EQ(expected.numNodes(), result.numNodes());
  EXPECT_EQ(expected.numEdges(), result.numEdges());
  EXPECT_EQ(expected.numLayers(), result.numLayers());

  std::set<LayerId> original_layers;
  for (const auto& id_layer_pair : expected.layers()) {
    original_layers.insert(id_layer_pair.first);
  }

  for (const auto& id_layer_pair : result.layers()) {
    EXPECT_TRUE(original_layers.count(id_layer_pair.first))
        << "layer " << id_layer_pair.first << " missing in serialized graph";
  }

  EXPECT_TRUE(result.hasNode(0));
  EXPECT_TRUE(result.hasNode(1));
  EXPECT_TRUE(result.hasNode(2));
  EXPECT_TRUE(result.hasEdge(0, 1));
  EXPECT_TRUE(result.hasEdge(1, 2));
}

TEST(SceneGraphSerializationTests, SerializeDsgBasic) {
  SceneGraph::LayerIds layers{1, 2, 3};
  DynamicSceneGraph expected(layers, 0);
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  const auto output = expected.serialize();

  DynamicSceneGraph result;
  result.deserialize(output);

  EXPECT_EQ(expected.numNodes(), result.numNodes()) << output;
  EXPECT_EQ(expected.numEdges(), result.numEdges());
  EXPECT_EQ(expected.numLayers(), result.numLayers());

  std::set<LayerId> original_layers;
  for (const auto& id_layer_pair : expected.layers()) {
    original_layers.insert(id_layer_pair.first);
  }

  for (const auto& id_layer_pair : result.layers()) {
    EXPECT_TRUE(original_layers.count(id_layer_pair.first))
        << "layer " << id_layer_pair.first << " missing in serialized graph";
  }

  EXPECT_TRUE(result.hasNode(0));
  EXPECT_TRUE(result.hasNode(1));
  EXPECT_TRUE(result.hasNode(2));
  EXPECT_TRUE(result.hasEdge(0, 1));
  EXPECT_TRUE(result.hasEdge(1, 2));
  EXPECT_EQ(expected.hasLayer(0), result.hasLayer(0));
}

TEST(SceneGraphSerializationTests, SerializeDsgWithNaNs) {
  SceneGraph::LayerIds layers{1, 2, 3};
  DynamicSceneGraph expected(layers, 0);
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

  const std::string output_str = expected.serialize();

  // TODO(nathan) make static
  DynamicSceneGraph result;
  result.deserialize(output_str);

  EXPECT_EQ(expected.numNodes(), result.numNodes());
  EXPECT_EQ(expected.numEdges(), result.numEdges());
  EXPECT_EQ(expected.numLayers(), result.numLayers());

  std::set<LayerId> original_layers;
  for (const auto& id_layer_pair : expected.layers()) {
    original_layers.insert(id_layer_pair.first);
  }

  for (const auto& id_layer_pair : result.layers()) {
    EXPECT_TRUE(original_layers.count(id_layer_pair.first))
        << "layer " << id_layer_pair.first << " missing in serialized graph";
  }

  EXPECT_TRUE(result.hasNode(0));
  EXPECT_TRUE(result.hasNode(1));
  EXPECT_TRUE(result.hasNode(2));
  EXPECT_TRUE(result.hasNode(3));
  EXPECT_TRUE(result.hasEdge(0, 1));
  EXPECT_TRUE(result.hasEdge(1, 2));
  EXPECT_TRUE(result.hasEdge(2, 3));
  EXPECT_EQ(expected.hasLayer(0), result.hasLayer(0));
}

TEST(SceneGraphSerializationTests, SerializeDsgDynamic) {
  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceDynamicNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceDynamicNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceDynamicNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceDynamicNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  const auto output = expected.serialize();

  DynamicSceneGraph result;
  result.deserialize(output);

  EXPECT_EQ(expected.numNodes(), result.numNodes()) << output;
  EXPECT_EQ(expected.numEdges(), result.numEdges());
  EXPECT_EQ(expected.numLayers(), result.numLayers());

  std::set<LayerId> original_layers;
  for (const auto& id_layer_pair : expected.layers()) {
    original_layers.insert(id_layer_pair.first);
  }

  for (const auto& id_layer_pair : result.layers()) {
    EXPECT_TRUE(original_layers.count(id_layer_pair.first))
        << "layer " << id_layer_pair.first << " missing in serialized graph";
  }

  EXPECT_TRUE(result.hasNode(0));
  EXPECT_TRUE(result.hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(result.hasNode(NodeSymbol('a', 1)));
  EXPECT_TRUE(result.hasNode(NodeSymbol('a', 2)));
  EXPECT_TRUE(result.hasNode(NodeSymbol('a', 3)));
  EXPECT_TRUE(result.hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_FALSE(result.hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(result.hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));

  EXPECT_TRUE(result.hasDynamicLayer(2, 'a'));
}

TEST(SceneGraphSerializationTests, SaveAndLoadGraph) {
  TempFile tmp_file;

  DynamicSceneGraph graph;
  graph.emplaceNode(KimeraDsgLayers::PLACES,
                    NodeSymbol('p', 0),
                    std::make_unique<NodeAttributes>(Eigen::Vector3d::Zero()));

  DynamicSceneGraph::MeshVertices fake_vertices;
  pcl::PolygonMesh fake_mesh;
  pcl::toPCLPointCloud2(fake_vertices, fake_mesh.cloud);
  graph.setMeshDirectly(fake_mesh);

  graph.save(tmp_file.path);

  DynamicSceneGraph other;
  other.load(tmp_file.path);

  EXPECT_EQ(graph.numNodes(), other.numNodes());
  EXPECT_EQ(graph.numLayers(), other.numLayers());
  EXPECT_EQ(graph.hasMesh(), other.hasMesh());
}

}  // namespace kimera
