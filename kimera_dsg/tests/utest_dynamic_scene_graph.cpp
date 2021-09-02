#include <gtest/gtest.h>
#include <kimera_dsg/dynamic_scene_graph.h>

#include <pcl/conversions.h>

namespace kimera {

using NodeRef = kimera::SceneGraph::NodeRef;
using Node = kimera::SceneGraph::Node;
using Edge = kimera::SceneGraph::Edge;
using EdgeInfo = kimera::SceneGraph::EdgeInfo;
using EdgeRef = kimera::SceneGraph::EdgeRef;

struct TestMesh {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices;
  std::shared_ptr<std::vector<pcl::Vertices>> faces;

  void reset() {
    vertices.reset();
    faces.reset();
  }
};

TestMesh makeMesh(size_t num_points) {
  TestMesh mesh;
  mesh.vertices.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  mesh.faces.reset(new std::vector<pcl::Vertices>());

  for (size_t i = 0; i < num_points; ++i) {
    mesh.vertices->push_back(pcl::PointXYZRGBA());
  }

  return mesh;
}

TEST(DynamicSceneGraphTests, DefaultConstructorInvariants) {
  DynamicSceneGraph graph;
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(5u, graph.numLayers());
}

TEST(DynamicSceneGraphTests, CustomLayerInvariants) {
  SceneGraph::LayerIds layer_ids{1, 2, 3, 4, 5};
  DynamicSceneGraph graph(layer_ids);
  EXPECT_EQ(0u, graph.numNodes());
  EXPECT_EQ(0u, graph.numEdges());
  EXPECT_EQ(layer_ids.size() + 1u, graph.numLayers());
}

TEST(DynamicSceneGraphTests, DuplicateMeshIdFails) {
  SceneGraph::LayerIds layer_ids{0, 1, 2, 3};
  try {
    DynamicSceneGraph graph(layer_ids, 0);
    FAIL();
  } catch (const std::runtime_error&) {
    SUCCEED();
  }
}

TEST(DynamicSceneGraphTest, NumNodesAndEdges) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.insertEdge(0, 1));

  TestMesh mesh = makeMesh(5);

  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(7u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());

  EXPECT_TRUE(graph.insertMeshEdge(0, 4));
  EXPECT_EQ(7u, graph.numNodes());
  EXPECT_EQ(2u, graph.numEdges());
}

TEST(DynamicSceneGraphTest, MeshEdgeInvariantsCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.insertEdge(0, 1));

  TestMesh mesh = makeMesh(5);
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(1u, graph.numEdges());

  EXPECT_TRUE(graph.insertMeshEdge(0, 1));
  EXPECT_EQ(2u, graph.numEdges());

  // no repeated edges
  EXPECT_FALSE(graph.insertMeshEdge(0, 1));
  EXPECT_EQ(2u, graph.numEdges());

  // no edges to invalid nodes
  EXPECT_FALSE(graph.insertMeshEdge(3, 1));
  EXPECT_EQ(2u, graph.numEdges());

  // no edges to invalid mesh vertices
  EXPECT_FALSE(graph.insertMeshEdge(0, 8));
  EXPECT_EQ(2u, graph.numEdges());

  // removing existing edges should work
  EXPECT_TRUE(graph.removeMeshEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());

  // removing non-existing edges shouldn't work
  EXPECT_FALSE(graph.removeMeshEdge(0, 1));
  EXPECT_EQ(1u, graph.numEdges());

  // inserting edges back should work
  EXPECT_TRUE(graph.insertMeshEdge(0, 1));
  EXPECT_EQ(2u, graph.numEdges());
}

TEST(DynamicSceneGraphTest, InvalidMeshEdgeInvariantsCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.insertEdge(0, 1));

  // edges when mesh is invalid don't work
  EXPECT_FALSE(graph.insertMeshEdge(0, 7, true));
  EXPECT_EQ(1u, graph.numEdges());

  TestMesh mesh = makeMesh(5);
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(1u, graph.numEdges());

  EXPECT_TRUE(graph.insertMeshEdge(0, 1, true));
  EXPECT_EQ(2u, graph.numEdges());

  // no repeated edges
  EXPECT_FALSE(graph.insertMeshEdge(0, 1, true));
  EXPECT_EQ(2u, graph.numEdges());

  // no edges to invalid nodes
  EXPECT_FALSE(graph.insertMeshEdge(3, 1, true));
  EXPECT_EQ(2u, graph.numEdges());

  // edges to invalid mesh vertices works
  EXPECT_TRUE(graph.insertMeshEdge(0, 8, true));
  EXPECT_EQ(3u, graph.numEdges());
}

TEST(DynamicSceneGraphTest, UpdateMeshEdgesCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  TestMesh mesh = makeMesh(5);
  graph.setMesh(mesh.vertices, mesh.faces);

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  // adding more vertices doesn't invalidate anything
  mesh = makeMesh(10);
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(10u, graph.numEdges());

  // adding less vertices does
  mesh = makeMesh(3);
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(6u, graph.numEdges());

  // forcing invalidation works as expected
  mesh = makeMesh(5);
  graph.setMesh(mesh.vertices, mesh.faces, true);
  EXPECT_EQ(0u, graph.numEdges());

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  // passing an invalid mesh also resets everything
  mesh.reset();
  graph.setMesh(mesh.vertices, mesh.faces);
  EXPECT_EQ(0u, graph.numEdges());
}

TEST(DynamicSceneGraphTest, RemoveNodesCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  TestMesh mesh = makeMesh(5);
  graph.setMesh(mesh.vertices, mesh.faces);

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  graph.removeNode(0);
  EXPECT_EQ(5u, graph.numEdges());
}

}  // namespace kimera
