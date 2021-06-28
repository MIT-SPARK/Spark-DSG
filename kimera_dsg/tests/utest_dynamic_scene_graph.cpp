#include <gtest/gtest.h>
#include <kimera_dsg/dynamic_scene_graph.h>

#include <pcl/conversions.h>

namespace kimera {

using NodeRef = kimera::SceneGraph::NodeRef;
using Node = kimera::SceneGraph::Node;
using Edge = kimera::SceneGraph::Edge;
using EdgeInfo = kimera::SceneGraph::EdgeInfo;
using EdgeRef = kimera::SceneGraph::EdgeRef;

DynamicSceneGraph::Mesh::Ptr makeMesh(size_t num_points) {
  DynamicSceneGraph::Mesh::Ptr mesh;
  mesh.reset(new DynamicSceneGraph::Mesh());

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (size_t i = 0; i < num_points; ++i) {
    cloud.push_back(pcl::PointXYZRGB());
  }

  pcl::toPCLPointCloud2(cloud, mesh->cloud);

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

  DynamicSceneGraph::Mesh::Ptr mesh = makeMesh(5);

  EXPECT_EQ(2u, graph.numNodes());
  EXPECT_EQ(1u, graph.numEdges());
  graph.setMesh(mesh);
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

  DynamicSceneGraph::Mesh::Ptr mesh = makeMesh(5);
  graph.setMesh(mesh);
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

TEST(DynamicSceneGraphTest, UpdateMeshEdgesCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  DynamicSceneGraph::Mesh::Ptr mesh = makeMesh(5);
  graph.setMesh(mesh);

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  // adding more vertices doesn't invalidate anything
  mesh = makeMesh(10);
  graph.setMesh(mesh);
  EXPECT_EQ(10u, graph.numEdges());

  // adding less vertices does
  mesh = makeMesh(3);
  graph.setMesh(mesh);
  EXPECT_EQ(6u, graph.numEdges());

  // forcing invalidation works as expected
  mesh = makeMesh(5);
  graph.setMesh(mesh, true);
  EXPECT_EQ(0u, graph.numEdges());

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  // passing an invalid mesh also resets everything
  mesh.reset();
  graph.setMesh(mesh);
  EXPECT_EQ(0u, graph.numEdges());
}

TEST(DynamicSceneGraphTest, RemoveNodesCorrect) {
  DynamicSceneGraph graph;
  EXPECT_TRUE(graph.emplaceNode(2, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(graph.emplaceNode(2, 1, std::make_unique<NodeAttributes>()));

  DynamicSceneGraph::Mesh::Ptr mesh = makeMesh(5);
  graph.setMesh(mesh);

  for (size_t i = 0; i < 5; ++i) {
    graph.insertMeshEdge(0, i);
    graph.insertMeshEdge(1, i);
  }
  EXPECT_EQ(10u, graph.numEdges());

  graph.removeNode(0);
  EXPECT_EQ(5u, graph.numEdges());
}

}  // namespace kimera
