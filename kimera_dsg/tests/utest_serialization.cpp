#include <kimera_dsg/attribute_serialization.h>
#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/scene_graph.h>
#include <kimera_dsg/serialization_helpers.h>

#include <gtest/gtest.h>

namespace kimera {

template <typename Scalar>
bool quaternionsEqual(const Eigen::Quaternion<Scalar>& lhs,
                      const Eigen::Quaternion<Scalar>& rhs) {
  return lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() &&
         lhs.z() == rhs.z();
}

bool operator==(const BoundingBox& lhs, const BoundingBox& rhs) {
  if (lhs.type != rhs.type) {
    return false;
  }

  switch (lhs.type) {
    case BoundingBox::Type::INVALID:
      return true;
    case BoundingBox::Type::AABB:
      return lhs.min == rhs.min && lhs.max == rhs.max;
    case BoundingBox::Type::OBB:
      return lhs.min == rhs.min && lhs.max == rhs.max &&
             lhs.world_P_center == rhs.world_P_center &&
             quaternionsEqual(Eigen::Quaternionf(lhs.world_R_center),
                              Eigen::Quaternionf(rhs.world_R_center));
    default:
      return false;
  }
}

bool operator==(const NodeAttributes& lhs, const NodeAttributes& rhs) {
  return lhs.position == rhs.position;
}

bool operator==(const SemanticNodeAttributes& lhs, const SemanticNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label;
}

bool operator==(const ObjectNodeAttributes& lhs, const ObjectNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label && lhs.registered == rhs.registered &&
         quaternionsEqual(lhs.world_R_object, rhs.world_R_object);
}

bool operator==(const RoomNodeAttributes& lhs, const RoomNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label;
}

bool operator==(const PlaceNodeAttributes& lhs, const PlaceNodeAttributes& rhs) {
  return lhs.position == rhs.position && lhs.name == rhs.name &&
         lhs.color == rhs.color && lhs.bounding_box == rhs.bounding_box &&
         lhs.semantic_label == rhs.semantic_label && lhs.distance == rhs.distance &&
         lhs.num_basis_points == rhs.num_basis_points;
}

bool operator==(const SceneGraphEdgeInfo& lhs, const SceneGraphEdgeInfo& rhs) {
  return lhs.weighted == rhs.weighted && lhs.weight == rhs.weight;
}

TEST(SceneGraphSerializationTests, SerializeEigenVector) {
  {  // double vector
    Eigen::Vector3d expected;
    expected << 1.0, 2.0, 3.0;

    json output = expected;

    auto result = output.get<Eigen::Vector3d>();
    EXPECT_EQ(expected, result);
  }

  {  // uint8_t vector
    SemanticNodeAttributes::ColorVector expected;
    expected << 1, 2, 3;

    json output = expected;

    auto result = output.get<SemanticNodeAttributes::ColorVector>();
    EXPECT_EQ(expected, result);
  }
}

TEST(SceneGraphSerializationTests, SerializeEigenQuaternion) {
  std::stringstream ss;

  {  // single-precision
    Eigen::Quaternionf expected(0.0, 0.0, 1.0, 0.0);
    json output = expected;
    auto result = output.get<Eigen::Quaternionf>();
    ASSERT_TRUE(quaternionsEqual(expected, result));
  }

  {  // double-precision
    Eigen::Quaterniond expected(0.0, 0.0, 0.0, 1.0);
    json output = expected;
    auto result = output.get<Eigen::Quaterniond>();
    ASSERT_TRUE(quaternionsEqual(expected, result));
  }
}

TEST(SceneGraphSerializationTests, SerializeBoundingBox) {
  {  // invalid type
    BoundingBox expected;

    json output = expected;
    BoundingBox result = output.get<BoundingBox>();

    EXPECT_EQ(expected, result);
  }

  {  // ABB
    Eigen::Vector3f expected_min;
    expected_min << 1.0f, 2.0f, 3.0f;
    Eigen::Vector3f expected_max;
    expected_max << 4.0f, 5.0f, 6.0f;

    BoundingBox expected(expected_min, expected_max);

    json output = expected;

    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }

  {  // OBB
    Eigen::Vector3f expected_min;
    expected_min << 1.0f, 2.0f, 3.0f;
    Eigen::Vector3f expected_max;
    expected_min << 4.0f, 5.0f, 6.0f;
    Eigen::Vector3f expected_pos;
    expected_min << 7.0f, 8.0f, 9.0f;
    Eigen::Quaternionf expected_rot(0.0, 0.0, 1.0, 0.0);

    BoundingBox expected(expected_min, expected_max, expected_pos, expected_rot);

    json output = expected;
    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }
}

TEST(SceneGraphSerializationTests, SerializeNodeAttributes) {
  NodeAttributeFactory factory = NodeAttributeFactory::Default();

  {  // base class
    NodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;

    json output = expected.toJson();

    NodeAttributes::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }

  {  // semantic attributes
    SemanticNodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;
    expected.name = "semantic_attributes";
    expected.color << 4, 5, 6;
    expected.bounding_box.type = BoundingBox::Type::AABB;
    expected.bounding_box.min << 7.0f, 8.0f, 9.0f;
    expected.bounding_box.max << 10.0f, 11.0f, 12.0f;
    expected.semantic_label = 13;

    json output = expected.toJson();

    NodeAttributes::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }

  {  // object attributes
    ObjectNodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;
    expected.name = "object_attributes";
    expected.color << 4, 5, 6;
    expected.bounding_box.type = BoundingBox::Type::AABB;
    expected.bounding_box.min << 7.0f, 8.0f, 9.0f;
    expected.bounding_box.max << 10.0f, 11.0f, 12.0f;
    expected.semantic_label = 13;
    expected.registered = true;
    expected.world_R_object = Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0);

    json output = expected.toJson();

    NodeAttributes::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }

  {  // semantic attributes
    RoomNodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;
    expected.name = "room_attributes";
    expected.color << 4, 5, 6;
    expected.bounding_box.type = BoundingBox::Type::AABB;
    expected.bounding_box.min << 7.0f, 8.0f, 9.0f;
    expected.bounding_box.max << 10.0f, 11.0f, 12.0f;
    expected.semantic_label = 13;

    json output = expected.toJson();

    NodeAttributes::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }

  {  // place attributes
    PlaceNodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;
    expected.name = "place_attributes";
    expected.color << 4, 5, 6;
    expected.bounding_box.type = BoundingBox::Type::AABB;
    expected.bounding_box.min << 7.0f, 8.0f, 9.0f;
    expected.bounding_box.max << 10.0f, 11.0f, 12.0f;
    expected.semantic_label = 13;
    expected.distance = 14.0;
    expected.num_basis_points = 15;

    json output = expected.toJson();
    std::cout << output << std::endl;

    NodeAttributes::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }
}

TEST(SceneGraphSerializationTests, SerializeEdgeInfo) {
  EdgeInfoFactory factory = EdgeInfoFactory::Default();

  {  // base class
    SceneGraphEdgeInfo expected;
    expected.weighted = true;
    expected.weight = 5.0;

    json output = expected.toJson();

    SceneGraphEdgeInfo::Ptr result = factory.create(output);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }
}

TEST(SceneGraphSerializationTests, SerializeGraphBasic) {
  NodeAttributeFactory attr_factory = NodeAttributeFactory::Default();
  EdgeInfoFactory info_factory = EdgeInfoFactory::Default();
  SceneGraph::JsonExportConfig config;

  SceneGraph::LayerIds layers{1, 2, 3};
  SceneGraph expected(layers);
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  json output = expected.toJson(config);

  SceneGraph result;
  result.fillFromJson(config, attr_factory, info_factory, output);

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
  NodeAttributeFactory attr_factory = NodeAttributeFactory::Default();
  EdgeInfoFactory info_factory = EdgeInfoFactory::Default();
  SceneGraph::JsonExportConfig config;

  SceneGraph::LayerIds layers{1, 2, 3};
  DynamicSceneGraph expected(layers, 0);
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  json output = expected.toJson(config);

  DynamicSceneGraph result;
  result.fillFromJson(config, attr_factory, info_factory, output);

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

}  // namespace kimera
