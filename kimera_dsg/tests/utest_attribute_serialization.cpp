#include <kimera_dsg/attribute_serialization.h>
#include <kimera_dsg/attribute_serializer.h>
#include <kimera_dsg/node_attributes.h>
#include <kimera_dsg/serialization_helpers.h>

#include <gtest/gtest.h>

namespace kimera {

namespace attributes {

template <typename T>
nlohmann::json to_json(const T& attributes) {
  AttributeSerializer serializer;
  attributes.serialize(serializer);
  return serializer.record;
}

}  // namespace attributes

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

bool operator==(const EdgeAttributes& lhs, const EdgeAttributes& rhs) {
  return lhs.weighted == rhs.weighted && lhs.weight == rhs.weight;
}

TEST(AttributeSerializationTests, SerializeEigenVector) {
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

  {  // dynamic float vector
    Eigen::VectorXf expected(5, 1);
    expected << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;

    json output = expected;

    auto result = output.get<Eigen::VectorXf>();
    EXPECT_EQ(expected, result);
  }

  {  // dynamic int vector
    Eigen::VectorXi expected(5, 1);
    expected << 1, 2, 3, 4, 5;

    json output = expected;

    auto result = output.get<Eigen::VectorXi>();
    EXPECT_EQ(expected, result);
  }
}

TEST(AttributeSerializationTests, SerializeEigenQuaternion) {
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

TEST(AttributeSerializationTests, SerializeBoundingBox) {
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
    expected_max << 4.0f, 5.0f, 6.0f;
    Eigen::Vector3f expected_pos;
    expected_pos << 7.0f, 8.0f, 9.0f;
    Eigen::Quaternionf expected_rot(0.0, 0.0, 1.0, 0.0);

    BoundingBox expected(expected_min, expected_max, expected_pos, expected_rot);

    json output = expected;
    BoundingBox result = output.get<BoundingBox>();
    EXPECT_EQ(expected, result);
  }
}

TEST(AttributeSerializationTests, SerializeNodeAttributes) {
  {  // base class
    NodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;

    json output = attributes::to_json(expected);
    auto result = NodeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
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

    json output = attributes::to_json(expected);
    auto result = NodeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
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

    json output = attributes::to_json(expected);
    auto result = NodeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
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

    json output = attributes::to_json(expected);
    auto result = NodeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
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

    json output = attributes::to_json(expected);
    auto result = NodeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
  }
}

TEST(AttributeSerializationTests, SerializeEdgeInfo) {
  {  // base class
    EdgeAttributes expected;
    expected.weighted = true;
    expected.weight = 5.0;

    json output = attributes::to_json(expected);
    auto result = EdgeAttributeFactory::get_default().create(output);

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }
}

TEST(AttributeSerializationTests, DeserializeAttributesWithNaN) {
  std::string raw_json =
      R"delim({
        "bounding_box": {
            "max": [-16.550033569335938, 21.749967575073242, 1.7499535083770752],
            "min": [-17.350046157836914, 21.249958038330078, 1.0030425786972046],
            "type": "AABB",
            "world_P_center": [-16.95003890991211, 21.499961853027344, 1.3764979839324951],
            "world_R_center": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        },
        "color": [39, 144, 65],
        "name": "O(0)",
        "position": [null, null, null],
        "registered": false,
        "semantic_label": 7,
        "type": "ObjectNodeAttributes",
        "world_R_object": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
      })delim";

  json attr_json = json::parse(raw_json);
  auto attrs = NodeAttributeFactory::get_default().create(attr_json);
  ASSERT_TRUE(attrs != nullptr);
  EXPECT_TRUE(attrs->position.hasNaN());

  std::string raw_json2 =
      R"delim({
        "position": [null, null, null],
        "type": "ObjectNodeAttributes"
      })delim";
  json attr_json2 = json::parse(raw_json2);

  NodeAttributes new_attrs;
  new_attrs.deserialize(AttributeSerializer(attr_json2));
  EXPECT_TRUE(new_attrs.position.hasNaN());
}

}  // namespace kimera
