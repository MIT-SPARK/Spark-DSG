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
#include <spark_dsg/attribute_factory.h>
#include <spark_dsg/graph_json_serialization.h>
#include <spark_dsg/serialization_helpers.h>

#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

using nlohmann::json;

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

    const json output = expected;
    auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

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

    const json output = expected;
    auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

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

    const json output = expected;
    auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

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

    const json output = expected;
    auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

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

    const json output = expected;
    auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result) << output;
  }
}

TEST(AttributeSerializationTests, SerializeEdgeInfo) {
  {  // base class
    EdgeAttributes expected;
    expected.weighted = true;
    expected.weight = 5.0;

    const json output = expected;
    auto result = JsonEdgeFactory::get_default().create(JsonConverter(&output));

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

  const json attr_json = json::parse(raw_json);
  auto attrs = JsonNodeFactory::get_default().create(JsonConverter(&attr_json));
  ASSERT_TRUE(attrs != nullptr);
  EXPECT_TRUE(attrs->position.hasNaN());

  std::string raw_json2 =
      R"delim({
        "position": [null, null, null],
        "type": "ObjectNodeAttributes"
      })delim";
  const json attr_json2 = json::parse(raw_json2);

  NodeAttributes new_attrs;
  attributes::deserialize(JsonConverter(&attr_json2), new_attrs);
  EXPECT_TRUE(new_attrs.position.hasNaN());
}

}  // namespace spark_dsg
