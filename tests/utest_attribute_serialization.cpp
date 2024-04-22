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

#include "spark_dsg/serialization/binary_serializer.h"
#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/serialization/json_serialization.h"
#include "spark_dsg_tests/default_attributes.h"
#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

using nlohmann::json;

TEST(AttributeSerializationTests, NodeAttributesJson) {
  NodeAttributes expected = getNodeAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, SemanticNodeAttributesJson) {
  SemanticNodeAttributes expected = getSemanticNodeAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, ObjectNodeAttributesJson) {
  ObjectNodeAttributes expected = getObjectNodeAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, RoomNodeAttributesJson) {
  RoomNodeAttributes expected = getRoomNodeAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, PlaceNodeAttributesJson) {
  PlaceNodeAttributes expected = getPlaceNodeAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, KhronosObjectAttributesJson) {
  const auto expected = getKhronosObjectAttributes();

  const json output = expected;
  auto result = JsonNodeFactory::get_default().create(JsonConverter(&output));

  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result) << output;
}

TEST(AttributeSerializationTests, EdgeInfoJson) {
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

TEST(AttributeSerializationTests, AttributesWithNaNJson) {
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

NodeAttributes::Ptr writeAttrsRT(const NodeAttributes& expected) {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(expected);

  serialization::BinaryDeserializer deserializer(buffer);
  serialization::BinaryConverter converter(&deserializer);
  auto result = serialization::BinaryNodeFactory::get_default().create(converter);
  converter.finalize();

  return result;
}

EdgeAttributes::Ptr writeAttrsRT(const EdgeAttributes& expected) {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(expected);

  serialization::BinaryDeserializer deserializer(buffer);
  serialization::BinaryConverter converter(&deserializer);
  auto result = serialization::BinaryEdgeFactory::get_default().create(converter);
  converter.finalize();

  return result;
}

TEST(AttributeSerializationTests, NodeAttributesBinary) {
  NodeAttributes expected = getNodeAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, SemanticNodeAttributesBinary) {
  SemanticNodeAttributes expected = getSemanticNodeAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, ObjectNodeAttributesBinary) {
  ObjectNodeAttributes expected = getObjectNodeAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, RoomNodeAttributesBinary) {
  RoomNodeAttributes expected = getRoomNodeAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, PlaceNodeAttributesBinary) {
  PlaceNodeAttributes expected = getPlaceNodeAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, KhronosObjectAttributesBinary) {
  const auto expected = getKhronosObjectAttributes();
  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

TEST(AttributeSerializationTests, EdgeInfoBinary) {
  EdgeAttributes expected;
  expected.weighted = true;
  expected.weight = 5.0;

  auto result = writeAttrsRT(expected);
  ASSERT_TRUE(result != nullptr);
  EXPECT_EQ(expected, *result);
}

}  // namespace spark_dsg
