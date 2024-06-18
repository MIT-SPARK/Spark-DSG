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

#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg_tests/default_attributes.h"
#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

using nlohmann::json;
using serialization::AttributeRegistry;
using serialization::Visitor;

template <typename T>
std::string printAttrs(const T& attrs) {
  const json output = attrs;
  return output;
}

struct NodeAttributeSerializationFixture
    : public testing::TestWithParam<std::shared_ptr<NodeAttributes>> {
  NodeAttributeSerializationFixture() {}
  virtual ~NodeAttributeSerializationFixture() = default;
};

struct EdgeAttributeSerializationFixture
    : public testing::TestWithParam<std::shared_ptr<EdgeAttributes>> {
  EdgeAttributeSerializationFixture() {}
  virtual ~EdgeAttributeSerializationFixture() = default;
};

const std::shared_ptr<NodeAttributes> node_attribute_test_cases[] = {
    std::make_shared<NodeAttributes>(getNodeAttributes()),
    std::make_shared<SemanticNodeAttributes>(getSemanticNodeAttributes()),
    std::make_shared<ObjectNodeAttributes>(getObjectNodeAttributes()),
    std::make_shared<RoomNodeAttributes>(getRoomNodeAttributes()),
    std::make_shared<PlaceNodeAttributes>(getPlaceNodeAttributes()),
    std::make_shared<KhronosObjectAttributes>(getKhronosObjectAttributes()),
};

const std::shared_ptr<EdgeAttributes> edge_attribute_test_cases[] = {
    std::make_shared<EdgeAttributes>(getEdgeAttributes()),
};

INSTANTIATE_TEST_SUITE_P(
    NodeAttributeSerialization,
    NodeAttributeSerializationFixture,
    testing::ValuesIn(node_attribute_test_cases),
    [](const testing::TestParamInfo<NodeAttributeSerializationFixture::ParamType>&
           info) { return info.param ? info.param->registration().name : "Invalid"; });

INSTANTIATE_TEST_SUITE_P(
    EdgeAttributeSerialization,
    EdgeAttributeSerializationFixture,
    testing::ValuesIn(edge_attribute_test_cases),
    [](const testing::TestParamInfo<EdgeAttributeSerializationFixture::ParamType>&
           info) { return info.param ? info.param->registration().name : "Invalid"; });

TEST_P(NodeAttributeSerializationFixture, JsonSerialization) {
  const auto expected = GetParam();
  ASSERT_TRUE(expected);

  const json output = *expected;
  auto result = Visitor::from(AttributeRegistry<NodeAttributes>::current(), output);

  ASSERT_TRUE(result);
  EXPECT_EQ(*expected, *result)
      << "expected: " << printAttrs(*expected) << ", result: " << printAttrs(*result);
}

TEST_P(EdgeAttributeSerializationFixture, JsonSerialization) {
  const auto expected = GetParam();
  ASSERT_TRUE(expected);

  const json output = *expected;
  auto result = Visitor::from(AttributeRegistry<EdgeAttributes>::current(), output);

  ASSERT_TRUE(result);
  EXPECT_EQ(*expected, *result)
      << "expected: " << printAttrs(*expected) << ", result: " << printAttrs(*result);
}

TEST_P(NodeAttributeSerializationFixture, BinarySerialization) {
  const auto expected = GetParam();
  ASSERT_TRUE(expected);

  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(*expected);

  serialization::BinaryDeserializer deserializer(buffer);
  auto result =
      Visitor::from(AttributeRegistry<NodeAttributes>::current(), deserializer);

  ASSERT_TRUE(result);
  EXPECT_EQ(*expected, *result)
      << "expected: " << printAttrs(*expected) << ", result: " << printAttrs(*result);
}

TEST_P(EdgeAttributeSerializationFixture, BinarySerialization) {
  const auto expected = GetParam();
  ASSERT_TRUE(expected);

  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(*expected);

  serialization::BinaryDeserializer deserializer(buffer);
  auto result =
      Visitor::from(AttributeRegistry<EdgeAttributes>::current(), deserializer);

  ASSERT_TRUE(result);
  EXPECT_EQ(*expected, *result)
      << "expected: " << printAttrs(*expected) << ", result: " << printAttrs(*result);
}

TEST(AttributeSerializationTests, AttributesWithNaNJson) {
  const auto factory = AttributeRegistry<NodeAttributes>::current();
  std::string raw_json =
      R"delim({
        "bounding_box": {
            "dimensions": [-16.550033569335938, 21.749967575073242, 1.7499535083770752],
            "type": "AABB",
            "world_P_center": [-16.95003890991211, 21.499961853027344, 1.3764979839324951],
            "world_R_center": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
        },
        "color": {"r": 39, "g": 144, "b": 65},
        "name": "O(0)",
        "position": [null, null, null],
        "registered": false,
        "semantic_label": 7,
        "type": "ObjectNodeAttributes",
        "world_R_object": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}
      })delim";

  const json attr_json = json::parse(raw_json);
  auto attrs = Visitor::from(factory, attr_json);
  ASSERT_TRUE(attrs);
  EXPECT_TRUE(attrs->position.hasNaN());

  std::string raw_json2 =
      R"delim({
        "position": [null, null, null],
        "type": "ObjectNodeAttributes"
      })delim";
  const json attr_json2 = json::parse(raw_json2);

  auto new_attrs = Visitor::from(factory, attr_json2);
  ASSERT_TRUE(new_attrs);
  EXPECT_TRUE(new_attrs->position.hasNaN());
}

}  // namespace spark_dsg
