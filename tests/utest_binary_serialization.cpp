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
#include <spark_dsg/binary_serializer.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <stdlib.h>
#include <unistd.h>

#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {
namespace serialization {

TEST(BinarySerializationTests, SwapCorrect16) {
  uint16_t original = 0x1234;

  uint16_t swapped;
  SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0x3412);

  uint16_t rt_value;
  SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerializationTests, SwapCorrect32) {
  uint32_t original = 0x12345678;

  uint32_t swapped;
  SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0x78563412);

  uint32_t rt_value;
  SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerializationTests, SwapCorrect64) {
  uint64_t original = 0x123456789abcdef0;

  uint64_t swapped = 0;
  SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0xf0debc9a78563412);

  uint64_t rt_value = 0;
  SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerializationTests, TestWriteWord) {
  std::vector<uint8_t> buffer;
  uint32_t word = 0x12345678;
  writeWord(buffer, word);
  ASSERT_EQ(buffer.size(), 4u);
  if (!NeedEndianSwap()) {
    EXPECT_EQ(buffer[0], 0x78);
    EXPECT_EQ(buffer[1], 0x56);
    EXPECT_EQ(buffer[2], 0x34);
    EXPECT_EQ(buffer[3], 0x12);
  } else {
    EXPECT_EQ(buffer[3], 0x78);
    EXPECT_EQ(buffer[2], 0x56);
    EXPECT_EQ(buffer[1], 0x34);
    EXPECT_EQ(buffer[0], 0x12);
  }

  int32_t signed_word = 0x12345678;
  writeWord(buffer, signed_word);
  ASSERT_EQ(buffer.size(), 8u);
  if (!NeedEndianSwap()) {
    EXPECT_EQ(buffer[4], 0x78);
    EXPECT_EQ(buffer[5], 0x56);
    EXPECT_EQ(buffer[6], 0x34);
    EXPECT_EQ(buffer[7], 0x12);
  } else {
    EXPECT_EQ(buffer[7], 0x78);
    EXPECT_EQ(buffer[6], 0x56);
    EXPECT_EQ(buffer[5], 0x34);
    EXPECT_EQ(buffer[4], 0x12);
  }
}

}  // namespace serialization

template <typename T>
T writeRT(const T& expected) {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(expected);

  serialization::BinaryDeserializer deserializer(buffer);

  T result;
  deserializer.read(result);
  return result;
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

TEST(BinarySerializationTests, SerializeEigenVector) {
  {  // double vector
    Eigen::Vector3d expected;
    expected << 1.0, 2.0, 3.0;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // uint8_t vector
    SemanticNodeAttributes::ColorVector expected;
    expected << 1, 2, 3;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // dynamic float vector
    Eigen::VectorXf expected(5, 1);
    expected << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // dynamic int vector
    Eigen::VectorXi expected(5, 1);
    expected << 1, 2, 3, 4, 5;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }
}

TEST(BinarySerializationTests, SerializeEigenQuaternion) {
  std::stringstream ss;

  {  // single-precision
    Eigen::Quaternionf expected(0.0, 0.0, 1.0, 0.0);
    auto result = writeRT(expected);
    ASSERT_TRUE(quaternionsEqual(expected, result));
  }

  {  // double-precision
    Eigen::Quaterniond expected(0.0, 0.0, 0.0, 1.0);
    auto result = writeRT(expected);
    ASSERT_TRUE(quaternionsEqual(expected, result));
  }
}

TEST(BinarySerializationTests, SerializeBoundingBox) {
  {  // invalid type
    BoundingBox expected;
    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // ABB
    Eigen::Vector3f expected_min;
    expected_min << 1.0f, 2.0f, 3.0f;
    Eigen::Vector3f expected_max;
    expected_max << 4.0f, 5.0f, 6.0f;

    BoundingBox expected(expected_min, expected_max);
    auto result = writeRT(expected);
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
    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }
}

TEST(BinarySerializationTests, SerializeNodeAttributes) {
  {  // base class
    NodeAttributes expected;
    expected.position << 1.0, 2.0, 3.0;

    auto result = writeAttrsRT(expected);
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

    auto result = writeAttrsRT(expected);
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

    auto result = writeAttrsRT(expected);
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

    auto result = writeAttrsRT(expected);
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

    auto result = writeAttrsRT(expected);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }
}

TEST(BinarySerializationTests, SerializeEdgeInfo) {
  {  // base class
    EdgeAttributes expected;
    expected.weighted = true;
    expected.weight = 5.0;

    auto result = writeAttrsRT(expected);
    ASSERT_TRUE(result != nullptr);
    EXPECT_EQ(expected, *result);
  }
}

TEST(BinarySerializationTests, SerializeDsgBasic) {
  DynamicSceneGraph expected({1, 2, 3}, 0);
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  std::vector<uint8_t> buffer;
  writeGraph(expected, buffer);
  auto result = readGraph(buffer);

  EXPECT_EQ(expected.numNodes(), result->numNodes());
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(1));
  EXPECT_TRUE(result->hasNode(2));
  EXPECT_TRUE(result->hasEdge(0, 1));
  EXPECT_TRUE(result->hasEdge(1, 2));
  EXPECT_EQ(expected.hasLayer(0), result->hasLayer(0));
}

TEST(BinarySerializationTests, SerializeDsgWithNaNs) {
  DynamicSceneGraph expected({1, 2, 3}, 0);
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

  std::vector<uint8_t> buffer;
  writeGraph(expected, buffer);
  auto result = readGraph(buffer);

  EXPECT_EQ(expected.numNodes(), result->numNodes());
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(1));
  EXPECT_TRUE(result->hasNode(2));
  EXPECT_TRUE(result->hasNode(3));
  EXPECT_TRUE(result->hasEdge(0, 1));
  EXPECT_TRUE(result->hasEdge(1, 2));
  EXPECT_TRUE(result->hasEdge(2, 3));
  EXPECT_EQ(expected.hasLayer(0), result->hasLayer(0));
}

TEST(BinarySerializationTests, SerializeDsgDynamic) {
  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  std::vector<uint8_t> buffer;
  writeGraph(expected, buffer);
  auto result = readGraph(buffer);

  EXPECT_EQ(expected.numNodes(), result->numNodes());
  EXPECT_EQ(expected.numEdges(), result->numEdges());
  EXPECT_EQ(expected.numLayers(), result->numLayers());
  EXPECT_EQ(expected.layer_ids, result->layer_ids);

  EXPECT_TRUE(result->hasNode(0));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 0)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 1)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasNode(NodeSymbol('a', 3)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 0), NodeSymbol('a', 1)));
  EXPECT_FALSE(result->hasEdge(NodeSymbol('a', 1), NodeSymbol('a', 2)));
  EXPECT_TRUE(result->hasEdge(NodeSymbol('a', 2), NodeSymbol('a', 3)));

  EXPECT_TRUE(result->hasLayer(2, 'a'));
}

TEST(BinarySerializationTests, UpdateDsgFromBinaryWithCorrection) {
  using namespace std::chrono_literals;
  DynamicSceneGraph original;
  original.emplaceNode(3, 0, std::make_unique<NodeAttributes>());
  original.emplaceNode(3, 1, std::make_unique<NodeAttributes>());
  original.emplaceNode(4, 3, std::make_unique<NodeAttributes>());

  original.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  original.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  original.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  original.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  DynamicSceneGraph updated;
  EXPECT_TRUE(updated.emplaceNode(3, 0, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(3, 1, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(3, 2, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(4, 3, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(4, 4, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.insertEdge(0, 1));
  EXPECT_TRUE(updated.insertEdge(0, 3));

  EXPECT_TRUE(updated.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.emplaceNode(3, 'b', 40ns, std::make_unique<NodeAttributes>()));
  EXPECT_TRUE(updated.insertEdge(0, "a0"_id));

  std::vector<uint8_t> buffer;
  writeGraph(original, buffer);
  updateGraph(updated, buffer, true);

  EXPECT_EQ(original.numDynamicNodes(), updated.numDynamicNodes());
  EXPECT_EQ(original.numNodes(), updated.numNodes());
  EXPECT_EQ(original.numEdges(), updated.numEdges());
  EXPECT_EQ(original.numLayers(), updated.numLayers());
  EXPECT_EQ(original.layer_ids, updated.layer_ids);

  EXPECT_FALSE(updated.hasNode(2));
  EXPECT_FALSE(updated.hasNode(4));
  EXPECT_FALSE(updated.hasNode("b0"_id));
  EXPECT_FALSE(updated.hasEdge(0, 1));
  EXPECT_FALSE(updated.hasEdge(0, 3));
  EXPECT_FALSE(updated.hasEdge(0, "a0"_id));
  EXPECT_FALSE(updated.hasEdge("a1"_id, "a2"_id));
  EXPECT_FALSE(updated.getNode(0)->get().hasParent());
}

}  // namespace spark_dsg
