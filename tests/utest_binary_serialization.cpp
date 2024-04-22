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

TEST(BinarySerializationTests, SerializeEigenMatrix) {
  {  // double fixed-size matrix
    Eigen::Matrix2d expected;
    expected << 1.0, 2.0, 3.0, 4.0;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // mixed matrix size
    Eigen::Matrix<float, 3, Eigen::Dynamic> expected(3, 4);
    expected << 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f,
        12.0f;

    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // dynamic matrix
    Eigen::MatrixXd expected(2, 3);
    expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

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

TEST(BinarySerializationTests, SerializeMap) {
  {  // simple type
    std::map<uint32_t, uint32_t> expected;
    expected[1] = 2;
    expected[3] = 4;
    expected[5] = 6;
    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
  }

  {  // more complex type
    std::map<std::string, std::vector<size_t>> expected;
    expected["a"] = {1, 2, 3};
    expected["b"] = {4, 5, 6};
    expected["c"] = {7, 8, 9};
    auto result = writeRT(expected);
    EXPECT_EQ(expected, result);
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

}  // namespace spark_dsg
