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

#include "spark_dsg/serialization/binary_serialization.h"

namespace spark_dsg::serialization {

TEST(BinarySerialization, SwapCorrect16) {
  uint16_t original = 0x1234;

  uint16_t swapped;
  detail::SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0x3412);

  uint16_t rt_value;
  detail::SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerialization, SwapCorrect32) {
  uint32_t original = 0x12345678;

  uint32_t swapped;
  detail::SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0x78563412);

  uint32_t rt_value;
  detail::SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerialization, SwapCorrect64) {
  uint64_t original = 0x123456789abcdef0;

  uint64_t swapped = 0;
  detail::SwapEndian::swap(original, swapped);

  ASSERT_EQ(swapped, 0xf0debc9a78563412);

  uint64_t rt_value = 0;
  detail::SwapEndian::swap(swapped, rt_value);
  ASSERT_EQ(original, rt_value);
}

TEST(BinarySerialization, TestWriteWord) {
  std::vector<uint8_t> buffer;
  uint32_t word = 0x12345678;
  detail::writeWord(buffer, word);
  ASSERT_EQ(buffer.size(), 4u);
  if (!detail::NeedEndianSwap()) {
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
  detail::writeWord(buffer, signed_word);
  ASSERT_EQ(buffer.size(), 8u);
  if (!detail::NeedEndianSwap()) {
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

}  // namespace spark_dsg::serialization
