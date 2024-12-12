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
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph_types.h>

namespace spark_dsg {

void PrintTo(const LayerKey& edge, std::ostream* os) { *os << edge; }

TEST(LayerKeyTests, TestEquality) {
  EXPECT_EQ(LayerKey(1), LayerKey(1));
  EXPECT_NE(LayerKey(1), LayerKey(2));
  EXPECT_NE(LayerKey(1), LayerKey(1, 0u));
  EXPECT_EQ(LayerKey(2, 0u), LayerKey(2, 0u));
  EXPECT_NE(LayerKey(2, 0u), LayerKey(2, 1u));
}

TEST(LayerKeyTests, TestIsParent) {
  LayerKey key1{1};
  LayerKey key2{1};
  LayerKey key3{2};
  LayerKey key4{2, 0u};
  LayerKey key5{3, 0u};
  LayerKey key6{2, 1u};

  // static
  EXPECT_TRUE(key3.isParentOf(key1));
  EXPECT_FALSE(key1.isParentOf(key2));
  EXPECT_FALSE(key1.isParentOf(key3));

  // intralayer groups
  EXPECT_TRUE(key4.isParentOf(key1));
  EXPECT_TRUE(key5.isParentOf(key6));
  EXPECT_FALSE(key6.isParentOf(key4));
}

}  // namespace spark_dsg
