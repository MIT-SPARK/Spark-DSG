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

#include <unordered_map>

#include "spark_dsg/color.h"

namespace spark_dsg {

TEST(Color, Hash) {
  const Color c1 = Color::cyan();
  const Color c2 = Color::magenta();
  const Color c3 = Color::yellow();
  const Color c4(12, 23, 34, 45);

  std::unordered_map<Color, int, Color::Hash> map;
  map[c1] = 1;
  map[c2] = 2;
  map[c3] = 3;
  map[c4] = 4;
  EXPECT_EQ(map.size(), 4);
  EXPECT_EQ(map.at(c1), 1);
  EXPECT_EQ(map.at(c2), 2);
  EXPECT_EQ(map.at(c3), 3);
  EXPECT_EQ(map.at(c4), 4);
}

TEST(Color, Random) {
  const Color c1 = Color::random();
  const Color c2 = Color::random();
  const Color c3 = Color::random();
  const Color c4 = Color::random();

  EXPECT_NE(c1, c2);
  EXPECT_NE(c1, c3);
  EXPECT_NE(c1, c4);
  EXPECT_NE(c2, c3);
  EXPECT_NE(c2, c4);
  EXPECT_NE(c3, c4);
}

TEST(Color, Blend) {
  Color r = Color::red();
  Color g = Color::green();

  // Non-blend
  Color rg = r.blend(g, 0.0f);
  EXPECT_EQ(rg, r);
  rg = r.blend(g, 1.0f);
  EXPECT_EQ(rg, g);

  // Outside ratio.
  rg = r.blend(g, -0.5f);
  EXPECT_EQ(rg, r);
  rg = r.blend(g, 100.0f);
  EXPECT_EQ(rg, g);

  // Blend
  rg = r.blend(g, 0.5f);
  EXPECT_EQ(rg, Color(127, 127, 0));
  rg = r.blend(g, 0.25f);
  EXPECT_EQ(rg, Color(191, 63, 0));
}

TEST(Color, Rainbow) {
  EXPECT_EQ(Color::rainbow(0), Color::red());
  EXPECT_EQ(Color::rainbow(0.1), Color(255,153,0));
  EXPECT_EQ(Color::rainbow(0.2), Color(203,255,0));
  EXPECT_EQ(Color::rainbow(0.3), Color(50,255,0));
  EXPECT_EQ(Color::rainbow(0.4), Color(0,255,102));
  EXPECT_EQ(Color::rainbow(0.5), Color::cyan());
  EXPECT_EQ(Color::rainbow(0.6), Color(0,101,255));
  EXPECT_EQ(Color::rainbow(0.7), Color(50,0,255));
  EXPECT_EQ(Color::rainbow(0.8), Color(204,0,255));
  EXPECT_EQ(Color::rainbow(0.9), Color(255,0,153));
  EXPECT_EQ(Color::rainbow(1), Color::red());
}

TEST(Color, RainbowID) {
  EXPECT_EQ(Color::rainbowId(0,16), Color::red());
  EXPECT_EQ(Color::rainbowId(1,16), Color(255,95,0));
  EXPECT_EQ(Color::rainbowId(8,16), Color::cyan());
  EXPECT_EQ(Color::rainbowId(15,16), Color(255,0,95));
  EXPECT_EQ(Color::rainbowId(16,16), Color(255, 47, 0));
  EXPECT_EQ(Color::rainbowId(17,16), Color(255, 143, 0));
  EXPECT_EQ(Color::rainbowId(32,16), Color(255, 23, 0));
  EXPECT_EQ(Color::rainbowId(33,16), Color(255, 119, 0));
}

TEST(Color, fromHSV) {
  EXPECT_EQ(Color::fromHSV(0.0f, 0.0f, 0.0f), Color::black());
  EXPECT_EQ(Color::fromHSV(0.0f, 0.0f, 1.0f), Color::white());
  EXPECT_EQ(Color::fromHSV(0.0f, 1.0f, 1.0f), Color::red());
  EXPECT_EQ(Color::fromHSV(0.3333f, 1.0f, 1.0f), Color::green());
  EXPECT_EQ(Color::fromHSV(0.6666f, 1.0f, 1.0f), Color::blue());
  EXPECT_EQ(Color::fromHSV(0.25f, 0.8f, 0.3f), Color(45, 76, 15));
  EXPECT_EQ(Color::fromHSV(0.6f, 0.45f, 0.65f), Color(91, 120, 165));
}

TEST(Color, fromHLS) {
  EXPECT_EQ(Color::fromHLS(0.0f, 0.0f, 0.0f), Color::black());
  EXPECT_EQ(Color::fromHLS(0.0f, 1.0f, 0.0f), Color::white());
  EXPECT_EQ(Color::fromHLS(0.0f, 0.5f, 1.0f), Color::red());
  EXPECT_EQ(Color::fromHLS(0.3333f, 0.5f, 1.0f), Color::green());
  EXPECT_EQ(Color::fromHLS(0.6666f, 0.5f, 1.0f), Color::blue());
  EXPECT_EQ(Color::fromHLS(0.25f, 0.3f, 0.8f), Color(76, 137, 15));
  EXPECT_EQ(Color::fromHLS(0.6f, 0.65f, 0.45f), Color(125, 157, 205));
}

}  // namespace spark_dsg
