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

#include <nlohmann/json.hpp>

#include "spark_dsg/metadata.h"

namespace spark_dsg {

TEST(Metadata, SetCorrect) {
  const auto node_1 = R"({"foo": 5, "bar": {"hello": 5, "world": 10}})"_json;
  Metadata data;
  data.set(node_1);

  EXPECT_EQ(node_1.dump(), data.get().dump());
}

TEST(Metadata, AddCorrect) {
  const auto node_1 = R"({"foo": 5, "bar": {"hello": 5, "world": 10}})"_json;
  const auto node_2 =
      R"({"foo": 6, "bar": {"world": "!", "other": 42}, "temp": -1.0})"_json;

  {  // 1, then 2
    Metadata data;
    data.add(node_1);
    data.add(node_2);
    const auto expected =
        R"({
  "foo": 6,
  "bar": {
    "hello": 5,
    "world": "!",
    "other": 42
  },
  "temp": -1.0
})"_json;
    EXPECT_EQ(expected.dump(), data.get().dump());
  }

  {  // 2, then 1
    Metadata data;
    data.add(node_2);
    data.add(node_1);
    const auto expected =
        R"({
  "foo": 5,
  "bar": {
    "hello": 5,
     "world": 10,
     "other": 42
  },
  "temp": -1.0
})"_json;
    EXPECT_EQ(expected.dump(), data.get().dump());
  }
}

TEST(Metadata, CopiesOwnTheirContents) {
  const auto original = R"({"nested": {"value": 4}})"_json;
  Metadata source(original);
  Metadata copied(source);
  Metadata assigned;
  assigned = source;
  source.add(R"({"nested": {"value": 9}})"_json);
  EXPECT_EQ(copied.get(), original);
  EXPECT_EQ(assigned.get(), original);

  copied.clear();
  EXPECT_TRUE(copied.empty());
  EXPECT_EQ(assigned.get(), original);
  EXPECT_EQ(source.get().at("nested").at("value"), 9);
}

TEST(Metadata, MovesPreserveContentsAndAllowReuse) {
  const auto contents = R"({"value": [1, 2, 3]})"_json;
  Metadata source(contents);
  Metadata moved(std::move(source));
  EXPECT_EQ(moved.get(), contents);
  EXPECT_TRUE(source.empty());
  source.add(R"({"new": 5})"_json);
  EXPECT_EQ(source.get(), R"({"new": 5})"_json);

  source = std::move(moved);
  EXPECT_EQ(source.get(), contents);
  EXPECT_TRUE(moved.empty());
  moved.set(R"({"reused": true})"_json);
  EXPECT_EQ(moved.get(), R"({"reused": true})"_json);
}

TEST(Metadata, EmptyAndNonObjectValues) {
  Metadata data;
  EXPECT_EQ(data.get(), nlohmann::json::object());
  data.add(Metadata(R"({"nested": {"value": 4}})"_json));
  EXPECT_EQ(data.get(), R"({"nested": {"value": 4}})"_json);

  data.add(R"([1, 2])"_json);
  EXPECT_EQ(data.get(), R"([1, 2])"_json);
  data.clear();
  EXPECT_EQ(data.get(), nlohmann::json::array());
  data.set(nullptr);
  EXPECT_TRUE(data.get().is_null());
  data = Metadata();
  EXPECT_EQ(data.get(), nlohmann::json::object());
}

}  // namespace spark_dsg
