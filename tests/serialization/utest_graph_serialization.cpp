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

#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/serialization/graph_json_serialization.h"
#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg {

void PrintTo(const DynamicSceneGraph& graph, std::ostream* os) {
  *os << io::json::writeGraph(graph, true);
}

struct SerializationMethod {
  using Ptr = std::shared_ptr<SerializationMethod>;
  virtual ~SerializationMethod() = default;
  virtual DynamicSceneGraph::Ptr compute(const DynamicSceneGraph& graph,
                                         bool include_mesh = true) const = 0;
  virtual std::string name() const = 0;
};

struct JsonRoundTrip : SerializationMethod {
  virtual ~JsonRoundTrip() = default;
  DynamicSceneGraph::Ptr compute(const DynamicSceneGraph& graph,
                                 bool include_mesh = true) const override {
    const auto output = io::json::writeGraph(graph, include_mesh);
    return io::json::readGraph(output);
  }

  std::string name() const override { return "Json"; }
};

struct BinaryRoundTrip : SerializationMethod {
  virtual ~BinaryRoundTrip() = default;
  DynamicSceneGraph::Ptr compute(const DynamicSceneGraph& graph,
                                 bool include_mesh = true) const override {
    std::vector<uint8_t> buffer;
    io::binary::writeGraph(graph, buffer, include_mesh);
    return io::binary::readGraph(buffer);
  }

  std::string name() const override { return "Binary"; }
};

struct GraphSerializationFixture
    : public testing::TestWithParam<SerializationMethod::Ptr> {
  GraphSerializationFixture() {}
  virtual ~GraphSerializationFixture() = default;
};

const SerializationMethod::Ptr serialization_test_cases[] = {
    std::make_shared<JsonRoundTrip>(),
    std::make_shared<BinaryRoundTrip>(),
};

INSTANTIATE_TEST_SUITE_P(
    GraphSerialization,
    GraphSerializationFixture,
    testing::ValuesIn(serialization_test_cases),
    [](const testing::TestParamInfo<GraphSerializationFixture::ParamType>& info) {
      return info.param ? info.param->name() : "Invalid";
    });

TEST_P(GraphSerializationFixture, DsgBasic) {
  const auto round_trip_serializer = GetParam();

  DynamicSceneGraph expected({1, 2, 3});
  expected.emplaceNode(1, 0, std::make_unique<NodeAttributes>());
  expected.emplaceNode(1, 1, std::make_unique<NodeAttributes>());
  expected.emplaceNode(3, 2, std::make_unique<NodeAttributes>());

  expected.insertEdge(0, 1);
  expected.insertEdge(1, 2);

  const auto result = round_trip_serializer->compute(expected);
  ASSERT_TRUE(result);
  EXPECT_EQ(expected, *result);
}

TEST_P(GraphSerializationFixture, DsgWithNaNs) {
  const auto round_trip_serializer = GetParam();

  DynamicSceneGraph expected({1, 2, 3});
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

  const auto result = round_trip_serializer->compute(expected);
  ASSERT_TRUE(result);
  EXPECT_EQ(expected, *result);
}

TEST_P(GraphSerializationFixture, DsgDynamic) {
  const auto round_trip_serializer = GetParam();

  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  const auto result = round_trip_serializer->compute(expected);
  ASSERT_TRUE(result);
  EXPECT_EQ(expected, *result);
}

TEST_P(GraphSerializationFixture, DsgWithMesh) {
  const auto round_trip_serializer = GetParam();

  using namespace std::chrono_literals;
  DynamicSceneGraph expected;
  expected.emplaceNode(3, 0, std::make_unique<NodeAttributes>());

  expected.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  expected.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>(), false);
  expected.emplaceNode(2, 'a', 40ns, std::make_unique<NodeAttributes>());

  auto mesh = std::make_shared<Mesh>();
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->points.push_back(Eigen::Vector3f::Zero());
  mesh->colors.push_back({10, 20, 30, 255});
  mesh->labels.push_back(2);
  mesh->labels.push_back(8);
  mesh->stamps.push_back(0);
  mesh->stamps.push_back(10);
  mesh->stamps.push_back(20);
  mesh->stamps.push_back(30);
  mesh->faces.push_back({{1, 2, 3}});
  expected.setMesh(mesh);

  const auto result = round_trip_serializer->compute(expected, true);
  ASSERT_TRUE(result);
  EXPECT_EQ(expected, *result);
}

TEST(GraphSerialization, UpdateDsgFromBinaryWithCorrection) {
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
  updated.emplaceNode(3, 0, std::make_unique<NodeAttributes>());
  updated.emplaceNode(3, 1, std::make_unique<NodeAttributes>());
  updated.emplaceNode(3, 2, std::make_unique<NodeAttributes>());
  updated.emplaceNode(4, 3, std::make_unique<NodeAttributes>());
  updated.emplaceNode(4, 4, std::make_unique<NodeAttributes>());
  updated.insertEdge(0, 1);
  updated.insertEdge(0, 3);

  updated.emplaceNode(2, 'a', 10ns, std::make_unique<NodeAttributes>());
  updated.emplaceNode(2, 'a', 20ns, std::make_unique<NodeAttributes>());
  updated.emplaceNode(2, 'a', 30ns, std::make_unique<NodeAttributes>());
  updated.emplaceNode(3, 'b', 40ns, std::make_unique<NodeAttributes>());
  updated.insertEdge(0, "a0"_id);

  std::vector<uint8_t> buffer;
  io::binary::writeGraph(original, buffer);
  io::binary::updateGraph(updated, buffer);

  EXPECT_EQ(original, updated);
}

}  // namespace spark_dsg
