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

#include <stdexcept>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/scene_graph_node.h"
#include "spark_dsg/serialization/attribute_registry.h"

namespace spark_dsg::serialization {
namespace {

struct CustomNodeAttributes : NodeAttributes {
  CustomNodeAttributes() { position.x() = 42.0; }

 protected:
  const RegistrationInfo& registrationImpl() const override {
    static const auto info =
        AttributeRegistry<NodeAttributes>::registration("CustomNodeAttributes");
    return info;
  }
};

}  // namespace

TEST(AttributeRegistry, BuiltinsAvailableWithoutConsumerRegistration) {
  const std::vector<std::string> expected{"NodeAttributes",
                                          "SemanticNodeAttributes",
                                          "ObjectNodeAttributes",
                                          "RoomNodeAttributes",
                                          "PlaceNodeAttributes",
                                          "Place2dNodeAttributes",
                                          "AgentNodeAttributes",
                                          "KhronosObjectAttributes",
                                          "TraversabilityNodeAttributes",
                                          "TravNodeAttributes"};
  const auto factory = AttributeRegistry<NodeAttributes>::current();
  for (const auto& name : expected) {
    const auto attrs = factory.create(name);
    ASSERT_NE(attrs, nullptr) << name;
    EXPECT_EQ(attrs->registration().name, name);
  }

  const auto edge =
      AttributeRegistry<EdgeAttributes>::current().create("EdgeAttributes");
  ASSERT_NE(edge, nullptr);
  EXPECT_EQ(edge->registration().name, "EdgeAttributes");
}

TEST(AttributeRegistry, FileNamesDetermineTypeIds) {
  const auto factory = AttributeRegistry<NodeAttributes>::fromNames(
      {"ObjectNodeAttributes", "UnknownAttributes", "NodeAttributes"});
  const auto object = factory.create(uint8_t{0});
  ASSERT_NE(object, nullptr);
  EXPECT_EQ(object->registration().name, "ObjectNodeAttributes");
  EXPECT_EQ(factory.create(uint8_t{1}), nullptr);
  const auto node = factory.create(uint8_t{2});
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->registration().name, "NodeAttributes");
  EXPECT_EQ(factory.create("UnknownAttributes"), nullptr);
}

TEST(AttributeRegistry, RejectsUnrepresentableTypeIds) {
  const std::vector<std::string> names(257, "NodeAttributes");
  EXPECT_THROW(AttributeRegistry<NodeAttributes>::fromNames(names), std::length_error);
}

TEST(AttributeRegistry, CustomTypesShareLibraryRegistry) {
  const AttributeRegistration<NodeAttributes, CustomNodeAttributes> registration(
      "CustomNodeAttributes");
  const auto factory = AttributeRegistry<NodeAttributes>::current();
  const auto attrs = factory.create(registration.info.type_id);
  ASSERT_NE(attrs, nullptr);
  EXPECT_NE(dynamic_cast<CustomNodeAttributes*>(attrs.get()), nullptr);
  EXPECT_EQ(attrs->position.x(), 42.0);
  EXPECT_EQ(attrs->registration().name, "CustomNodeAttributes");
  EXPECT_THROW(AttributeRegistry<NodeAttributes>::addAttributes<CustomNodeAttributes>(
                   "CustomNodeAttributes"),
               std::runtime_error);
}

}  // namespace spark_dsg::serialization
