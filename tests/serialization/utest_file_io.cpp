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

#include "spark_dsg/serialization/file_io.h"
#include "spark_dsg/serialization/versioning.h"
#include "spark_dsg_tests/temp_file.h"
#include "spark_dsg_tests/type_comparisons.h"

namespace spark_dsg::io {

TEST(FileIoTests, FileIdentifierString) {
  // Make sure that if someone changes the identifier string, at least the utests fail.
  EXPECT_EQ(FileHeader::IDENTIFIER_STRING, "SPARK_DSG");
}

TEST(FileIoTests, FileTypeIdentification) {
  EXPECT_EQ(identifyFileType("some/path/test.json"), FileType::JSON);
  EXPECT_EQ(identifyFileType("some/path/test.sparkdsg"), FileType::BINARY);
  EXPECT_EQ(identifyFileType("some/path/test"), FileType::NONE);
  EXPECT_EQ(identifyFileType("some/path/test.txt"), FileType::UNKNOWN);
}

TEST(FileIoTests, VersionSerialization) {
  FileHeader header;
  header.project_name = "test";
  header.version.major = 1;
  header.version.minor = 2;
  header.version.patch = 3;
  const auto buffer = header.serializeToBinary();

  // Check deserialization.
  const auto result = FileHeader::deserializeFromBinary(buffer);
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(header.project_name, result->project_name);
  EXPECT_EQ(header.version, result->version);

  // Check deserialization with invalid buffer.
  const std::vector<uint8_t> short_buffer = {0, 1, 2, 3};
  EXPECT_FALSE(FileHeader::deserializeFromBinary(short_buffer).has_value());

  std::vector<uint8_t> random_buffer = buffer;
  random_buffer[7] = 7;
  EXPECT_FALSE(FileHeader::deserializeFromBinary(random_buffer).has_value());
}

void testSaveLoad(const std::string& file_name) {
  DynamicSceneGraph graph;
  graph.emplaceNode(DsgLayers::PLACES,
                    NodeSymbol('p', 0),
                    std::make_unique<NodeAttributes>(Eigen::Vector3d::Zero()));
  graph.setMesh(std::make_shared<Mesh>());
  graph.save(file_name);
  auto other = DynamicSceneGraph::load(file_name);

  EXPECT_EQ(graph.numNodes(), other->numNodes());
  EXPECT_EQ(graph.numLayers(), other->numLayers());
  EXPECT_EQ(graph.hasMesh(), other->hasMesh());
}

TEST(FileIoTests, SaveLoadJson) {
  TempFile tmp_file;
  const std::string file_name = tmp_file.path + ".json";
  testSaveLoad(file_name);
}

TEST(FileIoTests, SaveLoadBinary) {
  TempFile tmp_file;
  const std::string file_name = tmp_file.path + ".sparkdsg";
  testSaveLoad(file_name);
}

}  // namespace spark_dsg::io
