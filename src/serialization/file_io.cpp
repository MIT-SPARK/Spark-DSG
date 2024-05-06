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
#include "spark_dsg/serialization/file_io.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/logging.h"
#include "spark_dsg/serialization/graph_binary_serialization.h"
#include "spark_dsg/serialization/graph_json_serialization.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg::io {

FileType identifyFileType(const std::string& filepath) {
  const auto ext = std::filesystem::path(filepath).extension().string();
  if (ext.empty()) {
    return FileType::NONE;
  } else if (ext == JSON_EXTENSION || ext == BSON_EXTENSION) {
    return FileType::JSON;
  } else if (ext == BINARY_EXTENSION) {
    return FileType::BINARY;
  }
  return FileType::UNKNOWN;
}

FileType verifyFileExtension(std::string& filepath) {
  io::FileType type = io::identifyFileType(filepath);

  // If no file extension is provided, default to binary.
  if (type == io::FileType::NONE) {
    type = io::FileType::BINARY;
    filepath += io::BINARY_EXTENSION;
  }

  // Check the file extension is valid.
  if (type == io::FileType::UNKNOWN) {
    std::stringstream msg;
    msg << "Invalid file extension for '" << filepath << "'. Supported are '"
        << io::BINARY_EXTENSION << "', '" << io::JSON_EXTENSION << "', '"
        << io::BSON_EXTENSION << "', and no extension (defaults to binary save mode).";
    throw std::runtime_error(msg.str());
  }

  return type;
}

void saveDsgBinary(const DynamicSceneGraph& graph,
                   const std::string& filepath,
                   bool include_mesh) {
  // Get the header data.
  const FileHeader header = FileHeader::current();
  const std::vector<uint8_t> header_buffer = header.serializeToBinary();

  // Get the DSG data.
  std::vector<uint8_t> graph_buffer;
  binary::writeGraph(graph, graph_buffer, include_mesh);

  // Write the header and graph data to the file.
  std::ofstream out(filepath, std::ios::out | std::ios::binary);
  out.write(reinterpret_cast<const char*>(header_buffer.data()), header_buffer.size());
  out.write(reinterpret_cast<const char*>(graph_buffer.data()), graph_buffer.size());
}

DynamicSceneGraph::Ptr loadDsgBinary(const std::string& filepath) {
  // Read the file into a buffer.
  std::ifstream infile(filepath, std::ios::in | std::ios::binary);
  std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(infile)),
                              std::istreambuf_iterator<char>());

  // Deserialize the header.
  size_t offset;
  const FileHeader header =
      FileHeader::deserializeFromBinary(buffer, &offset).value_or(FileHeader::legacy());

  // Check for compatibility issues.
  checkCompatibility(header);

  // Deserialize the graph.
  GlobalInfo::ScopedInfo info(header);
  return binary::readGraph(buffer.data() + offset, buffer.size() - offset);
}

void saveDsgJson(const DynamicSceneGraph& graph,
                 const std::string& filepath,
                 bool include_mesh) {
  std::ofstream outfile(filepath);
  outfile << json::writeGraph(graph, include_mesh);
}

DynamicSceneGraph::Ptr loadDsgJson(const std::string& filepath) {
  std::ifstream infile(filepath);
  std::stringstream ss;
  ss << infile.rdbuf();
  return json::readGraph(ss.str());
}

}  // namespace spark_dsg::io
