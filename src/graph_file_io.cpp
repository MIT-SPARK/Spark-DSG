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
#include "spark_dsg/graph_file_io.h"

#include <filesystem>
#include <fstream>
#include <sstream>

#include "spark_dsg/binary_serializer.h"
#include "spark_dsg/dynamic_scene_graph.h"
#include "spark_dsg/graph_binary_serialization.h"
#include "spark_dsg_version.h"

namespace spark_dsg {

void DynamicSceneGraph::save(std::string filepath, bool include_mesh) const {
  const io::FileType type = io::verifyFileExtension(filepath);

  if (type == io::FileType::JSON) {
    std::ofstream outfile(filepath);
    outfile << this->serializeToJson(include_mesh);
    return;
  }

  // Can only be binary after verification.
  io::saveDsgBinary(*this, filepath, include_mesh);
}

DynamicSceneGraph::Ptr DynamicSceneGraph::load(std::string filepath) {
  const io::FileType type = io::verifyFileExtension(filepath);

  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("graph file does not exist: " + filepath);
  }

  if (type == io::FileType::JSON) {
    std::ifstream infile(filepath);
    std::stringstream ss;
    ss << infile.rdbuf();
    return deserializeFromJson(ss.str());
  }

  // Can only be binary after verification (worstcase: throws meaningful error)
  return io::loadDsgBinary(filepath);
}

namespace io {

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
        << io::BINARY_EXTENSION << "', '" << io::JSON_EXTENSION
        << "', and no extension (defaults to binary save mode).";
    throw std::runtime_error(msg.str());
  }

  return type;
}

void saveDsgBinary(const DynamicSceneGraph& graph,
                   const std::string& filepath,
                   bool include_mesh) {
  // Get the header data.
  const FileHeader header = FileHeader::current();
  const std::vector<uint8_t> header_buffer = header.serialize();

  // Get the DSG data.
  std::vector<uint8_t> graph_buffer;
  writeGraph(graph, graph_buffer, include_mesh);

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
  const std::optional<FileHeader> header = FileHeader::deserialize(buffer, &offset);
  if (!header) {
    throw std::runtime_error(
        "invalid file: attempted to load a binary file that is not a "
        "spark-dsg");
  }

  // Check for compatibility issues.
  checkCompatibility(*header);

  // Deserialize the graph.
  return readGraph(buffer.data() + offset, buffer.size() - offset, *header);
}

void checkCompatibility(const FileHeader& loaded, const FileHeader& current) {
  // Check the project name.
  if (loaded.project_name != current.project_name) {
    std::stringstream msg;
    msg << "Attempted to load invalid binary file: the loaded file was created with a "
           "different project name ("
        << loaded.project_name << ") than the current project name ("
        << current.project_name << ").";
    throw(std::runtime_error(msg.str()));
    // NOTE(lschmid): Modifications for external projects will always be breaking
    // changes that are unknown to, so we employ a hard check here. Alternatively, this
    // distinction could also be more fine graind for known projects.
  }

  // TODO(lschmid): Add version compatibility checks if needed and once once there.
}

FileHeader FileHeader::current() {
  FileHeader header;
  header.project_name = CURRENT_PROJECT_NAME;
  header.version.major = SPARK_DSG_VERSION_MAJOR;
  header.version.minor = SPARK_DSG_VERSION_MINOR;
  header.version.patch = SPARK_DSG_VERSION_PATCH;
  return header;
}

bool FileHeader::Version::operator==(const Version& other) const {
  return major == other.major && minor == other.minor && patch == other.patch;
}

bool FileHeader::Version::operator<(const Version& other) const {
  if (major < other.major) {
    return true;
  } else if (major == other.major) {
    if (minor < other.minor) {
      return true;
    } else if (minor == other.minor) {
      return patch < other.patch;
    }
  }
  return false;
}

std::string FileHeader::Version::toString() const {
  std::stringstream ss;
  ss << static_cast<int>(major) << "." << static_cast<int>(minor) << "."
     << static_cast<int>(patch);
  return ss.str();
}

std::vector<uint8_t> FileHeader::serialize() const {
  std::vector<uint8_t> buffer;
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(IDENTIFIER_STRING);
  serializer.write(project_name);
  serializer.write(version.major);
  serializer.write(version.minor);
  serializer.write(version.patch);
  return buffer;
}

std::optional<FileHeader> FileHeader::deserialize(const std::vector<uint8_t>& buffer,
                                                  size_t* offset) {
  // Check the buffer is valid.
  serialization::BinaryDeserializer deserializer(buffer);
  if (deserializer.getCurrType() != serialization::PackType::ARR32) {
    return std::nullopt;
  }
  std::string identifier;
  deserializer.read(identifier);
  if (identifier != IDENTIFIER_STRING) {
    return std::nullopt;
  }

  // Deserialize the header.
  FileHeader header;
  deserializer.read(header.project_name);
  deserializer.read(header.version.major);
  deserializer.read(header.version.minor);
  deserializer.read(header.version.patch);

  if (offset) {
    *offset = deserializer.pos;
  }
  return header;
}

}  // namespace io

}  // namespace spark_dsg