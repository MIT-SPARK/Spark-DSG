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
#include <filesystem>
#include <fstream>

#include "spark_dsg/binary_serializer.h"
#include "spark_dsg/graph_file_io.h"
#include "spark_dsg/mesh.h"

namespace spark_dsg {

void Mesh::serializeToBinary(std::vector<uint8_t>& buffer) const {
  serialization::BinarySerializer serializer(&buffer);

  // Write stats
  serializer.write(has_colors);
  serializer.write(has_timestamps);
  serializer.write(has_labels);

  // write vertices
  serializer.startFixedArray(6 * points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& pos = points[i];
    serializer.write<float>(pos.x());
    serializer.write<float>(pos.y());
    serializer.write<float>(pos.z());
    if (i < colors.size()) {
      const auto& color = colors[i];
      serializer.write<float>(255.0f * color.r);
      serializer.write<float>(255.0f * color.g);
      serializer.write<float>(255.0f * color.b);
    } else {
      serializer.write<float>(0.0f);
      serializer.write<float>(0.0f);
      serializer.write<float>(0.0f);
    }
  }

  // write faces
  serializer.startFixedArray(3 * faces.size());
  for (const auto& face : faces) {
    serializer.write(face[0]);
    serializer.write(face[1]);
    serializer.write(face[2]);
  }

  // write vertex attributes
  serializer.write(stamps);
  serializer.write(labels);
  serializer.write(last_seen_stamps);
}

void Mesh::save(std::string filepath) const {
  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    std::ofstream outfile(filepath);
    outfile << serializeToJson();
    return;
  }

  const auto header_buffer = io::FileHeader::current().serialize();
  std::vector<uint8_t> mesh_buffer;
  serializeToBinary(mesh_buffer);

  // Write the header and graph data to the file.
  std::ofstream out(filepath, std::ios::out | std::ios::binary);
  out.write(reinterpret_cast<const char*>(header_buffer.data()), header_buffer.size());
  out.write(reinterpret_cast<const char*>(mesh_buffer.data()), mesh_buffer.size());
}

Mesh::Ptr Mesh::deserializeFromBinary(const uint8_t* const buffer, size_t length) {
  serialization::BinaryDeserializer deserializer(buffer, length);

  bool has_colors, has_timestamps, has_labels;
  deserializer.read(has_colors);
  deserializer.read(has_timestamps);
  deserializer.read(has_labels);

  auto mesh = std::make_shared<Mesh>();
  size_t num_vertices = deserializer.readFixedArrayLength() / 6;
  for (size_t i = 0; i < num_vertices; ++i) {
    Mesh::Pos pos;
    deserializer.read(pos.x());
    deserializer.read(pos.y());
    deserializer.read(pos.z());
    mesh->points.push_back(pos);

    Eigen::Vector3f color;
    deserializer.read(color.x());
    deserializer.read(color.y());
    deserializer.read(color.z());
    color *= 255.0f;
    mesh->colors.push_back({static_cast<uint8_t>(color.x()),
                            static_cast<uint8_t>(color.y()),
                            static_cast<uint8_t>(color.z()),
                            255});
  }

  size_t num_faces = deserializer.readFixedArrayLength() / 3;
  mesh->faces.resize(num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    auto& face = mesh->face(i);
    deserializer.read(face[0]);
    deserializer.read(face[1]);
    deserializer.read(face[2]);
  }

  if (deserializer.checkIfTrue()) {
    size_t num_stamps = deserializer.readFixedArrayLength();
    mesh->stamps.resize(num_stamps);
    for (size_t i = 0; i < num_stamps; ++i) {
      deserializer.read(mesh->stamps.at(i));
    }
  }

  if (deserializer.checkIfTrue()) {
    size_t num_labels = deserializer.readFixedArrayLength();
    mesh->labels.resize(num_labels);
    for (size_t i = 0; i < num_labels; ++i) {
      deserializer.read(mesh->labels.at(i));
    }
  }

  return mesh;
}

Mesh::Ptr Mesh::load(std::string filepath) {
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("mesh file does not exist: " + filepath);
  }

  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    std::ifstream infile(filepath);
    std::stringstream ss;
    ss << infile.rdbuf();
    return deserializeFromJson(ss.str());
  }

  // Read the file into a buffer.
  std::ifstream infile(filepath, std::ios::in | std::ios::binary);
  std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(infile)),
                              std::istreambuf_iterator<char>());

  size_t offset;
  const auto header = io::FileHeader::deserialize(buffer, &offset);
  if (!header) {
    throw std::runtime_error("invalid file: file has bad encoding");
  }

  checkCompatibility(*header);
  return deserializeFromBinary(buffer.data() + offset, buffer.size() - offset);
}

}  // namespace spark_dsg
