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

#include "spark_dsg/mesh.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/file_io.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

void write_binary(serialization::BinarySerializer& serializer, const Mesh& mesh) {
  // Write the mesh configuration.
  serializer.write(mesh.has_colors);
  serializer.write(mesh.has_timestamps);
  serializer.write(mesh.has_labels);
  serializer.write(mesh.has_first_seen_stamps);

  // Write vertices.
  serializer.write(mesh.points);

  // NOTE(lschmid): I opted to save everything that is in the mesh, even if it is not
  // in accordance with the initial mesh spec. This should not matter if the meshes are
  // handled correctly but should save headaches if people want to use the meshes in
  // other ways.
  serializer.write(mesh.colors);
  serializer.write(mesh.stamps);
  serializer.write(mesh.labels);
  serializer.write(mesh.first_seen_stamps);

  // Write faces
  serializer.write(mesh.faces);
}

void read_binary(const serialization::BinaryDeserializer& deserializer, Mesh& mesh) {
  // Mesh flags.
  bool has_colors, has_timestamps, has_labels, has_first_seen_stamps;
  deserializer.read(has_colors);
  deserializer.read(has_timestamps);
  deserializer.read(has_labels);
  deserializer.read(has_first_seen_stamps);
  mesh = Mesh(has_colors, has_timestamps, has_labels, has_first_seen_stamps);

  // Various attribute fields
  deserializer.read(mesh.points);
  deserializer.read(mesh.colors);
  deserializer.read(mesh.stamps);
  deserializer.read(mesh.labels);
  deserializer.read(mesh.first_seen_stamps);

  // Faces.
  deserializer.read(mesh.faces);
}

void Mesh::serializeToBinary(std::vector<uint8_t>& buffer) const {
  serialization::BinarySerializer serializer(&buffer);
  serializer.write(*this);
}

Mesh::Ptr Mesh::deserializeFromBinary(const uint8_t* const buffer, size_t length) {
  serialization::BinaryDeserializer deserializer(buffer, length);

  auto mesh = std::make_shared<Mesh>();
  deserializer.read(*mesh);
  return mesh;
}

void Mesh::save(std::filesystem::path filepath) const {
  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    throw std::runtime_error("json is unsupported");
  }

  const auto header_buffer = io::FileHeader::current().serialize();
  std::vector<uint8_t> mesh_buffer;
  serializeToBinary(mesh_buffer);

  // Write the header and graph data to the file.
  std::ofstream out(filepath, std::ios::out | std::ios::binary);
  out.write(reinterpret_cast<const char*>(header_buffer.data()), header_buffer.size());
  out.write(reinterpret_cast<const char*>(mesh_buffer.data()), mesh_buffer.size());
}

Mesh::Ptr Mesh::load(std::filesystem::path filepath) {
  if (!std::filesystem::exists(filepath)) {
    throw std::runtime_error("mesh file does not exist: " + filepath.string());
  }

  const auto type = io::verifyFileExtension(filepath);
  if (type == io::FileType::JSON) {
    throw std::runtime_error("json is unsupported");
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

  // NOTE(nathan) if we change the mesh format, we should add a version check here
  return deserializeFromBinary(buffer.data() + offset, buffer.size() - offset);
}

}  // namespace spark_dsg
