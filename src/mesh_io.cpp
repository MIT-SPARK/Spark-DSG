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
  const auto header = io::FileHeader::current();

  // Write the mesh configuration.
  serializer.write(has_colors);
  serializer.write(has_timestamps);
  serializer.write(has_labels);

  // Write vertices.
  serializer.startFixedArray(3 * points.size());
  for (const auto& point : points) {
    serializer.write(point.x());
    serializer.write(point.y());
    serializer.write(point.z());
  }

  // NOTE(lschmid): I opted to save everything that is in the mesh, even if it is not
  // in accordance with the initial mesh spec. This should not matter if the meshes are
  // handled correctly but should save headaches if people want to use the meshes in
  // other ways.
  serializer.startFixedArray(4 * colors.size());
  for (const auto& color : colors) {
    serializer.write(color.r);
    serializer.write(color.g);
    serializer.write(color.b);
    serializer.write(color.a);
  }

  serializer.startFixedArray(stamps.size());
  for (const auto& stamp : stamps) {
    serializer.write(stamp);
  }

  serializer.startFixedArray(labels.size());
  for (const auto& label : labels) {
    serializer.write(label);
  }

  if (header.project_name == "khronos") {
    serializer.startFixedArray(first_seen_stamps.size());
    for (const auto& stamp : first_seen_stamps) {
      serializer.write(stamp);
    }
  }

  // Faces.
  serializer.startFixedArray(3 * faces.size());
  for (const auto& face : faces) {
    serializer.write(face[0]);
    serializer.write(face[1]);
    serializer.write(face[2]);
  }
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

Mesh::Ptr deserializeLegacyMesh(const serialization::BinaryDeserializer& deserializer) {
  // NOTE(lschmid): Since the new serialization is completely different all of this is
  // factored out.
  const auto header = io::GlobalInfo::loadedHeader();
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

  // Stamps in old khronos format for compatibility.
  std::vector<uint64_t> stamps;
  if (header.project_name == "khronos") {
    deserializer.read(mesh->first_seen_stamps);
    deserializer.read(stamps);
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

  // Overwrite stamps w/ khronos last seenn stamps for now.
  if (!stamps.empty()) {
    mesh->stamps = stamps;
  }

  return mesh;
}

Mesh::Ptr Mesh::deserializeFromBinary(const uint8_t* const buffer, size_t length) {
  serialization::BinaryDeserializer deserializer(buffer, length);
  const auto header = io::GlobalInfo::loadedHeader();

  if (header.version < io::FileHeader::Version(1, 0, 1)) {
    // Load legacy mesh.
    return deserializeLegacyMesh(deserializer);
  }

  // Mesh flags.
  bool has_colors, has_timestamps, has_labels;
  deserializer.read(has_colors);
  deserializer.read(has_timestamps);
  deserializer.read(has_labels);
  auto mesh = std::make_shared<Mesh>(has_colors, has_timestamps, has_labels);

  // Vertices.
  const size_t num_points = deserializer.readFixedArrayLength() / 3;
  mesh->points.resize(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    auto& point = mesh->points.at(i);
    deserializer.read(point.x());
    deserializer.read(point.y());
    deserializer.read(point.z());
  }

  const size_t num_colors = deserializer.readFixedArrayLength() / 4;
  mesh->colors.resize(num_colors);
  for (size_t i = 0; i < num_colors; ++i) {
    Color& color = mesh->colors.at(i);
    deserializer.read(color.r);
    deserializer.read(color.g);
    deserializer.read(color.b);
    deserializer.read(color.a);
  }

  const size_t num_stamps = deserializer.readFixedArrayLength();
  mesh->stamps.resize(num_stamps);
  for (size_t i = 0; i < num_stamps; ++i) {
    deserializer.read(mesh->stamps.at(i));
  }

  const size_t num_labels = deserializer.readFixedArrayLength();
  mesh->labels.resize(num_labels);
  for (size_t i = 0; i < num_labels; ++i) {
    deserializer.read(mesh->labels.at(i));
  }

  if (header.project_name == "khronos") {
    const size_t num_first_seen_stamps = deserializer.readFixedArrayLength();
    mesh->first_seen_stamps.resize(num_first_seen_stamps);
    for (size_t i = 0; i < num_first_seen_stamps; ++i) {
      deserializer.read(mesh->first_seen_stamps.at(i));
    }
  }

  // Faces.
  const size_t num_faces = deserializer.readFixedArrayLength() / 3;
  mesh->faces.resize(num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    auto& face = mesh->faces.at(i);
    deserializer.read(face[0]);
    deserializer.read(face[1]);
    deserializer.read(face[2]);
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

  // TODO(lschmid): This check should probably be replaced to only consider thinsg
  // relevant to meshes.
  checkCompatibility(*header);
  return deserializeFromBinary(buffer.data() + offset, buffer.size() - offset);
}

}  // namespace spark_dsg
