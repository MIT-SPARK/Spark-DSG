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
#include "spark_dsg/serialization/binary_conversions.h"

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/mesh.h"
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/serialization/attribute_serialization.h"

namespace spark_dsg {

void read_binary(const serialization::BinaryDeserializer& s, BoundingBox& box) {
  s.checkFixedArrayLength(4);

  int32_t raw_type;
  s.read(raw_type);
  box.type = static_cast<BoundingBox::Type>(raw_type);
  s.read(box.dimensions);

  s.read(box.world_P_center);
  s.read(box.world_R_center);
}

void write_binary(serialization::BinarySerializer& s, const BoundingBox& box) {
  s.startFixedArray(4);
  s.write(static_cast<int32_t>(box.type));
  s.write(box.dimensions);
  s.write(box.world_P_center);
  s.write(box.world_R_center);
}

void write_binary(serialization::BinarySerializer& s, const LayerKey& key) {
  s.startFixedArray(2);
  s.write(key.layer);
  s.write(key.partition);
}

void read_binary(const serialization::BinaryDeserializer& s, LayerKey& key) {
  s.checkFixedArrayLength(2);
  s.read(key.layer);
  s.read(key.partition);
}

void read_binary(const serialization::BinaryDeserializer& s, NearestVertexInfo& info) {
  s.checkFixedArrayLength(4);

  s.checkFixedArrayLength(3);
  s.read(info.block[0]);
  s.read(info.block[1]);
  s.read(info.block[2]);

  s.checkFixedArrayLength(3);
  s.read(info.voxel_pos[0]);
  s.read(info.voxel_pos[1]);
  s.read(info.voxel_pos[2]);

  s.read(info.vertex);
  s.read(info.label);
}

void write_binary(serialization::BinarySerializer& s, const NearestVertexInfo& info) {
  s.startFixedArray(4);

  s.startFixedArray(3);
  s.write(info.block[0]);
  s.write(info.block[1]);
  s.write(info.block[2]);

  s.startFixedArray(3);
  s.write(info.voxel_pos[0]);
  s.write(info.voxel_pos[1]);
  s.write(info.voxel_pos[2]);

  s.write(info.vertex);
  s.write(info.label);
}

void read_binary(const serialization::BinaryDeserializer& s, Color& c) {
  s.read(c.r);
  s.read(c.g);
  s.read(c.b);
  s.read(c.a);
}

void write_binary(serialization::BinarySerializer& s, const Color& c) {
  s.write(c.r);
  s.write(c.g);
  s.write(c.b);
  s.write(c.a);
}

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

void write_binary(serialization::BinarySerializer& s, const NodeAttributes& attrs) {
  serialization::Visitor::to(s, attrs);
}

void write_binary(serialization::BinarySerializer& s, const EdgeAttributes& attrs) {
  serialization::Visitor::to(s, attrs);
}

}  // namespace spark_dsg
