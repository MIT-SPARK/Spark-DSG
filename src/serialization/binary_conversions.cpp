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
#include "spark_dsg/node_attributes.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

void read_binary(const serialization::BinaryDeserializer& s, BoundingBox& box) {
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 0, 3)) {
    // Legacy bboxes with min/max encoding.
    s.checkFixedArrayLength(5);
    io::warnOutdatedHeader(header);
  } else {
    // New bboxes with dimensions encoding.
    s.checkFixedArrayLength(4);
  }

  int32_t raw_type;
  s.read(raw_type);
  box.type = static_cast<BoundingBox::Type>(raw_type);

  if (header.version < io::Version(1, 0, 3)) {
    Eigen::Vector3f min, max;
    s.read(min);
    s.read(max);
    box.dimensions = max - min;
  } else {
    s.read(box.dimensions);
  }

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

void read_binary(const serialization::BinaryDeserializer& s, NearestVertexInfo& info) {
  // array: [block_index, pos, vertex_index, label]
  s.checkFixedArrayLength(4);
  // block index
  s.checkFixedArrayLength(3);
  s.read(info.block[0]);
  s.read(info.block[1]);
  s.read(info.block[2]);
  // pos
  s.checkFixedArrayLength(3);
  s.read(info.voxel_pos[0]);
  s.read(info.voxel_pos[1]);
  s.read(info.voxel_pos[2]);
  // vertex
  s.read(info.vertex);
  // label
  s.read(info.label);
}

void write_binary(serialization::BinarySerializer& s, const NearestVertexInfo& info) {
  // array: [block_index, pos, vertex_index, label]
  s.startFixedArray(4);
  // block index
  s.startFixedArray(3);
  s.write(info.block[0]);
  s.write(info.block[1]);
  s.write(info.block[2]);
  // pos
  s.startFixedArray(3);
  s.write(info.voxel_pos[0]);
  s.write(info.voxel_pos[1]);
  s.write(info.voxel_pos[2]);
  // vertex
  s.write(info.vertex);
  // label
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

// TODO(nathan) mesh?

void write_binary(serialization::BinarySerializer& s, const NodeAttributes& attrs) {
  serialization::Visitor::to(s, attrs);
}

void write_binary(serialization::BinarySerializer& s, const EdgeAttributes& attrs) {
  serialization::Visitor::to(s, attrs);
}

namespace io {

void read_binary(const serialization::BinaryDeserializer& s, FileHeader& header) {
  s.read(header.project_name);
  s.read(header.version.major);
  s.read(header.version.minor);
  s.read(header.version.patch);
}

void write_binary(serialization::BinarySerializer& s, const FileHeader& header) {
  s.write(header.project_name);
  s.write(header.version.major);
  s.write(header.version.minor);
  s.write(header.version.patch);
}

}  // namespace io

}  // namespace spark_dsg
