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
#pragma once
#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/graph_file_io.h"
#include "spark_dsg/node_attributes.h"

namespace spark_dsg {
namespace attributes {

template <typename Converter>
void serialize(Converter& converter, const NodeAttributes& attrs) {
  converter.write("position", attrs.position);
  converter.write("last_update_time_ns", attrs.last_update_time_ns);
  converter.write("is_active", attrs.is_active);
}

template <typename Converter>
void deserialize(const Converter& converter, NodeAttributes& attrs) {
  converter.read("position", attrs.position);
  converter.read("last_update_time_ns", attrs.last_update_time_ns);
  converter.read("is_active", attrs.is_active);
}

template <typename Converter>
void serialize(Converter& converter, const SemanticNodeAttributes& attrs) {
  serialize(converter, static_cast<const NodeAttributes&>(attrs));
  converter.write("name", attrs.name);
  converter.write("color", attrs.color);
  converter.write("bounding_box", attrs.bounding_box);
  converter.write("semantic_label", attrs.semantic_label);
}

template <typename Converter>
void deserialize(const Converter& converter, SemanticNodeAttributes& attrs) {
  deserialize(converter, static_cast<NodeAttributes&>(attrs));
  converter.read("name", attrs.name);
  converter.read("color", attrs.color);
  converter.read("bounding_box", attrs.bounding_box);
  converter.read("semantic_label", attrs.semantic_label);
}

template <typename Converter>
void serialize(Converter& converter, const ObjectNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
  converter.write("mesh_connections", attrs.mesh_connections);
  converter.write("registered", attrs.registered);
  converter.write("world_R_object", attrs.world_R_object);
}

template <typename Converter>
void deserialize(const Converter& converter, ObjectNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("mesh_connections", attrs.mesh_connections);
  converter.read("registered", attrs.registered);
  converter.read("world_R_object", attrs.world_R_object);
}

template <typename Converter>
void serialize(Converter& converter, const KhronosObjectAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));

  // Save time information.
  converter.write("first_observed_ns", attrs.first_observed_ns);
  converter.write("last_observed_ns", attrs.last_observed_ns);

  // Save mesh in binary form.
  std::vector<uint8_t> buffer;
  attrs.mesh.serializeToBinary(buffer);
  converter.write("mesh", buffer);

  // Store all other attributes.
  converter.write("trajectory_positions", attrs.trajectory_positions);
  converter.write("trajectory_timestamps", attrs.trajectory_timestamps);
  converter.write("dynamic_object_points", attrs.dynamic_object_points);
  // NOTE(lschmid): It may well be that json serialization does not support this.
  converter.write("details", attrs.details);
}

template <typename Converter>
void deserialize(const Converter& converter, KhronosObjectAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  const auto header = io::GlobalInfo::loadedHeader();

  // Load time information.
  converter.read("first_observed_ns", attrs.first_observed_ns);
  converter.read("last_observed_ns", attrs.last_observed_ns);

  // Load mesh.
  if (header.version < io::FileHeader::Version(1, 0, 1)) {
    // Support Legacy Meshes.
    std::vector<float> xyz;
    std::vector<uint8_t> rgba;
    std::vector<uint32_t> faces;
    converter.read("vertices", xyz);
    converter.read("colors", rgba);
    converter.read("faces", faces);
    const size_t num_vertices = xyz.size() / 3;
    const size_t num_faces = faces.size() / 3;
    attrs.mesh.resizeVertices(num_vertices);
    attrs.mesh.resizeFaces(num_faces);
    for (size_t i = 0; i < num_vertices; ++i) {
      attrs.mesh.setPos(i, {xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]});
      attrs.mesh.setColor(
          i, {rgba[4 * i], rgba[4 * i + 1], rgba[4 * i + 2], rgba[4 * i + 3]});
    }
    for (size_t i = 0; i < num_faces; ++i) {
      auto& face = attrs.mesh.face(i);
      for (size_t j = 0; j < 3; ++j) {
        face[j] = faces[3 * i + j];
      }
    }
    // Load trajectory by unraveling the packed positions.
    std::vector<float> trajectory;
    converter.read("trajectory_positions", trajectory);
    converter.read("trajectory_timestamps", attrs.trajectory_timestamps);
    const size_t num_trajectory_points = trajectory.size() / 3;
    attrs.trajectory_positions.resize(num_trajectory_points);
    for (size_t i = 0; i < num_trajectory_points; ++i) {
      Eigen::Vector3f& pos = attrs.trajectory_positions[i];
      pos.x() = trajectory[3 * i];
      pos.y() = trajectory[3 * i + 1];
      pos.z() = trajectory[3 * i + 2];
    }
    return;
  }

  std::vector<uint8_t> buffer;
  converter.read("mesh", buffer);
  attrs.mesh = *Mesh::deserializeFromBinary(buffer.data(), buffer.size());

  converter.read("trajectory_positions", attrs.trajectory_positions);
  converter.read("trajectory_timestamps", attrs.trajectory_timestamps);
  converter.read("dynamic_object_points", attrs.dynamic_object_points);
  converter.read("details", attrs.details);
}

template <typename Converter>
void serialize(Converter& converter, const RoomNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
}

template <typename Converter>
void deserialize(const Converter& converter, RoomNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
}

template <typename Converter>
void serialize(Converter& converter, const PlaceNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
  converter.write("distance", attrs.distance);
  converter.write("num_basis_points", attrs.num_basis_points);
  converter.write("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.write("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.write("mesh_vertex_labels", attrs.mesh_vertex_labels);
  converter.write("deformation_connections", attrs.deformation_connections);
}

template <typename Converter>
void deserialize(const Converter& converter, PlaceNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("distance", attrs.distance);
  converter.read("num_basis_points", attrs.num_basis_points);
  converter.read("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.read("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.read("mesh_vertex_labels", attrs.mesh_vertex_labels);
  converter.read("deformation_connections", attrs.deformation_connections);
}

template <typename Converter>
void serialize(Converter& converter, const AgentNodeAttributes& attrs) {
  serialize(converter, static_cast<const NodeAttributes&>(attrs));
  converter.write("world_R_body", attrs.world_R_body);
  converter.write("external_key", attrs.external_key);
  converter.write("dbow_ids", attrs.dbow_ids);
  converter.write("dbow_values", attrs.dbow_values);
}

template <typename Converter>
void deserialize(const Converter& converter, AgentNodeAttributes& attrs) {
  deserialize(converter, static_cast<NodeAttributes&>(attrs));
  converter.read("world_R_body", attrs.world_R_body);
  converter.read("external_key", attrs.external_key);
  converter.read("dbow_ids", attrs.dbow_ids);
  converter.read("dbow_values", attrs.dbow_values);
}

template <typename Converter>
void serialize(Converter& converter, const EdgeAttributes& attrs) {
  converter.write("weighted", attrs.weighted);
  converter.write("weight", attrs.weight);
}

template <typename Converter>
void deserialize(const Converter& converter, EdgeAttributes& attrs) {
  converter.read("weighted", attrs.weighted);
  converter.read("weight", attrs.weight);
}

}  // namespace attributes
}  // namespace spark_dsg
