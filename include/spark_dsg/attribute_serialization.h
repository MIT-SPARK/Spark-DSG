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
  converter.write("first_observed_ns", attrs.first_observed_ns);

  // Work around for mesh for now: Pack vertices and faces into vectors.
  std::vector<float> xyz;
  std::vector<uint8_t> rgba;
  std::vector<uint32_t> faces;
  xyz.reserve(3 * attrs.vertices.size());
  rgba.reserve(4 * attrs.vertices.size());
  faces.reserve(3 * attrs.faces.size());
  for (const auto& vertex : attrs.vertices) {
    xyz.emplace_back(vertex.x);
    xyz.emplace_back(vertex.y);
    xyz.emplace_back(vertex.z);
    rgba.emplace_back(vertex.r);
    rgba.emplace_back(vertex.g);
    rgba.emplace_back(vertex.b);
    rgba.emplace_back(vertex.a);
  }
  for (const auto& face : attrs.faces) {
    for (size_t i = 0; i < 3; ++i) {
      faces.emplace_back(face[i]);
    }
  }
  converter.write("vertices", xyz);
  converter.write("colors", rgba);
  converter.write("faces", faces);
}

template <typename Converter>
void deserialize(const Converter& converter, KhronosObjectAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("first_observed_ns", attrs.first_observed_ns);

  // Undo work around for mesh.
  // NOTE(lschmid): Could also check for validity here but unless someone messes with
  // the data it will be valid as constructed above.
  std::vector<float> xyz;
  std::vector<uint8_t> rgba;
  std::vector<uint32_t> faces;
  converter.read("vertices", xyz);
  converter.read("colors", rgba);
  converter.read("faces", faces);
  const size_t num_vertices = xyz.size() / 3;
  const size_t num_faces = faces.size() / 3;
  attrs.vertices.resize(num_vertices);
  attrs.faces.resize(num_faces);
  for (size_t i = 0; i < num_vertices; ++i) {
    pcl::PointXYZRGBA& vertex = attrs.vertices[i];
    vertex.x = xyz[3 * i];
    vertex.y = xyz[3 * i + 1];
    vertex.z = xyz[3 * i + 2];
    vertex.r = rgba[4 * i];
    vertex.g = rgba[4 * i + 1];
    vertex.b = rgba[4 * i + 2];
    vertex.a = rgba[4 * i + 3];
  }
  for (size_t i = 0; i < num_faces; ++i) {
    KhronosObjectAttributes::MeshFace& face = attrs.faces[i];
    for (size_t j = 0; j < 3; ++j) {
      face[j] = faces[3 * i + j];
    }
  }
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
