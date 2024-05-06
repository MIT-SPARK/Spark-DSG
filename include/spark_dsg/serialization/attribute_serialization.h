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
#include "spark_dsg/serialization/versioning.h"

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
  converter.write("semantic_feature", attrs.semantic_feature);
}

template <typename Converter>
void deserialize(const Converter& converter, SemanticNodeAttributes& attrs) {
  deserialize(converter, static_cast<NodeAttributes&>(attrs));
  converter.read("name", attrs.name);
  converter.read("color", attrs.color);
  converter.read("bounding_box", attrs.bounding_box);
  converter.read("semantic_label", attrs.semantic_label);
  converter.read("semantic_feature", attrs.semantic_feature);
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
  converter.write("last_observed_ns", attrs.last_observed_ns);
  converter.write("mesh", attrs.mesh);
  converter.write("trajectory_positions", attrs.trajectory_positions);
  converter.write("trajectory_timestamps", attrs.trajectory_timestamps);
  converter.write("dynamic_object_points", attrs.dynamic_object_points);
  converter.write("details", attrs.details);
}

template <typename Converter>
void deserialize(const Converter& converter, KhronosObjectAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("first_observed_ns", attrs.first_observed_ns);
  converter.read("last_observed_ns", attrs.last_observed_ns);
  converter.read("mesh", attrs.mesh);
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
  converter.write("real_place", attrs.real_place);
  converter.write("predicted_place", attrs.predicted_place);
  converter.write("active_frontier", attrs.active_frontier);
  converter.write("frontier_scale", attrs.frontier_scale);
  converter.write("orientation", attrs.orientation);
  converter.write("need_cleanup", attrs.need_cleanup);
  converter.write("num_frontier_voxels", attrs.num_frontier_voxels);
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
  converter.read("real_place", attrs.real_place);
  converter.read("predicted_place", attrs.predicted_place);
  converter.read("active_frontier", attrs.active_frontier);
  converter.read("frontier_scale", attrs.frontier_scale);
  converter.read("orientation", attrs.orientation);
  converter.read("need_cleanup", attrs.need_cleanup);
  converter.read("num_frontier_voxels", attrs.num_frontier_voxels);
}

template <typename Converter>
void serialize(Converter& converter, const Place2dNodeAttributes& attrs) {
  serialize(converter, static_cast<const SemanticNodeAttributes&>(attrs));
  converter.write("boundary", attrs.boundary);
  converter.write("ellipse_centroid", attrs.ellipse_centroid);
  converter.write("ellipse_matrix_compress", attrs.ellipse_matrix_compress);
  converter.write("ellipse_matrix_expand", attrs.ellipse_matrix_expand);
  converter.write("pcl_boundary_connections", attrs.pcl_boundary_connections);
  converter.write("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.write("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.write("mesh_vertex_labels", attrs.mesh_vertex_labels);
  converter.write("deformation_connections", attrs.deformation_connections);
  converter.write("need_cleanup_splitting", attrs.need_cleanup_splitting);
  converter.write("has_active_mesh_indices", attrs.has_active_mesh_indices);
}

template <typename Converter>
void deserialize(const Converter& converter, Place2dNodeAttributes& attrs) {
  deserialize(converter, static_cast<SemanticNodeAttributes&>(attrs));
  converter.read("boundary", attrs.boundary);
  converter.read("ellipse_centroid", attrs.ellipse_centroid);
  converter.read("ellipse_matrix_compress", attrs.ellipse_matrix_compress);
  converter.read("ellipse_matrix_expand", attrs.ellipse_matrix_expand);
  converter.read("pcl_boundary_connections", attrs.pcl_boundary_connections);
  converter.read("voxblox_mesh_connections", attrs.voxblox_mesh_connections);
  converter.read("pcl_mesh_connections", attrs.pcl_mesh_connections);
  converter.read("mesh_vertex_labels", attrs.mesh_vertex_labels);
  converter.read("deformation_connections", attrs.deformation_connections);
  converter.read("need_cleanup_splitting", attrs.need_cleanup_splitting);
  converter.read("has_active_mesh_indices", attrs.has_active_mesh_indices);
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
