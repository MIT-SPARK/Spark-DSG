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
#include "spark_dsg/attributes/place_node_attributes.h"

#include "attribute_utilities.h"
#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

using serialization::RegistrationInfo;
using NodeRegistry = serialization::AttributeRegistry<NodeAttributes>;
using attributes_detail::quaternionsEqual;

const RegistrationInfo& PlaceNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("PlaceNodeAttributes");
  return info;
}

const RegistrationInfo& Place2dNodeAttributes::registrationImpl() const {
  static const auto info = NodeRegistry::registration("Place2dNodeAttributes");
  return info;
}

bool operator==(const NearestVertexInfo& lhs, const NearestVertexInfo& rhs) {
  const auto block_equal = Eigen::Map<const Eigen::Vector3i>(lhs.block) ==
                           Eigen::Map<const Eigen::Vector3i>(rhs.block);
  const auto pos_equal = Eigen::Map<const Eigen::Vector3d>(lhs.voxel_pos) ==
                         Eigen::Map<const Eigen::Vector3d>(rhs.voxel_pos);
  return block_equal && pos_equal && lhs.vertex == rhs.vertex && lhs.label == rhs.label;
}

PlaceNodeAttributes::PlaceNodeAttributes() : PlaceNodeAttributes(0.0, 0) {}

PlaceNodeAttributes::PlaceNodeAttributes(double distance, unsigned int num_basis_points)
    : SemanticNodeAttributes(),
      distance(distance),
      num_basis_points(num_basis_points),
      frontier_scale(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()) {}

NodeAttributes::Ptr PlaceNodeAttributes::clone() const {
  return std::make_unique<PlaceNodeAttributes>(*this);
}

std::ostream& PlaceNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - distance: " << distance;
  out << "\n  - num basis points: " << num_basis_points;
  out << std::boolalpha << "\n  - real place: " << real_place;
  out << std::boolalpha << "\n  - need cleanup: " << need_cleanup;
  out << std::boolalpha << "\n  - active frontier: " << active_frontier;
  out << std::boolalpha << "\n  - anti frontier: " << active_frontier;
  out << "\n  - num frontier voxels: " << num_frontier_voxels;
  return out;
}

void PlaceNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("distance", distance);
  serialization::field("num_basis_points", num_basis_points);
  serialization::field("voxblox_mesh_connections", voxblox_mesh_connections);
  serialization::field("pcl_mesh_connections", pcl_mesh_connections);
  serialization::field("mesh_vertex_labels", mesh_vertex_labels);
  serialization::field("deformation_connections", deformation_connections);
  serialization::field("real_place", real_place);
  serialization::field("active_frontier", active_frontier);
  serialization::field("frontier_scale", frontier_scale);
  serialization::field("orientation", orientation);
  serialization::field("need_cleanup", need_cleanup);
  serialization::field("num_frontier_voxels", num_frontier_voxels);
  const auto& version = io::GlobalInfo::loadedVersion();
  if (version < io::Version(1, 1, 3)) {
    io::GlobalInfo::warnOutdated();
  } else {
    serialization::field("anti_frontier", anti_frontier);
  }
}

bool PlaceNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const PlaceNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return distance == derived->distance &&
         num_basis_points == derived->num_basis_points &&
         voxblox_mesh_connections == derived->voxblox_mesh_connections &&
         pcl_mesh_connections == derived->pcl_mesh_connections &&
         mesh_vertex_labels == derived->mesh_vertex_labels &&
         deformation_connections == derived->deformation_connections &&
         real_place == derived->real_place &&
         active_frontier == derived->active_frontier &&
         anti_frontier == derived->anti_frontier &&
         frontier_scale == derived->frontier_scale &&
         quaternionsEqual(orientation, derived->orientation) &&
         need_cleanup == derived->need_cleanup &&
         num_frontier_voxels == derived->num_frontier_voxels;
}

Place2dNodeAttributes::Place2dNodeAttributes()
    : SemanticNodeAttributes(),
      min_mesh_index(0),
      max_mesh_index(0),
      need_finish_merge(false),
      has_active_mesh_indices(false) {}

NodeAttributes::Ptr Place2dNodeAttributes::clone() const {
  return std::make_unique<Place2dNodeAttributes>(*this);
}

std::ostream& Place2dNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - boundary.size(): " << boundary.size();
  return out;
}

void Place2dNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  const auto& version = io::GlobalInfo::loadedVersion();
  if (version < io::Version(1, 1, 4)) {
    io::GlobalInfo::warnOutdated();
    serialization::field("boundary", boundary);
    serialization::field("ellipse_centroid", ellipse_centroid);
    serialization::field("ellipse_matrix_compress", ellipse_matrix_compress);
    serialization::field("ellipse_matrix_expand", ellipse_matrix_expand);
    serialization::field("pcl_boundary_connections", boundary_connections);
    {  // temp
      std::vector<NearestVertexInfo> temp;
      serialization::field("voxblox_mesh_connections", temp);
    }

    serialization::field("pcl_mesh_connections", mesh_connections);
    {  // temp scope
      std::vector<uint8_t> temp;
      serialization::field("mesh_vertex_labels", temp);
    }

    {  // temp scope
      std::vector<size_t> temp;
      serialization::field("deformation_connections", temp);
    }

    {  // temp scope
      bool temp;
      serialization::field("need_cleanup_splitting", temp);
    }

    serialization::field("has_active_mesh_indices", has_active_mesh_indices);
  } else {
    serialization::field("mesh_connections", mesh_connections);
    serialization::field("boundary_connections", boundary_connections);
    serialization::field("boundary", boundary);
    serialization::field("ellipse_centroid", ellipse_centroid);
    serialization::field("ellipse_matrix_compress", ellipse_matrix_compress);
    serialization::field("ellipse_matrix_expand", ellipse_matrix_expand);
    serialization::field("min_mesh_index", min_mesh_index);
    serialization::field("max_mesh_index", max_mesh_index);
    serialization::field("need_finish_merge", need_finish_merge);
    serialization::field("has_active_mesh_indices", has_active_mesh_indices);
  }
}

bool Place2dNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const Place2dNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return mesh_connections == derived->mesh_connections &&
         boundary_connections == derived->boundary_connections &&
         boundary == derived->boundary &&
         ellipse_centroid == derived->ellipse_centroid &&
         ellipse_matrix_compress == derived->ellipse_matrix_compress &&
         ellipse_matrix_expand == derived->ellipse_matrix_expand &&
         min_mesh_index == derived->min_mesh_index &&
         max_mesh_index == derived->max_mesh_index &&
         need_finish_merge == derived->need_finish_merge &&
         has_active_mesh_indices == derived->has_active_mesh_indices;
}

}  // namespace spark_dsg
