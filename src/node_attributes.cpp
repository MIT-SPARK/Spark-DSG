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
#include "spark_dsg/node_attributes.h"

#include "spark_dsg/serialization/attribute_serialization.h"
#include "spark_dsg/serialization/binary_conversions.h"
#include "spark_dsg/serialization/json_conversions.h"
#include "spark_dsg/serialization/versioning.h"

namespace spark_dsg {

template <typename T>
std::string showIterable(const T& iterable, size_t max_length = 80) {
  std::stringstream ss;
  ss << "[";
  auto iter = iterable.begin();
  while (iter != iterable.end()) {
    ss << *iter;

    ++iter;
    if (iter != iterable.end()) {
      ss << ", ";
    }

    if (max_length && ss.str().size() >= max_length) {
      ss << "...";
      break;
    }
  }
  ss << "]";

  return ss.str();
}

template <typename Scalar>
std::string quatToString(const Eigen::Quaternion<Scalar>& q) {
  std::stringstream ss;
  ss << "{w: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "}";
  return ss.str();
}

template <typename Derived>
bool matricesEqual(const Eigen::DenseBase<Derived>& lhs,
                   const Eigen::DenseBase<Derived>& rhs) {
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
    return false;
  }

  bool same = true;
  for (int r = 0; r < lhs.rows(); ++r) {
    for (int c = 0; c < lhs.cols(); ++c) {
      const auto lhs_nan = std::isnan(lhs(r, c));
      const auto rhs_nan = std::isnan(rhs(r, c));
      // if one value is nan, this still works
      same &= (lhs_nan && rhs_nan) || lhs(r, c) == rhs(r, c);
    }
  }

  return same;
}

bool operator==(const NearestVertexInfo& lhs, const NearestVertexInfo& rhs) {
  const auto block_equal = Eigen::Map<const Eigen::Vector3i>(lhs.block) ==
                           Eigen::Map<const Eigen::Vector3i>(rhs.block);
  const auto pos_equal = Eigen::Map<const Eigen::Vector3d>(lhs.voxel_pos) ==
                         Eigen::Map<const Eigen::Vector3d>(rhs.voxel_pos);
  return block_equal && pos_equal && lhs.vertex == rhs.vertex && lhs.label == rhs.label;
}

std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs) {
  return attrs.fill_ostream(out);
}

NodeAttributes::NodeAttributes() : NodeAttributes(Eigen::Vector3d::Zero()) {}

NodeAttributes::NodeAttributes(const Eigen::Vector3d& pos)
    : position(pos), last_update_time_ns(0), is_active(false), is_predicted(false) {}

NodeAttributes::Ptr NodeAttributes::clone() const {
  return std::make_unique<NodeAttributes>(*this);
}

bool NodeAttributes::operator==(const NodeAttributes& other) const {
  return is_equal(other);
}

std::ostream& NodeAttributes::fill_ostream(std::ostream& out) const {
  auto format = getDefaultVectorFormat();
  out << "  - position: " << position.transpose().format(format) << "\n";
  out << "  - last update time: "
      << (last_update_time_ns == 0 ? "n/a" : std::to_string(last_update_time_ns))
      << "\n";
  out << std::boolalpha << "  - is_active: " << is_active;
  out << std::boolalpha << "  - is_predicted: " << is_predicted;
  return out;
}

void NodeAttributes::serialization_info() {
  serialization::field("position", position);
  serialization::field("last_update_time_ns", last_update_time_ns);
  serialization::field("is_active", is_active);
  const auto& header = io::GlobalInfo::loadedHeader();
  if (header.version < io::Version(1, 0, 4)) {
    io::warnOutdatedHeader(header);
  } else {
    serialization::field("is_predicted", is_predicted);
  }
}

void NodeAttributes::serialization_info() const {
  const_cast<NodeAttributes*>(this)->serialization_info();
}

bool NodeAttributes::is_equal(const NodeAttributes& other) const {
  return matricesEqual(position, other.position) &&
         last_update_time_ns == other.last_update_time_ns &&
         is_active == other.is_active && is_predicted == other.is_predicted;
}

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(),
      name(""),
      semantic_label(NO_SEMANTIC_LABEL),
      semantic_feature(0, 0) {}

NodeAttributes::Ptr SemanticNodeAttributes::clone() const {
  return std::make_unique<SemanticNodeAttributes>(*this);
}

bool SemanticNodeAttributes::hasLabel() const {
  return semantic_label != NO_SEMANTIC_LABEL;
}

bool SemanticNodeAttributes::hasFeature() const {
  return semantic_feature.rows() * semantic_feature.cols() != 0;
}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << "\n  - color: " << color << "\n"
      << "  - name: '" << name << "'\n"
      << "  - bounding box: " << bounding_box << "\n"
      << "  - label: " << std::to_string(semantic_label) << "\n"
      << "  - feature: [" << semantic_feature.rows() << " x " << semantic_feature.cols()
      << "]";
  return out;
}

void SemanticNodeAttributes::serialization_info() {
  NodeAttributes::serialization_info();
  serialization::field("name", name);
  serialization::field("color", color);
  serialization::field("bounding_box", bounding_box);
  serialization::field("semantic_label", semantic_label);
  serialization::field("semantic_feature", semantic_feature);
}

bool SemanticNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const SemanticNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!NodeAttributes::is_equal(other)) {
    return false;
  }

  return name == derived->name && color == derived->color &&
         bounding_box == derived->bounding_box &&
         semantic_label == derived->semantic_label &&
         semantic_feature == derived->semantic_feature;
}

ObjectNodeAttributes::ObjectNodeAttributes()
    : SemanticNodeAttributes(),
      registered(false),
      world_R_object(Eigen::Quaterniond::Identity()) {}

NodeAttributes::Ptr ObjectNodeAttributes::clone() const {
  return std::make_unique<ObjectNodeAttributes>(*this);
}

std::ostream& ObjectNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - mesh_connections: " << showIterable(mesh_connections);
  out << "\n  - registered?: " << (registered ? "yes" : "no");
  out << "\n  - world_R_object: " << quatToString(world_R_object);
  return out;
}

void ObjectNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("mesh_connections", mesh_connections);
  serialization::field("registered", registered);
  serialization::field("world_R_object", world_R_object);
}

template <typename Scalar>
bool quaternionsEqual(const Eigen::Quaternion<Scalar>& lhs,
                      const Eigen::Quaternion<Scalar>& rhs) {
  return lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() &&
         lhs.z() == rhs.z();
}

bool ObjectNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const ObjectNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return mesh_connections == derived->mesh_connections &&
         registered == derived->registered &&
         quaternionsEqual(world_R_object, derived->world_R_object);
}

RoomNodeAttributes::RoomNodeAttributes() : SemanticNodeAttributes() {}

NodeAttributes::Ptr RoomNodeAttributes::clone() const {
  return std::make_unique<RoomNodeAttributes>(*this);
}

std::ostream& RoomNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  return out;
}

void RoomNodeAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("semantic_class_probabilities", semantic_class_probabilities);
}

bool RoomNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const RoomNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  return SemanticNodeAttributes::is_equal(other);
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
         frontier_scale == derived->frontier_scale &&
         quaternionsEqual(orientation, derived->orientation) &&
         need_cleanup == derived->need_cleanup &&
         num_frontier_voxels == derived->num_frontier_voxels;
}

Place2dNodeAttributes::Place2dNodeAttributes()
    : Place2dNodeAttributes(std::vector<Eigen::Vector3d>()) {}

Place2dNodeAttributes::Place2dNodeAttributes(std::vector<Eigen::Vector3d> boundary)
    : SemanticNodeAttributes(),
      boundary(boundary),
      pcl_min_index(0),
      pcl_max_index(0),
      need_finish_merge(false),
      need_cleanup_splitting(false),
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
  serialization::field("boundary", boundary);
  serialization::field("ellipse_centroid", ellipse_centroid);
  serialization::field("ellipse_matrix_compress", ellipse_matrix_compress);
  serialization::field("ellipse_matrix_expand", ellipse_matrix_expand);
  serialization::field("pcl_boundary_connections", pcl_boundary_connections);
  serialization::field("voxblox_mesh_connections", voxblox_mesh_connections);
  serialization::field("pcl_mesh_connections", pcl_mesh_connections);
  serialization::field("mesh_vertex_labels", mesh_vertex_labels);
  serialization::field("deformation_connections", deformation_connections);
  serialization::field("need_cleanup_splitting", need_cleanup_splitting);
  serialization::field("has_active_mesh_indices", has_active_mesh_indices);
}

bool Place2dNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const Place2dNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!SemanticNodeAttributes::is_equal(other)) {
    return false;
  }

  return boundary == derived->boundary &&
         ellipse_centroid == derived->ellipse_centroid &&
         ellipse_matrix_compress == derived->ellipse_matrix_compress &&
         ellipse_matrix_expand == derived->ellipse_matrix_expand &&
         pcl_boundary_connections == derived->pcl_boundary_connections &&
         voxblox_mesh_connections == derived->voxblox_mesh_connections &&
         pcl_mesh_connections == derived->pcl_mesh_connections &&
         mesh_vertex_labels == derived->mesh_vertex_labels &&
         deformation_connections == derived->deformation_connections &&
         need_cleanup_splitting == derived->need_cleanup_splitting &&
         has_active_mesh_indices == derived->has_active_mesh_indices;
}

AgentNodeAttributes::AgentNodeAttributes() : NodeAttributes() {}

AgentNodeAttributes::AgentNodeAttributes(const Eigen::Quaterniond& world_R_body,
                                         const Eigen::Vector3d& world_P_body,
                                         NodeId external_key)
    : NodeAttributes(world_P_body),
      world_R_body(world_R_body),
      external_key(external_key) {}

NodeAttributes::Ptr AgentNodeAttributes::clone() const {
  return std::make_unique<AgentNodeAttributes>(*this);
}

std::ostream& AgentNodeAttributes::fill_ostream(std::ostream& out) const {
  NodeAttributes::fill_ostream(out);
  out << "\n  - orientation: " << quatToString(world_R_body);
  return out;
}

void AgentNodeAttributes::serialization_info() {
  NodeAttributes::serialization_info();
  serialization::field("world_R_body", world_R_body);
  serialization::field("external_key", external_key);
  serialization::field("dbow_ids", dbow_ids);
  serialization::field("dbow_values", dbow_values);
}

bool AgentNodeAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const AgentNodeAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!NodeAttributes::is_equal(other)) {
    return false;
  }

  return quaternionsEqual(world_R_body, derived->world_R_body) &&
         external_key == derived->external_key && dbow_ids == derived->dbow_ids &&
         dbow_values == derived->dbow_values;
}

KhronosObjectAttributes::KhronosObjectAttributes() : mesh(true, false, false){};

NodeAttributes::Ptr KhronosObjectAttributes::clone() const {
  return std::make_unique<KhronosObjectAttributes>(*this);
}

std::ostream& KhronosObjectAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  out << "\n  - first_observed_ns: ";
  for (uint64_t t : first_observed_ns) {
    out << t << " ";
  }
  out << "\n  - last_observed_ns: ";
  for (uint64_t t : last_observed_ns) {
    out << t << " ";
  }
  out << "\n  - mesh: " << mesh.numVertices() << " vertices, " << mesh.numFaces()
      << " faces";
  return out;
}

void KhronosObjectAttributes::serialization_info() {
  SemanticNodeAttributes::serialization_info();
  serialization::field("first_observed_ns", first_observed_ns);
  serialization::field("last_observed_ns", last_observed_ns);
  serialization::field("mesh", mesh);
  serialization::field("trajectory_positions", trajectory_positions);
  serialization::field("trajectory_timestamps", trajectory_timestamps);
  serialization::field("dynamic_object_points", dynamic_object_points);
  serialization::field("details", details);
}

bool KhronosObjectAttributes::is_equal(const NodeAttributes& other) const {
  const auto derived = dynamic_cast<const KhronosObjectAttributes*>(&other);
  if (!derived) {
    return false;
  }

  if (!ObjectNodeAttributes::is_equal(other)) {
    return false;
  }

  return first_observed_ns == derived->first_observed_ns &&
         last_observed_ns == derived->last_observed_ns && mesh == derived->mesh &&
         trajectory_positions == derived->trajectory_positions &&
         dynamic_object_points == derived->dynamic_object_points &&
         details == derived->details;
}

}  // namespace spark_dsg
