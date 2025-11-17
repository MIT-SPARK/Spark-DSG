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

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/mesh.h>
#include <spark_dsg/node_attributes.h>

#include <filesystem>

#include "spark_dsg/python/mesh.h"
#include "spark_dsg/python/python_types.h"

namespace spark_dsg::python {

namespace py = pybind11;

using namespace py::literals;

void init_attributes(py::module_& m) {
  py::class_<NearestVertexInfo>(m, "NearestVertexInfo")
      .def(py::init<>())
      .def_property(
          "block",
          [](const NearestVertexInfo& info) -> Eigen::Vector3i {
            return {info.block[0], info.block[1], info.block[2]};
          },
          [](NearestVertexInfo& info, const Eigen::Vector3i& array) {
            info.block[0] = array.x();
            info.block[1] = array.y();
            info.block[2] = array.z();
          })
      .def_property(
          "voxel_pos",
          [](const NearestVertexInfo& info) -> Eigen::Vector3d {
            return {info.voxel_pos[0], info.voxel_pos[1], info.voxel_pos[2]};
          },
          [](NearestVertexInfo& info, const Eigen::Vector3d& array) {
            info.voxel_pos[0] = array.x();
            info.voxel_pos[1] = array.y();
            info.voxel_pos[2] = array.z();
          })
      .def_readwrite("vertex", &NearestVertexInfo::vertex)
      .def_readwrite("label", &NearestVertexInfo::label)
      .def("__repr__", [](const NearestVertexInfo& info) {
        std::stringstream ss;
        ss << std::setprecision(6) << "VertexInfo<block=[" << info.block[0] << ", " << info.block[1] << ", "
           << info.block[2] << "], pos=[" << info.voxel_pos[0] << ", " << info.voxel_pos[1] << ", " << info.voxel_pos[2]
           << "], label=";
        if (info.label) {
          ss << info.label.value();
        } else {
          ss << "n/a";
        }
        ss << ">";
        return ss.str();
      });

  py::class_<NodeAttributes>(m, "NodeAttributes")
      .def(py::init<>())
      .def_readwrite("position", &NodeAttributes::position)
      .def_readwrite("last_update_time_ns", &NodeAttributes::last_update_time_ns)
      .def_readwrite("is_active", &NodeAttributes::is_active)
      .def_readwrite("is_predicted", &NodeAttributes::is_predicted)
      .def_readwrite("_metadata", &NodeAttributes::metadata)
      .def("__repr__", [](const NodeAttributes& attrs) {
        std::stringstream ss;
        ss << attrs;
        return ss.str();
      });

  py::class_<SemanticNodeAttributes, NodeAttributes>(m, "SemanticNodeAttributes")
      .def(py::init<>())
      .def_readwrite("name", &SemanticNodeAttributes::name)
      .def_property(
          "color",
          [](const SemanticNodeAttributes& attrs) {
            Eigen::Matrix<uint8_t, 3, 1> color;
            color << attrs.color.r, attrs.color.g, attrs.color.b;
            return color;
          },
          [](SemanticNodeAttributes& attrs, const Eigen::Matrix<uint8_t, 3, 1> color) {
            attrs.color.r = color(0);
            attrs.color.g = color(1);
            attrs.color.b = color(2);
          })
      .def_readwrite("bounding_box", &SemanticNodeAttributes::bounding_box)
      .def_readwrite("semantic_label", &SemanticNodeAttributes::semantic_label)
      .def_readwrite("semantic_feature", &SemanticNodeAttributes::semantic_feature)
      .def_readonly_static("NO_SEMANTIC_LABEL", &SemanticNodeAttributes::NO_SEMANTIC_LABEL);

  py::class_<ObjectNodeAttributes, SemanticNodeAttributes>(m, "ObjectNodeAttributes")
      .def(py::init<>())
      .def_readwrite("registered", &ObjectNodeAttributes::registered)
      .def_readwrite("mesh_connections", &ObjectNodeAttributes::mesh_connections)
      .def_property(
          "world_R_object",
          [](const ObjectNodeAttributes& attrs) { return Quaternion(attrs.world_R_object); },
          [](ObjectNodeAttributes& attrs, const Quaternion& rot) { attrs.world_R_object = rot; });

  py::class_<KhronosObjectAttributes, ObjectNodeAttributes>(m, "KhronosObjectAttributes")
      .def(py::init<>())
      .def_readonly("first_observed_ns", &KhronosObjectAttributes::first_observed_ns)
      .def_readonly("last_observed_ns", &KhronosObjectAttributes::last_observed_ns)
      .def(
          "mesh",
          [](const KhronosObjectAttributes& attrs) { return &attrs.mesh; },
          py::return_value_policy::reference_internal)
      .def_readonly("trajectory_timestamps", &KhronosObjectAttributes::trajectory_timestamps)
      .def_readonly("trajectory_positions", &KhronosObjectAttributes::trajectory_positions)
      .def_readonly("dynamic_object_points", &KhronosObjectAttributes::dynamic_object_points)
      .def_readonly("details", &KhronosObjectAttributes::details);

  py::class_<RoomNodeAttributes, SemanticNodeAttributes>(m, "RoomNodeAttributes")
      .def(py::init<>())
      .def_readwrite("semantic_class_probabilities", &RoomNodeAttributes::semantic_class_probabilities);

  py::class_<PlaceNodeAttributes, SemanticNodeAttributes>(m, "PlaceNodeAttributes")
      .def(py::init<>())
      .def_readwrite("distance", &PlaceNodeAttributes::distance)
      .def_readwrite("num_basis_points", &PlaceNodeAttributes::num_basis_points)
      .def_readwrite("voxblox_mesh_connections", &PlaceNodeAttributes::voxblox_mesh_connections)
      .def_readwrite("pcl_mesh_connections", &PlaceNodeAttributes::pcl_mesh_connections)
      .def_readwrite("mesh_vertex_labels", &PlaceNodeAttributes::mesh_vertex_labels)
      .def_readwrite("deformation_connections", &PlaceNodeAttributes::deformation_connections)
      .def_readwrite("real_place", &PlaceNodeAttributes::real_place)
      .def_readwrite("active_frontier", &PlaceNodeAttributes::active_frontier)
      .def_readwrite("anti_frontier", &PlaceNodeAttributes::anti_frontier)
      .def_readwrite("frontier_scale", &PlaceNodeAttributes::frontier_scale)
      .def_property(
          "orientation",
          [](const PlaceNodeAttributes& attrs) { return Quaternion(attrs.orientation); },
          [](PlaceNodeAttributes& attrs, const Quaternion& rot) { attrs.orientation = rot; })
      .def_readwrite("num_frontier_voxels", &PlaceNodeAttributes::num_frontier_voxels)
      .def_readwrite("need_cleanup", &PlaceNodeAttributes::need_cleanup);

  py::class_<Place2dNodeAttributes, SemanticNodeAttributes>(m, "Place2dNodeAttributes")
      .def(py::init<>())
      .def_readwrite("boundary", &Place2dNodeAttributes::boundary)
      .def_readwrite("ellipse_matrix_compress", &Place2dNodeAttributes::ellipse_matrix_compress)
      .def_readwrite("ellipse_matrix_expand", &Place2dNodeAttributes::ellipse_matrix_expand)
      .def_readwrite("ellipse_centroid", &Place2dNodeAttributes::ellipse_centroid)
      .def_readwrite("pcl_boundary_connections", &Place2dNodeAttributes::pcl_boundary_connections)
      .def_readwrite("voxblox_mesh_connections", &Place2dNodeAttributes::voxblox_mesh_connections)
      .def_readwrite("pcl_mesh_connections", &Place2dNodeAttributes::pcl_mesh_connections)
      .def_readwrite("mesh_vertex_labels", &Place2dNodeAttributes::mesh_vertex_labels)
      .def_readwrite("deformation_connections", &Place2dNodeAttributes::deformation_connections);

  py::class_<BoundaryInfo>(m, "BoundaryInfo")
      .def(py::init<>())
      .def_readwrite("min", &BoundaryInfo::min)
      .def_readwrite("max", &BoundaryInfo::max)
      .def_readwrite("states", &BoundaryInfo::states);

  py::enum_<TraversabilityState>(m, "TraversabilityState")
      .value("UNKNOWN", TraversabilityState::UNKNOWN)
      .value("TRAVERSABLE", TraversabilityState::TRAVERSABLE)
      .value("INTRAVERSABLE", TraversabilityState::INTRAVERSABLE)
      .value("TRAVERSED", TraversabilityState::TRAVERSED);

  py::class_<TraversabilityNodeAttributes, NodeAttributes>(m, "TraversabilityNodeAttributes")
      .def(py::init<>())
      .def_readwrite("boundary", &TraversabilityNodeAttributes::boundary)
      .def_readwrite("first_observed_ns", &TraversabilityNodeAttributes::first_observed_ns)
      .def_readwrite("last_observed_ns", &TraversabilityNodeAttributes::last_observed_ns)
      .def_readwrite("distance", &TraversabilityNodeAttributes::distance);

  py::class_<AgentNodeAttributes, NodeAttributes>(m, "AgentNodeAttributes")
      .def(py::init<>())
      .def_property(
          "world_R_body",
          [](const AgentNodeAttributes& attrs) { return Quaternion(attrs.world_R_body); },
          [](AgentNodeAttributes& attrs, const Quaternion& rot) { attrs.world_R_body = rot; })
      .def_readwrite("timestamp", &AgentNodeAttributes::timestamp)
      .def_readwrite("external_key", &AgentNodeAttributes::external_key)
      .def_readwrite("dbow_ids", &AgentNodeAttributes::dbow_ids)
      .def_readwrite("dbow_values", &AgentNodeAttributes::dbow_values);

  py::class_<EdgeAttributes>(m, "EdgeAttributes")
      .def(py::init<>())
      .def_readwrite("weighted", &EdgeAttributes::weighted)
      .def_readwrite("weight", &EdgeAttributes::weight)
      .def_readwrite("_metadata", &EdgeAttributes::metadata);
}

}  // namespace spark_dsg::python
