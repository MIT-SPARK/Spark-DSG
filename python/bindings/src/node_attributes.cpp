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
#include "spark_dsg/python/node_attributes.h"

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <spark_dsg/node_attributes.h>

#include <iomanip>

namespace py = pybind11;
using namespace py::literals;

namespace spark_dsg::python::node_attributes {

template <typename Scalar>
struct Quaternion {
  Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

  Quaternion(Scalar w, Scalar x, Scalar y, Scalar z) : w(w), x(x), y(y), z(z) {}

  explicit Quaternion(const Eigen::Quaternion<Scalar> other) {
    w = other.w();
    x = other.x();
    y = other.y();
    z = other.z();
  }

  operator Eigen::Quaternion<Scalar>() const {
    return Eigen::Quaternion<Scalar>(w, x, y, z);
  }

  Scalar w;
  Scalar x;
  Scalar y;
  Scalar z;
};

template <typename Scalar>
std::ostream& operator<<(std::ostream& out, const Quaternion<Scalar>& q) {
  out << "Quaternion<w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z
      << ">";
  return out;
}

void addBindings(pybind11::module_& module) {
  py::class_<Quaternion<float>>(module, "Quaternionf")
      .def(py::init<>())
      .def(py::init<float, float, float, float>())
      .def_readwrite("w", &Quaternion<float>::w)
      .def_readwrite("x", &Quaternion<float>::x)
      .def_readwrite("y", &Quaternion<float>::y)
      .def_readwrite("z", &Quaternion<float>::z)
      .def("__repr__", [](const Quaternion<float>& q) {
        std::stringstream ss;
        ss << q;
        return ss.str();
      });

  py::class_<Quaternion<double>>(module, "Quaterniond")
      .def(py::init<>())
      .def(py::init<double, double, double, double>())
      .def_readwrite("w", &Quaternion<double>::w)
      .def_readwrite("x", &Quaternion<double>::x)
      .def_readwrite("y", &Quaternion<double>::y)
      .def_readwrite("z", &Quaternion<double>::z)
      .def("__repr__", [](const Quaternion<double>& q) {
        std::stringstream ss;
        ss << q;
        return ss.str();
      });

  py::class_<NodeAttributes>(module, "NodeAttributes")
      .def(py::init<>())
      .def_readwrite("position", &NodeAttributes::position)
      .def_readwrite("last_update_time_ns", &NodeAttributes::last_update_time_ns)
      .def_readwrite("is_active", &NodeAttributes::is_active)
      .def_readwrite("is_predicted", &NodeAttributes::is_predicted)
      .def("__repr__", [](const NodeAttributes& attrs) {
        std::stringstream ss;
        ss << attrs;
        return ss.str();
      });

  py::class_<SemanticNodeAttributes, NodeAttributes>(module, "SemanticNodeAttributes")
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
      .def_readwrite("semantic_class_probabilities",
                     &RoomNodeAttributes::semantic_class_probabilities)
      .def_readonly_static("NO_SEMANTIC_LABEL",
                           &SemanticNodeAttributes::NO_SEMANTIC_LABEL);

  py::class_<ObjectNodeAttributes, SemanticNodeAttributes>(module,
                                                           "ObjectNodeAttributes")
      .def(py::init<>())
      .def_readwrite("registered", &ObjectNodeAttributes::registered)
      .def_property(
          "world_R_object",
          [](const ObjectNodeAttributes& attrs) {
            return Quaternion<double>(attrs.world_R_object);
          },
          [](ObjectNodeAttributes& attrs, const Quaternion<double>& rot) {
            attrs.world_R_object = rot;
          });

  py::class_<RoomNodeAttributes, SemanticNodeAttributes>(module, "RoomNodeAttributes")
      .def(py::init<>());

  py::class_<NearestVertexInfo>(module, "NearestVertexInfo")
      .def(py::init<>())
      .def_property(
          "block",
          [](const NearestVertexInfo& info) -> std::array<int32_t, 3> {
            return {info.block[0], info.block[1], info.block[2]};
          },
          [](NearestVertexInfo& info, const std::array<int32_t, 3>& array) {
            info.block[0] = array[0];
            info.block[1] = array[1];
            info.block[2] = array[2];
          })
      .def_property(
          "voxel_pos",
          [](const NearestVertexInfo& info) -> std::array<double, 3> {
            return {info.voxel_pos[0], info.voxel_pos[1], info.voxel_pos[2]};
          },
          [](NearestVertexInfo& info, const std::array<double, 3>& array) {
            info.voxel_pos[0] = array[0];
            info.voxel_pos[1] = array[1];
            info.voxel_pos[2] = array[2];
          })
      .def_readwrite("vertex", &NearestVertexInfo::vertex)
      .def_readwrite("label", &NearestVertexInfo::label)
      .def("__repr__", [](const NearestVertexInfo& info) {
        std::stringstream ss;
        ss << std::setprecision(6) << "VertexInfo<block=[" << info.block[0] << ", "
           << info.block[1] << ", " << info.block[2] << "], pos=[" << info.voxel_pos[0]
           << ", " << info.voxel_pos[1] << ", " << info.voxel_pos[2] << "], label=";
        if (info.label) {
          ss << info.label.value();
        } else {
          ss << "n/a";
        }
        ss << ">";
        return ss.str();
      });

  py::class_<PlaceNodeAttributes, SemanticNodeAttributes>(module, "PlaceNodeAttributes")
      .def(py::init<>())
      .def_readwrite("distance", &PlaceNodeAttributes::distance)
      .def_readwrite("num_basis_points", &PlaceNodeAttributes::num_basis_points)
      .def_readwrite("voxblox_mesh_connections",
                     &PlaceNodeAttributes::voxblox_mesh_connections)
      .def_readwrite("pcl_mesh_connections", &PlaceNodeAttributes::pcl_mesh_connections)
      .def_readwrite("mesh_vertex_labels", &PlaceNodeAttributes::mesh_vertex_labels)
      .def_readwrite("deformation_connections",
                     &PlaceNodeAttributes::deformation_connections)
      .def_readwrite("real_place", &PlaceNodeAttributes::real_place)
      .def_readwrite("active_frontier", &PlaceNodeAttributes::active_frontier)
      .def_readwrite("frontier_scale", &PlaceNodeAttributes::frontier_scale)
      .def_property(
          "orientation",
          [](const PlaceNodeAttributes& attrs) {
            return Quaternion<double>(attrs.orientation);
          },
          [](PlaceNodeAttributes& attrs, const Quaternion<double>& rot) {
            attrs.orientation = rot;
          })
      .def_readwrite("num_frontier_voxels", &PlaceNodeAttributes::num_frontier_voxels)
      .def_readwrite("need_cleanup", &PlaceNodeAttributes::need_cleanup);

  py::class_<Place2dNodeAttributes, SemanticNodeAttributes>(module,
                                                            "Place2dNodeAttributes")
      .def(py::init<>())
      .def_readwrite("boundary", &Place2dNodeAttributes::boundary)
      .def_readwrite("ellipse_matrix_compress",
                     &Place2dNodeAttributes::ellipse_matrix_compress)
      .def_readwrite("ellipse_matrix_expand",
                     &Place2dNodeAttributes::ellipse_matrix_expand)
      .def_readwrite("ellipse_centroid", &Place2dNodeAttributes::ellipse_centroid)
      .def_readwrite("pcl_boundary_connections",
                     &Place2dNodeAttributes::pcl_boundary_connections)
      .def_readwrite("voxblox_mesh_connections",
                     &Place2dNodeAttributes::voxblox_mesh_connections)
      .def_readwrite("pcl_mesh_connections",
                     &Place2dNodeAttributes::pcl_mesh_connections)
      .def_readwrite("mesh_vertex_labels", &Place2dNodeAttributes::mesh_vertex_labels)
      .def_readwrite("deformation_connections",
                     &Place2dNodeAttributes::deformation_connections);

  py::class_<AgentNodeAttributes, NodeAttributes>(module, "AgentNodeAttributes")
      .def(py::init<>())
      .def_property(
          "world_R_body",
          [](const AgentNodeAttributes& attrs) {
            return Quaternion<double>(attrs.world_R_body);
          },
          [](AgentNodeAttributes& attrs, const Quaternion<double>& rot) {
            attrs.world_R_body = rot;
          })
      .def_readwrite("external_key", &AgentNodeAttributes::external_key)
      .def_readwrite("dbow_ids", &AgentNodeAttributes::dbow_ids)
      .def_readwrite("dbow_values", &AgentNodeAttributes::dbow_values);
}

}  // namespace spark_dsg::python::node_attributes
