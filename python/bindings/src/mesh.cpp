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

#include "spark_dsg/python/mesh.h"

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/mesh.h>

#include <filesystem>
#include <string>

namespace spark_dsg::python {

namespace py = pybind11;

using namespace py::literals;

Eigen::MatrixXd getEigenVertices(const Mesh& mesh) {
  const auto num_vertices = mesh.numVertices();
  Eigen::MatrixXd to_return(6, num_vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    const auto& pos = mesh.pos(i);
    to_return(0, i) = pos.x();
    to_return(1, i) = pos.y();
    to_return(2, i) = pos.z();
    if (i < mesh.colors.size()) {
      const auto c = mesh.color(i);
      to_return(3, i) = c.r / 255.0;
      to_return(4, i) = c.g / 255.0;
      to_return(5, i) = c.b / 255.0;
    } else {
      to_return(3, i) = 0.0;
      to_return(4, i) = 0.0;
      to_return(5, i) = 0.0;
    }
  }
  return to_return;
}

void setEigenVertices(Mesh& mesh, const Eigen::MatrixXd& points) {
  if (points.rows() != 6) {
    std::stringstream ss;
    ss << "point rows do not match expected: " << points.rows() << " != 6";
    throw std::invalid_argument(ss.str());
  }

  mesh.resizeVertices(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    Eigen::Vector3f pos = points.col(i).head<3>().cast<float>();
    mesh.setPos(i, pos);
    if (mesh.has_colors) {
      Color color{static_cast<uint8_t>(points(3, i) * 255),
                  static_cast<uint8_t>(points(4, i) * 255),
                  static_cast<uint8_t>(points(5, i) * 255),
                  255};
      mesh.setColor(i, color);
    }
  }
}

Eigen::MatrixXi getEigenFaces(const Mesh& mesh) {
  const auto num_faces = mesh.numFaces();
  Eigen::MatrixXi to_return(3, num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    const auto& face = mesh.face(i);
    to_return(0, i) = face[0];
    to_return(1, i) = face[1];
    to_return(2, i) = face[2];
  }
  return to_return;
}

void setEigenFaces(Mesh& mesh, const Eigen::MatrixXi& indices) {
  if (indices.rows() != 3) {
    std::stringstream ss;
    ss << "index rows do not match expected: " << indices.rows() << " != 3";
    throw std::invalid_argument(ss.str());
  }

  mesh.resizeFaces(indices.cols());
  for (int i = 0; i < indices.cols(); ++i) {
    Mesh::Face face{
        {static_cast<size_t>(indices(0, i)), static_cast<size_t>(indices(1, i)), static_cast<size_t>(indices(2, i))}};
    mesh.face(i) = face;
  }
}

void eraseEigenVertices(Mesh& mesh, const Eigen::VectorXi& indices) {
  std::unordered_set<size_t> to_erase;
  for (int i = 0; i < indices.size(); ++i) {
    to_erase.insert(static_cast<size_t>(indices(i)));
  }
  mesh.eraseVertices(to_erase);
}

void eraseEigenFaces(Mesh& mesh, const Eigen::VectorXi& indices, bool update_vertices) {
  std::unordered_set<size_t> to_erase;
  for (int i = 0; i < indices.size(); ++i) {
    to_erase.insert(static_cast<size_t>(indices(i)));
  }
  mesh.eraseFaces(to_erase, update_vertices);
}

void init_mesh(py::module_& m) {
  py::class_<Mesh, std::shared_ptr<Mesh>>(m, "Mesh")
      .def(py::init<bool, bool, bool, bool>(),
           "has_colors"_a = true,
           "has_timestamps"_a = true,
           "has_labels"_a = true,
           "has_first_seen_stamps"_a = true)
      .def("empty", &Mesh::empty)
      .def("clear", &Mesh::clear)
      .def("num_vertices", &Mesh::numVertices)
      .def("num_faces", &Mesh::numFaces)
      .def("reserve_vertices", &Mesh::reserveVertices)
      .def("resize_vertices", &Mesh::resizeVertices)
      .def("resize_faces", &Mesh::resizeFaces)
      .def("clone", &Mesh::clone)
      .def("pos", &Mesh::pos)
      .def("set_pos", &Mesh::setPos)
      .def("color", &Mesh::color)
      .def("set_color", &Mesh::setColor)
      .def("timestamp", &Mesh::timestamp)
      .def("set_timestamp", &Mesh::setTimestamp)
      .def("first_seen_timestamp", &Mesh::firstSeenTimestamp)
      .def("set_first_seen_timestamp", &Mesh::setFirstSeenTimestamp)
      .def("label", &Mesh::label)
      .def("set_label", &Mesh::setLabel)
      .def("face", py::overload_cast<size_t>(&Mesh::face, py::const_))
      .def("set_face", [](Mesh& mesh, size_t index, const Mesh::Face& face) { mesh.face(index) = face; })
      .def("to_json", &Mesh::serializeToJson)
      .def_static("from_json", &Mesh::deserializeFromJson)
      .def("to_binary",
           [](const Mesh& mesh) {
             std::vector<uint8_t> buffer;
             mesh.serializeToBinary(buffer);
             return py::bytes(reinterpret_cast<char*>(buffer.data()), buffer.size());
           })
      .def_static("from_binary",
                  [](const py::bytes& contents) {
                    const auto view = static_cast<std::string_view>(contents);
                    return Mesh::deserializeFromBinary(reinterpret_cast<const uint8_t*>(view.data()), view.size());
                  })
      .def("save", &Mesh::save)
      .def("save", [](const Mesh& mesh, const std::string& path) { mesh.save(path); })
      .def_static("load", &Mesh::load)
      .def_static("load", [](const std::string& path) { return Mesh::load(path); })
      .def("get_vertices", [](const Mesh& mesh) { return spark_dsg::python::getEigenVertices(mesh); })
      .def("get_faces", [](const Mesh& mesh) { return spark_dsg::python::getEigenFaces(mesh); })
      .def("get_labels", [](const Mesh& mesh) { return mesh.labels; })
      .def("set_vertices",
           [](Mesh& mesh, const Eigen::MatrixXd& points) { spark_dsg::python::setEigenVertices(mesh, points); })
      .def("set_faces", [](Mesh& mesh, const Eigen::MatrixXi& faces) { spark_dsg::python::setEigenFaces(mesh, faces); })
      .def("erase_vertices", &Mesh::eraseVertices, "indices"_a)
      .def("erase_faces", &Mesh::eraseFaces, "indices"_a, "update_vertices"_a = true)
      .def(
          "transform",
          [](Mesh& mesh, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
            const Eigen::Isometry3d transform = Eigen::Translation3d(translation) * Eigen::Quaterniond(rotation);
            mesh.transform(transform.cast<float>());
          },
          "rotation"_a = Eigen::Matrix3d::Identity(),
          "translation"_a = Eigen::Vector3d::Zero())
      .def("append", &Mesh::append)
      .def("total_bytes", &Mesh::totalBytes)
      .def(py::self += py::self);
}

}  // namespace spark_dsg::python
