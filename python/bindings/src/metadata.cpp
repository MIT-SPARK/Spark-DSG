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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/bounding_box.h>
#include <spark_dsg/labelspace.h>
#include <spark_dsg/mesh.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph_layer.h>
#include <spark_dsg/scene_graph_node.h>
#include <spark_dsg/scene_graph_types.h>
#include <spark_dsg/scene_graph_utilities.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>
#include <spark_dsg/serialization/versioning.h>
#include <spark_dsg/zmq_interface.h>

#include <filesystem>
#include <iomanip>
#include <sstream>

#include "spark_dsg/python/spark_types.h"

namespace spark_dsg::python {

namespace py = pybind11;
using namespace py::literals;

using namespace spark_dsg;

void init_metadata(py::module_& m) {
  py::class_<Metadata>(m, "_Metadata")
      .def(py::init<>())
      .def("_get", [](const Metadata& data) { return data().dump(); })
      .def("_set", [](Metadata& data, const std::string& contents) { data.set(nlohmann::json::parse(contents)); })
      .def("_add", [](Metadata& data, const std::string& contents) { data.add(nlohmann::json::parse(contents)); });

  py::class_<Labelspace>(m, "Labelspace")
      .def(py::init<>())
      .def(py::init<const std::map<SemanticLabel, std::string>&>())
      .def("get_label", &Labelspace::getLabel)
      .def("get_category",
           [](const Labelspace& labelspace, SemanticLabel label) { return labelspace.getCategory(label); })
      .def(
          "get_node_category",
          [](const Labelspace& labelspace, const SceneGraphNode& node, const std::string& unknown_name) {
            const auto attrs = node.tryAttributes<SemanticNodeAttributes>();
            return attrs ? labelspace.getCategory(*attrs, unknown_name) : unknown_name;
          },
          "node"_a,
          "unknown_name"_a = "UNKNOWN")
      .def("__bool__", [](const Labelspace& labelspace) { return static_cast<bool>(labelspace); })
      .def_property_readonly("labels_to_names", &Labelspace::labels_to_names)
      .def_property_readonly("names_to_labels", &Labelspace::names_to_labels);
}

}  // namespace spark_dsg::python
