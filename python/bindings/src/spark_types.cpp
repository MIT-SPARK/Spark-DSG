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
#include "spark_dsg/python/spark_types.h"

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>
#include <spark_dsg/scene_graph_types.h>
#include <spark_dsg/serialization/versioning.h>

#include "spark_dsg/python/python_types.h"

namespace spark_dsg::python {

namespace py = pybind11;
using namespace py::literals;

using namespace spark_dsg;

void init_spark_types(py::module_& m) {
  py::class_<DsgLayers>(m, "DsgLayers")
      .def_readonly_static("SEGMENTS", &DsgLayers::SEGMENTS)
      .def_readonly_static("OBJECTS", &DsgLayers::OBJECTS)
      .def_readonly_static("AGENTS", &DsgLayers::AGENTS)
      .def_readonly_static("PLACES", &DsgLayers::PLACES)
      .def_readonly_static("MESH_PLACES", &DsgLayers::MESH_PLACES)
      .def_readonly_static("TRAVERSABILITY", &DsgLayers::TRAVERSABILITY)
      .def_readonly_static("ROOMS", &DsgLayers::ROOMS)
      .def_readonly_static("BUILDINGS", &DsgLayers::BUILDINGS)
      .def_static(
          "name_to_layer_id",
          [](const std::string& name) -> std::optional<LayerKey> { return DsgLayers::nameToLayerId(name); },
          "name"_a);

  py::class_<LayerKey>(m, "LayerKey")
      .def(py::init<LayerId>())
      .def(py::init<LayerId, PythonPartitionId>())
      .def_readwrite("layer", &LayerKey::layer)
      .def_readwrite("partition", &LayerKey::partition)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__lt__",
           [](const LayerKey& lhs, const LayerKey& rhs) {
             // note that this is a partial ordering; all partitions are equal
             return lhs.layer < rhs.layer;
           })
      .def("__gt__",
           [](const LayerKey& lhs, const LayerKey& rhs) {
             // note that this is a partial ordering; all partitions are equal
             return lhs.layer > rhs.layer;
           })
      .def("__repr__", [](const LayerKey& key) {
        std::stringstream ss;
        ss << key;
        return ss.str();
      });

  py::implicitly_convertible<LayerId, LayerKey>();

  py::class_<NodeSymbol>(m, "NodeSymbol")
      .def(py::init([](char key, size_t index) { return NodeSymbol(key, index); }))
      .def(py::init([](size_t value) { return NodeSymbol(value); }))
      .def_property("category_id", &NodeSymbol::categoryId, nullptr)
      .def_property("category", &NodeSymbol::category, nullptr)
      .def_property(
          "value", [](const NodeSymbol& symbol) { return static_cast<NodeId>(symbol); }, nullptr)
      .def("__repr__", [](const NodeSymbol& ns) { return ns.str(false); })
      .def("__hash__", [](const NodeSymbol& symbol) { return static_cast<NodeId>(symbol); })
      .def("str", &NodeSymbol::str, "literal"_a = true)
      .def(pybind11::self == pybind11::self)
      .def(pybind11::self != pybind11::self);

  py::implicitly_convertible<NodeId, NodeSymbol>();
}

}  // namespace spark_dsg::python
