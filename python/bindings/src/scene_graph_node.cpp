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
#include "spark_dsg/python/scene_graph_node.h"

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <spark_dsg/scene_graph_node.h>

namespace py = pybind11;
using namespace py::literals;

namespace spark_dsg::python::scene_graph_node {

void addBindings(pybind11::module_& module) {
  py::class_<SceneGraphNode>(module, "SceneGraphNode")
      .def("has_parent", &SceneGraphNode::hasParent)
      .def("has_siblings", &SceneGraphNode::hasSiblings)
      .def("has_children", &SceneGraphNode::hasChildren)
      .def("get_parent", &SceneGraphNode::getParent)
      .def("siblings", &SceneGraphNode::siblings)
      .def("children", &SceneGraphNode::children)
      .def_property_readonly("timestamp",
                             [](const SceneGraphNode& node) -> std::optional<uint64_t> {
                               if (node.timestamp) {
                                 return node.timestamp.value().count();
                               } else {
                                 return std::nullopt;
                               }
                             })
      .def_property("attributes",
                    &SceneGraphNode::getAttributesPtr,
                    &SceneGraphNode::getAttributesPtr,
                    py::return_value_policy::reference_internal)
      .def_property_readonly(
          "id", [](const SceneGraphNode& node) { return NodeSymbol(node.id); })
      .def_readonly("layer", &SceneGraphNode::layer)
      .def("__repr__", [](const SceneGraphNode& node) {
        std::stringstream ss;
        ss << node;
        return ss.str();
      });
}

}  // namespace spark_dsg::python::scene_graph_node
