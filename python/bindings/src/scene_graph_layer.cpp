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
#include "spark_dsg/python/scene_graph_layer.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <spark_dsg/scene_graph_layer.h>

#include "spark_dsg/python/scene_graph_iterators.h"

namespace py = pybind11;
using namespace py::literals;

namespace spark_dsg::python::scene_graph_layer {

void addBindings(pybind11::module_& module) {
  // TODO(nathan) iterator over nodes and edges
  py::class_<IsolatedSceneGraphLayer, std::shared_ptr<IsolatedSceneGraphLayer>>(
      module, "SceneGraphLayer")
      .def(py::init<LayerId>())
      .def(
          "add_node",
          [](IsolatedSceneGraphLayer& layer, NodeId node, const NodeAttributes& attrs) {
            layer.emplaceNode(node, attrs.clone());
          })
      .def("insert_edge",
           [](IsolatedSceneGraphLayer& layer, NodeId source, NodeId target) {
             return layer.insertEdge(source, target);
           })
      .def("insert_edge",
           [](IsolatedSceneGraphLayer& layer,
              NodeId source,
              NodeId target,
              const EdgeAttributes& info) {
             return layer.insertEdge(source, target, info.clone());
           })
      .def("has_node", &IsolatedSceneGraphLayer::hasNode)
      .def("has_edge", &IsolatedSceneGraphLayer::hasEdge)
      .def("get_node",
           &IsolatedSceneGraphLayer::getNode,
           py::return_value_policy::reference_internal)
      .def("find_node",
           &IsolatedSceneGraphLayer::findNode,
           py::return_value_policy::reference_internal)
      .def("get_edge",
           &IsolatedSceneGraphLayer::getEdge,
           py::return_value_policy::reference_internal)
      .def("find_edge",
           &IsolatedSceneGraphLayer::findEdge,
           py::return_value_policy::reference_internal)
      .def("remove_edge", &IsolatedSceneGraphLayer::removeEdge)
      .def("num_nodes", &IsolatedSceneGraphLayer::numNodes)
      .def("num_edges", &IsolatedSceneGraphLayer::numEdges)
      .def("get_position", &IsolatedSceneGraphLayer::getPosition)
      .def_readonly("id", &IsolatedSceneGraphLayer::id)
      .def_property(
          "nodes",
          [](const IsolatedSceneGraphLayer& view) {
            return py::make_iterator(NodeIter(view.nodes()), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const IsolatedSceneGraphLayer& view) {
            return py::make_iterator(EdgeIter(view.edges()), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal);
}

}  // namespace spark_dsg::python::scene_graph_layer
