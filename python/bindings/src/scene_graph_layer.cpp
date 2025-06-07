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
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/scene_graph_layer.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include "spark_dsg/python/python_layer_view.h"
#include "spark_dsg/python/scene_graph_iterators.h"

namespace spark_dsg::python {

namespace py = pybind11;
using namespace py::literals;

void init_scene_graph_layer(py::module_& m) {
  py::class_<SceneGraphLayer, std::shared_ptr<SceneGraphLayer>>(m, "SceneGraphLayer")
      .def(py::init<LayerId>())
      .def(py::init<const std::string&>())
      .def("add_node",
           [](SceneGraphLayer& layer, NodeSymbol node, const NodeAttributes& attrs) {
             layer.emplaceNode(node, attrs.clone());
           })
      .def(
          "insert_edge",
          [](SceneGraphLayer& layer, NodeSymbol source, NodeSymbol target) { return layer.insertEdge(source, target); })
      .def("insert_edge",
           [](SceneGraphLayer& layer, NodeSymbol source, NodeSymbol target, const EdgeAttributes& info) {
             return layer.insertEdge(source, target, info.clone());
           })
      .def("has_node", &SceneGraphLayer::hasNode)
      .def("has_edge", &SceneGraphLayer::hasEdge)
      .def("get_node", &SceneGraphLayer::getNode, py::return_value_policy::reference_internal)
      .def("find_node", &SceneGraphLayer::findNode, py::return_value_policy::reference_internal)
      .def("get_edge", &SceneGraphLayer::getEdge, py::return_value_policy::reference_internal)
      .def("find_edge", &SceneGraphLayer::findEdge, py::return_value_policy::reference_internal)
      .def("remove_edge", &SceneGraphLayer::removeEdge)
      .def("num_nodes", &SceneGraphLayer::numNodes)
      .def("num_edges", &SceneGraphLayer::numEdges)
      .def("get_position",
           [](const SceneGraphLayer& layer, NodeSymbol node) { return layer.getNode(node).attributes().position; })
      .def_readonly("id", &SceneGraphLayer::id)
      .def_property(
          "nodes",
          [](const SceneGraphLayer& view) { return py::make_iterator(NodeIter(view.nodes()), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const SceneGraphLayer& view) { return py::make_iterator(EdgeIter(view.edges()), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def("to_binary",
           [](const SceneGraphLayer& layer) -> py::bytes {
             std::vector<uint8_t> buffer;
             io::binary::writeLayer(layer, buffer);
             return py::bytes(reinterpret_cast<char*>(buffer.data()), buffer.size());
           })
      .def_static("from_binary", [](const py::bytes& contents) {
        const auto view = static_cast<std::string_view>(contents);
        return io::binary::readLayer(reinterpret_cast<const uint8_t*>(view.data()), view.size());
      });

  py::class_<LayerView>(m, "LayerView")
      .def("has_node", &LayerView::hasNode)
      .def("has_edge", &LayerView::hasEdge)
      .def("get_node", &LayerView::getNode, py::return_value_policy::reference_internal)
      .def("get_edge", &LayerView::getEdge, py::return_value_policy::reference_internal)
      .def("num_nodes", &LayerView::numNodes)
      .def("num_edges", &LayerView::numEdges)
      .def("get_position", &LayerView::getPosition)
      .def_property(
          "key", [](const LayerView& view) { return view.id; }, nullptr)
      .def_property(
          "id", [](const LayerView& view) { return view.id.layer; }, nullptr)
      .def_property(
          "partition", [](const LayerView& view) { return view.id.partition; }, nullptr)
      .def_property(
          "nodes",
          [](const LayerView& view) { return py::make_iterator(view.nodes(), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const LayerView& view) { return py::make_iterator(view.edges(), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal);
}

}  // namespace spark_dsg::python
