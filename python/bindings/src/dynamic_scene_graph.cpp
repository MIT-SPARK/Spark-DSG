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
#include "spark_dsg/python/dynamic_scene_graph.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include "spark_dsg/python/scene_graph_iterators.h"

namespace py = pybind11;
using namespace py::literals;

namespace spark_dsg::python::dynamic_scene_graph {

void addBindings(pybind11::module_& module) {
  py::class_<LayerView>(module, "LayerView")
      .def("has_node", &LayerView::hasNode)
      .def("has_edge", &LayerView::hasEdge)
      .def("get_node", &LayerView::getNode, py::return_value_policy::reference_internal)
      .def("get_edge", &LayerView::getEdge, py::return_value_policy::reference_internal)
      .def("num_nodes", &LayerView::numNodes)
      .def("num_edges", &LayerView::numEdges)
      .def("get_position", &LayerView::getPosition)
      .def_readonly("id", &LayerView::id)
      .def_property(
          "nodes",
          [](const LayerView& view) {
            return py::make_iterator(view.nodes(), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const LayerView& view) {
            return py::make_iterator(view.edges(), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal);

  py::class_<DynamicLayerView>(module, "DynamicLayerView")
      .def_readonly("id", &DynamicLayerView::id)
      .def_readonly("prefix", &DynamicLayerView::prefix)
      .def("num_nodes", &DynamicLayerView::numNodes)
      .def("num_edges", &DynamicLayerView::numEdges)
      .def_property(
          "nodes",
          [](const DynamicLayerView& view) {
            return py::make_iterator(view.nodes(), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const DynamicLayerView& view) {
            return py::make_iterator(view.edges(), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal);

  py::class_<Color>(module, "Color")
      .def_readwrite("r", &Color::r)
      .def_readwrite("g", &Color::g)
      .def_readwrite("b", &Color::b)
      .def_readwrite("a", &Color::a);

  py::class_<DynamicSceneGraph, std::shared_ptr<DynamicSceneGraph>>(
      module, "DynamicSceneGraph", py::dynamic_attr())
      .def(py::init<>())
      .def(py::init<const DynamicSceneGraph::LayerIds&>())
      .def("clear", &DynamicSceneGraph::clear)
      .def("create_dynamic_layer", &DynamicSceneGraph::createDynamicLayer)
      .def("add_node",
           [](DynamicSceneGraph& graph,
              LayerId layer_id,
              NodeId node_id,
              const NodeAttributes& attrs) {
             graph.emplaceNode(layer_id, node_id, attrs.clone());
           })
      .def(
          "add_node",
          [](DynamicSceneGraph& graph,
             LayerId layer_id,
             LayerPrefix prefix,
             std::chrono::nanoseconds timestamp,
             const NodeAttributes& attrs,
             bool add_edge_to_previous) {
            graph.emplaceNode(
                layer_id, prefix, timestamp, attrs.clone(), add_edge_to_previous);
          },
          "layer_id"_a,
          "prefix"_a,
          "timestamp"_a,
          "attrs"_a,
          "add_edge_to_previous"_a = true)
      .def(
          "insert_edge",
          [](DynamicSceneGraph& graph,
             NodeId source,
             NodeId target,
             bool enforce_single_parent) {
            return enforce_single_parent ? graph.insertParentEdge(source, target)
                                         : graph.insertEdge(source, target);
          },
          "source"_a,
          "target"_a,
          "enforce_single_parent"_a = false)
      .def(
          "insert_edge",
          [](DynamicSceneGraph& graph,
             NodeId source,
             NodeId target,
             const EdgeAttributes& info,
             bool enforce_single_parent) {
            return enforce_single_parent
                       ? graph.insertParentEdge(source, target, info.clone())
                       : graph.insertEdge(source, target, info.clone());
          },
          "source"_a,
          "target"_a,
          "info"_a,
          "enforce_single_parent"_a = false)
      .def("has_layer",
           static_cast<bool (DynamicSceneGraph::*)(LayerId) const>(
               &DynamicSceneGraph::hasLayer))
      .def("has_layer",
           static_cast<bool (DynamicSceneGraph::*)(LayerId, LayerPrefix) const>(
               &DynamicSceneGraph::hasLayer))
      .def("has_node", &DynamicSceneGraph::hasNode)
      .def("has_edge",
           py::overload_cast<NodeId, NodeId>(&DynamicSceneGraph::hasEdge, py::const_))
      .def("has_mesh", &DynamicSceneGraph::hasMesh)
      .def(
          "get_layer",
          [](const DynamicSceneGraph& graph, LayerId layer_id) {
            if (!graph.hasLayer(layer_id)) {
              throw std::out_of_range("layer doesn't exist");
            }
            return LayerView(graph.getLayer(layer_id));
          },
          py::return_value_policy::reference_internal)
      .def(
          "get_dynamic_layer",
          [](const DynamicSceneGraph& graph, LayerId layer_id, LayerPrefix prefix) {
            if (!graph.hasLayer(layer_id, prefix)) {
              throw std::out_of_range("layer doesn't exist");
            }
            return DynamicLayerView(graph.getLayer(layer_id, prefix));
          },
          py::return_value_policy::reference_internal)
      .def("get_node",
           &DynamicSceneGraph::getNode,
           py::return_value_policy::reference_internal)
      .def("find_node",
           &DynamicSceneGraph::findNode,
           py::return_value_policy::reference_internal)
      .def("get_edge",
           &DynamicSceneGraph::getEdge,
           py::return_value_policy::reference_internal)
      .def("find_edge",
           &DynamicSceneGraph::findEdge,
           py::return_value_policy::reference_internal)
      .def("remove_node", &DynamicSceneGraph::removeNode)
      .def("remove_edge", &DynamicSceneGraph::removeEdge)
      .def("is_dynamic", &DynamicSceneGraph::isDynamic)
      .def("num_layers", &DynamicSceneGraph::numLayers)
      .def("num_dynamic_layers_of_type", &DynamicSceneGraph::numDynamicLayersOfType)
      .def("num_dynamic_layers", &DynamicSceneGraph::numDynamicLayers)
      .def("num_nodes", &DynamicSceneGraph::numNodes, "include_mesh"_a = false)
      .def("num_static_nodes", &DynamicSceneGraph::numStaticNodes)
      .def("num_dynamic_nodes", &DynamicSceneGraph::numDynamicNodes)
      .def("empty", &DynamicSceneGraph::empty)
      .def("num_edges", &DynamicSceneGraph::numEdges)
      .def("num_static_edges", &DynamicSceneGraph::numStaticEdges)
      .def("num_dynamic_edges", &DynamicSceneGraph::numDynamicEdges)
      .def("get_position", &DynamicSceneGraph::getPosition)
      .def(
          "save",
          [](const DynamicSceneGraph& graph,
             const std::string& filepath,
             bool include_mesh) { graph.save(filepath, include_mesh); },
          "filepath"_a,
          "include_mesh"_a = true)
      .def(
          "save",
          [](const DynamicSceneGraph& graph,
             const std::filesystem::path& filepath,
             bool include_mesh) { graph.save(filepath, include_mesh); },
          "filepath"_a,
          "include_mesh"_a = true)
      .def_static("load", &DynamicSceneGraph::load)
      .def_static("load",
                  [](const std::filesystem::path& filepath) {
                    return DynamicSceneGraph::load(filepath);
                  })
      .def_property(
          "layers",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(LayerIter(graph.layers()), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "dynamic_layers",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(DynamicLayerIter(graph.dynamicLayers()),
                                     IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "nodes",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(GlobalNodeIter(graph), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(GlobalEdgeIter(graph), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "interlayer_edges",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(EdgeIter(graph.interlayer_edges()),
                                     IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "dynamic_interlayer_edges",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(EdgeIter(graph.dynamic_interlayer_edges()),
                                     IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "mesh",
          [](const DynamicSceneGraph& graph) { return graph.mesh(); },
          [](DynamicSceneGraph& graph, const Mesh::Ptr& mesh) { graph.setMesh(mesh); })
      .def("clone", &DynamicSceneGraph::clone)
      .def("__deepcopy__",
           [](const DynamicSceneGraph& G, py::object) { return G.clone(); })
      .def(
          "to_binary",
          [](const DynamicSceneGraph& graph, bool include_mesh) {
            std::vector<uint8_t> buffer;
            io::binary::writeGraph(graph, buffer, include_mesh);
            return py::bytes(reinterpret_cast<char*>(buffer.data()), buffer.size());
          },
          "include_mesh"_a = false)
      .def(
          "update_from_binary",
          [](DynamicSceneGraph& graph, const py::bytes& contents) {
            const auto& view = static_cast<const std::string_view&>(contents);
            return io::binary::updateGraph(
                graph, reinterpret_cast<const uint8_t*>(view.data()), view.size());
          },
          "contents"_a)
      .def_static("from_binary", [](const py::bytes& contents) {
        const auto& view = static_cast<const std::string_view&>(contents);
        return io::binary::readGraph(reinterpret_cast<const uint8_t*>(view.data()),
                                     view.size());
      });
}

}  // namespace spark_dsg::python::dynamic_scene_graph
