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
#include "spark_dsg/python/scene_graph.h"

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/labelspace.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_utilities.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <filesystem>
#include <iomanip>
#include <sstream>

#include "spark_dsg/python/python_layer_view.h"
#include "spark_dsg/python/python_types.h"
#include "spark_dsg/python/scene_graph_iterators.h"

namespace spark_dsg::python {

namespace py = pybind11;
using namespace py::literals;

void init_scene_graph(py::module_& m) {
  m.def("compute_ancestor_bounding_box",
        &computeAncestorBoundingBox,
        "G"_a,
        "node_id"_a,
        "depth"_a = 1,
        "bbox_type"_a = BoundingBox::Type::AABB);

  py::class_<DynamicSceneGraph, std::shared_ptr<DynamicSceneGraph>>(m, "DynamicSceneGraph", py::dynamic_attr())
      .def(py::init<bool>(), "empty"_a = false)
      .def(py::init<const DynamicSceneGraph::LayerKeys&, const DynamicSceneGraph::LayerNames&>(),
           "layer_keys"_a,
           "layer_names"_a = DynamicSceneGraph::LayerNames{})
      .def("clear", &DynamicSceneGraph::clear, "include_mesh"_a = true)
      .def("reset", &DynamicSceneGraph::reset)
      .def(
          "has_layer",
          [](const DynamicSceneGraph& graph, LayerId layer, PythonPartitionId partition) {
            return graph.hasLayer(layer, partition);
          },
          "layer"_a,
          "partition"_a = 0)
      .def(
          "has_layer",
          [](const DynamicSceneGraph& graph, const std::string& name) { return graph.hasLayer(name); },
          "layer"_a)
      .def(
          "get_layer",
          [](const DynamicSceneGraph& graph, LayerId layer, PythonPartitionId partition) {
            return LayerView(graph.getLayer(layer, partition));
          },
          "layer"_a,
          "partition"_a = 0,
          py::return_value_policy::reference_internal)
      .def(
          "get_layer",
          [](const DynamicSceneGraph& graph, const std::string& layer) { return LayerView(graph.getLayer(layer)); },
          "layer"_a,
          py::return_value_policy::reference_internal)
      .def(
          "add_layer",
          [](DynamicSceneGraph& graph, LayerId layer, PythonPartitionId partition, const std::string& name) {
            return LayerView(graph.addLayer(layer, partition, name));
          },
          "layer"_a,
          "partition"_a = 0,
          "name"_a = "",
          py::return_value_policy::reference_internal)
      .def(
          "remove_layer",
          [](DynamicSceneGraph& graph, LayerId layer, PythonPartitionId partition) {
            graph.removeLayer(layer, partition);
          },
          "layer"_a,
          "partition"_a = 0)
      .def(
          "add_node",
          [](DynamicSceneGraph& graph, LayerKey key, NodeSymbol node_id, const NodeAttributes& attrs) {
            return graph.emplaceNode(key, node_id, attrs.clone());
          },
          "layer"_a,
          "node_id"_a,
          "attrs"_a)
      .def(
          "add_node",
          [](DynamicSceneGraph& graph,
             LayerId layer,
             NodeSymbol node_id,
             const NodeAttributes& attrs,
             PythonPartitionId partition) { return graph.emplaceNode(layer, node_id, attrs.clone(), partition); },
          "layer"_a,
          "node_id"_a,
          "attrs"_a,
          "partition"_a = 0)
      .def(
          "add_node",
          [](DynamicSceneGraph& graph, const std::string& layer, NodeSymbol node_id, const NodeAttributes& attrs) {
            return graph.emplaceNode(layer, node_id, attrs.clone());
          },
          "layer"_a,
          "node_id"_a,
          "attrs"_a)
      .def(
          "insert_edge",
          [](DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target, bool enforce_single_parent) {
            return graph.insertEdge(source, target, nullptr, enforce_single_parent);
          },
          "source"_a,
          "target"_a,
          "enforce_single_parent"_a = false)
      .def(
          "insert_edge",
          [](DynamicSceneGraph& graph,
             NodeSymbol source,
             NodeSymbol target,
             const EdgeAttributes& info,
             bool enforce_single_parent) {
            return graph.insertEdge(source, target, info.clone(), enforce_single_parent);
          },
          "source"_a,
          "target"_a,
          "info"_a,
          "enforce_single_parent"_a = false)
      .def("has_node", [](const DynamicSceneGraph& graph, NodeSymbol node_id) { return graph.hasNode(node_id); })
      .def("has_edge",
           [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) {
             return graph.hasEdge(source, target);
           })
      .def("has_mesh", &DynamicSceneGraph::hasMesh)
      .def(
          "get_node",
          [](const DynamicSceneGraph& graph, NodeSymbol node) -> const SceneGraphNode& { return graph.getNode(node); },
          py::return_value_policy::reference_internal)
      .def(
          "find_node",
          [](const DynamicSceneGraph& graph, NodeSymbol node) -> const SceneGraphNode* { return graph.findNode(node); },
          py::return_value_policy::reference_internal)
      .def(
          "get_edge",
          [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) -> const SceneGraphEdge& {
            return graph.getEdge(source, target);
          },
          py::return_value_policy::reference_internal)
      .def(
          "find_edge",
          [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) -> const SceneGraphEdge* {
            return graph.findEdge(source, target);
          },
          py::return_value_policy::reference_internal)
      .def("remove_node", [](DynamicSceneGraph& graph, NodeSymbol node) -> bool { return graph.removeNode(node); })
      .def("remove_edge",
           [](DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) -> bool {
             return graph.removeEdge(source, target);
           })
      .def("num_layers", &DynamicSceneGraph::numLayers)
      .def(
          "num_nodes",
          [](const DynamicSceneGraph& graph, bool include_partitions) {
            return include_partitions ? graph.numNodes() : graph.numUnpartitionedNodes();
          },
          "include_partitions"_a = true)
      .def(
          "num_edges",
          [](const DynamicSceneGraph& graph, bool include_partitions) {
            return include_partitions ? graph.numEdges() : graph.numUnpartitionedEdges();
          },
          "include_partitions"_a = true)
      .def("empty", &DynamicSceneGraph::empty)
      .def("get_position", &DynamicSceneGraph::getPosition)
      .def(
          "save",
          [](const DynamicSceneGraph& graph, const std::string& filepath, bool include_mesh) {
            graph.save(filepath, include_mesh);
          },
          "filepath"_a,
          "include_mesh"_a = true)
      .def(
          "save",
          [](const DynamicSceneGraph& graph, const std::filesystem::path& filepath, bool include_mesh) {
            graph.save(filepath, include_mesh);
          },
          "filepath"_a,
          "include_mesh"_a = true)
      .def_static("load", &DynamicSceneGraph::load)
      .def_static("load", [](const std::string& filepath) { return DynamicSceneGraph::load(filepath); })
      .def_readwrite("_metadata", &DynamicSceneGraph::metadata)
      .def_property_readonly("layer_ids", &DynamicSceneGraph::layer_ids)
      .def_property_readonly("layer_keys", &DynamicSceneGraph::layer_keys)
      .def_property_readonly("layer_names", &DynamicSceneGraph::layer_names)
      .def_property_readonly("node_lookup", &DynamicSceneGraph::node_lookup)
      .def_property(
          "layers",
          [](const DynamicSceneGraph& graph) { return py::make_iterator(LayerIter(graph.layers()), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "layer_partitions",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(PartitionIter(graph.layer_partitions()), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "nodes",
          [](const DynamicSceneGraph& graph) { return py::make_iterator(GlobalNodeIter(graph), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "edges",
          [](const DynamicSceneGraph& graph) { return py::make_iterator(GlobalEdgeIter(graph), IterSentinel()); },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "unpartitioned_nodes",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(GlobalNodeIter(graph, false), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "unpartitioned_edges",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(GlobalEdgeIter(graph, false), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "interlayer_edges",
          [](const DynamicSceneGraph& graph) {
            return py::make_iterator(EdgeIter(graph.interlayer_edges()), IterSentinel());
          },
          nullptr,
          py::return_value_policy::reference_internal)
      .def_property(
          "mesh",
          [](const DynamicSceneGraph& graph) { return graph.mesh(); },
          [](DynamicSceneGraph& graph, const Mesh::Ptr& mesh) { graph.setMesh(mesh); })
      .def("get_layer_key", &DynamicSceneGraph::getLayerKey, "name"_a)
      .def("clone", &DynamicSceneGraph::clone)
      .def("transform",
           [](DynamicSceneGraph& G, const Eigen::Matrix4d& mat) {
             Eigen::Isometry3d iso(mat);
             G.transform(iso);
           })
      .def("__deepcopy__", [](const DynamicSceneGraph& G, py::object) { return G.clone(); })
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
            const auto view = static_cast<std::string_view>(contents);
            return io::binary::updateGraph(graph, reinterpret_cast<const uint8_t*>(view.data()), view.size());
          },
          "contents"_a)
      .def_static("from_binary",
                  [](const py::bytes& contents) {
                    const auto view = static_cast<std::string_view>(contents);
                    return io::binary::readGraph(reinterpret_cast<const uint8_t*>(view.data()), view.size());
                  })
      .def(
          "get_labelspace",
          [](const DynamicSceneGraph& graph, LayerId layer, PartitionId partition) {
            return Labelspace::fromMetadata(graph, layer, partition);
          },
          "layer"_a,
          "partition"_a = 0)
      .def(
          "get_labelspace",
          [](const DynamicSceneGraph& graph, const std::string& name) { return Labelspace::fromMetadata(graph, name); },
          "name"_a)
      .def(
          "set_labelspace",
          [](DynamicSceneGraph& graph, const Labelspace& labelspace, LayerId layer, PartitionId partition) {
            labelspace.save(graph, layer, partition);
          },
          "labelspace"_a,
          "layer"_a,
          "partition"_a = 0)
      .def(
          "set_labelspace",
          [](DynamicSceneGraph& graph, const Labelspace& labelspace, const std::string& name) {
            labelspace.save(graph, name);
          },
          "labelspace"_a,
          "name"_a);
}

}  // namespace spark_dsg::python
