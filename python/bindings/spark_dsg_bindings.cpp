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
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/graph_binary_serialization.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/scene_graph_utilities.h>

#include <iostream>
#include <sstream>
#include <string>

#include "scene_graph_iterators.h"
#include "zmq_bindings.h"

namespace py = pybind11;
using namespace py::literals;

using namespace spark_dsg;

PYBIND11_MAKE_OPAQUE(std::map<size_t, MeshEdge>);

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

PYBIND11_MODULE(_dsg_bindings, module) {
  py::options options;
  // options.disable_function_signatures();

  add_zmq_bindings(module);

  py::enum_<BoundingBox::Type>(module, "BoundingBoxType")
      .value("INVALID", BoundingBox::Type::INVALID)
      .value("AABB", BoundingBox::Type::AABB)
      .value("OBB", BoundingBox::Type::OBB)
      .value("RAABB", BoundingBox::Type::RAABB);

  py::class_<DsgLayers>(module, "DsgLayers")
      .def_readonly_static("MESH", &DsgLayers::MESH)
      .def_readonly_static("OBJECTS", &DsgLayers::OBJECTS)
      .def_readonly_static("AGENTS", &DsgLayers::AGENTS)
      .def_readonly_static("PLACES", &DsgLayers::PLACES)
      .def_readonly_static("STRUCTURE", &DsgLayers::STRUCTURE)
      .def_readonly_static("ROOMS", &DsgLayers::ROOMS)
      .def_readonly_static("BUILDINGS", &DsgLayers::BUILDINGS);

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

  py::class_<BoundingBox>(module, "BoundingBox")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&>())
      .def(py::init(
          [](const Eigen::Vector3f& min,
             const Eigen::Vector3f& max,
             const Eigen::Vector3f& pos,
             const Quaternion<float>& rot) { return BoundingBox(min, max, pos, rot); }))
      .def_readwrite("type", &BoundingBox::type)
      .def_readwrite("min", &BoundingBox::min)
      .def_readwrite("max", &BoundingBox::max)
      .def_readwrite("world_P_center", &BoundingBox::world_P_center)
      .def_readwrite("world_R_center", &BoundingBox::world_R_center)
      .def("is_inside",
           static_cast<bool (BoundingBox::*)(const Eigen::Vector3d&) const>(
               &BoundingBox::isInside))
      .def("is_inside",
           static_cast<bool (BoundingBox::*)(const Eigen::Vector3f&) const>(
               &BoundingBox::isInside))
      .def("__repr__", [](const BoundingBox& box) {
        std::stringstream ss;
        ss << box;
        return ss.str();
      });

  py::class_<NodeAttributes>(module, "NodeAttributes")
      .def(py::init<>())
      .def_readwrite("position", &NodeAttributes::position)
      .def_readwrite("last_update_time_ns", &NodeAttributes::last_update_time_ns)
      .def("__repr__", [](const NodeAttributes& attrs) {
        std::stringstream ss;
        ss << attrs;
        return ss.str();
      });

  py::class_<SemanticNodeAttributes, NodeAttributes>(module, "SemanticNodeAttributes")
      .def(py::init<>())
      .def_readwrite("name", &SemanticNodeAttributes::name)
      .def_readwrite("color", &SemanticNodeAttributes::color)
      .def_readwrite("bounding_box", &SemanticNodeAttributes::bounding_box)
      .def_readwrite("semantic_label", &SemanticNodeAttributes::semantic_label);

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
      .def_readwrite("vertex", &NearestVertexInfo::vertex);

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
      .def_readwrite("is_active", &PlaceNodeAttributes::is_active);

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

  py::class_<NodeSymbol>(module, "NodeSymbol")
      .def(py::init([](char key, size_t index) { return NodeSymbol(key, index); }))
      .def(py::init([](size_t value) { return NodeSymbol(value); }))
      .def_property("category_id", &NodeSymbol::categoryId, nullptr)
      .def_property("category", &NodeSymbol::category, nullptr)
      .def_property(
          "value",
          [](const NodeSymbol& symbol) { return static_cast<NodeId>(symbol); },
          nullptr)
      .def("__repr__", &NodeSymbol::getLabel);

  py::class_<LayerPrefix>(module, "LayerPrefix")
      .def(py::init([](char key) { return LayerPrefix(key); }))
      .def(py::init([](char key, uint32_t index) { return LayerPrefix(key, index); }))
      .def(py::init([](uint32_t index) { return LayerPrefix(index); }))
      .def_property(
          "value",
          [](const LayerPrefix& prefix) { return static_cast<uint32_t>(prefix); },
          nullptr)
      .def("__repr__", [](const LayerPrefix& prefix) { return prefix.str(true); });

  py::class_<SceneGraphNode>(module, "SceneGraphNode")
      .def("has_parent", &SceneGraphNode::hasParent)
      .def("has_siblings", &SceneGraphNode::hasSiblings)
      .def("has_children", &SceneGraphNode::hasChildren)
      .def("get_parent", &SceneGraphNode::getParent)
      .def("siblings", &SceneGraphNode::siblings)
      .def("children", &SceneGraphNode::children)
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

  py::class_<DynamicSceneGraphNode, SceneGraphNode>(module, "DynamicSceneGraphNode")
      .def_property(
          "timestamp",
          [](const DynamicSceneGraphNode& node) { return node.timestamp.count(); },
          nullptr);

  py::class_<EdgeAttributes>(module, "EdgeAttributes")
      .def(py::init<>())
      .def_readwrite("weighted", &EdgeAttributes::weighted)
      .def_readwrite("weight", &EdgeAttributes::weight);

  py::class_<SceneGraphEdge>(module, "SceneGraphEdge")
      .def_readonly("source", &SceneGraphEdge::source)
      .def_readonly("target", &SceneGraphEdge::target)
      .def_property(
          "info",
          [](const SceneGraphEdge& edge) { return *(edge.info); },
          [](SceneGraphEdge& edge, const EdgeAttributes& info) { *edge.info = info; })
      .def("__repr__", [](const SceneGraphEdge& edge) {
        std::stringstream ss;
        ss << "Edge<source=" << NodeSymbol(edge.source).getLabel()
           << ", target=" << NodeSymbol(edge.target).getLabel() << ">";
        return ss.str();
      });

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
      .def("get_node", &IsolatedSceneGraphLayer::getNode)
      .def("get_edge", &IsolatedSceneGraphLayer::getEdge)
      .def("remove_edge", &IsolatedSceneGraphLayer::removeEdge)
      .def("num_nodes", &IsolatedSceneGraphLayer::numNodes)
      .def("num_edges", &IsolatedSceneGraphLayer::numEdges)
      .def("get_position", &IsolatedSceneGraphLayer::getPosition)
      .def("to_bson",
           [](const IsolatedSceneGraphLayer& layer) -> py::bytes {
             return layer.toBson();
           })
      .def("to_json",
           [](const IsolatedSceneGraphLayer& layer) {
             std::unordered_set<NodeId> nodes;
             for (const auto& id_node_pair : layer.nodes()) {
               nodes.insert(id_node_pair.first);
             }
             return layer.serializeLayer(nodes);
           })
      .def_static("from_json", &IsolatedSceneGraphLayer::readFromJson)
      .def_static("from_bson",
                  [](const py::bytes& contents) {
                    return IsolatedSceneGraphLayer::fromBson(contents);
                  })
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

  py::class_<LayerView>(module, "LayerView")
      .def("has_node", &LayerView::hasNode)
      .def("has_edge", &LayerView::hasEdge)
      .def("get_node", &LayerView::getNode)
      .def("get_edge", &LayerView::getEdge)
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

  py::class_<MeshEdge>(module, "MeshEdge")
      .def_property_readonly(
          "node_id", [](const MeshEdge& edge) { return NodeSymbol(edge.source_node); })
      .def_readonly("mesh_vertex", &MeshEdge::mesh_vertex);

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
      .def("insert_edge",
           [](DynamicSceneGraph& graph, NodeId source, NodeId target) {
             return graph.insertEdge(source, target);
           })
      .def("insert_edge",
           [](DynamicSceneGraph& graph,
              NodeId source,
              NodeId target,
              const EdgeAttributes& info) {
             return graph.insertEdge(source, target, info.clone());
           })
      .def("has_layer",
           static_cast<bool (DynamicSceneGraph::*)(LayerId) const>(
               &DynamicSceneGraph::hasLayer))
      .def("has_layer",
           static_cast<bool (DynamicSceneGraph::*)(LayerId, LayerPrefix) const>(
               &DynamicSceneGraph::hasLayer))
      .def("has_node", &DynamicSceneGraph::hasNode)
      .def("has_edge",
           py::overload_cast<NodeId, NodeId>(&DynamicSceneGraph::hasEdge, py::const_))
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
      .def("get_node", &DynamicSceneGraph::getNode)
      .def("get_dynamic_node", &DynamicSceneGraph::getDynamicNode)
      .def("get_edge", &DynamicSceneGraph::getEdge)
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
      .def("num_edges", &DynamicSceneGraph::numEdges, "include_mesh"_a = false)
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
      .def_static("load", &DynamicSceneGraph::load)
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
      .def("clone", &DynamicSceneGraph::clone)
      .def("__deepcopy__",
           [](const DynamicSceneGraph& G, py::object) { return G.clone(); })
      .def("get_mesh_vertices",
           [](const DynamicSceneGraph& G) {
             auto vertices = G.getMeshVertices();
             if (!vertices) {
               return Eigen::MatrixXd();
             }

             Eigen::MatrixXd to_return(6, vertices->size());
             for (size_t i = 0; i < vertices->size(); ++i) {
               const auto& point = vertices->at(i);
               to_return(0, i) = point.x;
               to_return(1, i) = point.y;
               to_return(2, i) = point.z;
               to_return(3, i) = point.r / 255.0;
               to_return(4, i) = point.g / 255.0;
               to_return(5, i) = point.b / 255.0;
             }
             return to_return;
           })
      .def("get_mesh_faces",
           [](const DynamicSceneGraph& G) {
             auto faces = G.getMeshFaces();
             if (!faces) {
               return Eigen::MatrixXi();
             }

             Eigen::MatrixXi to_return(3, faces->size());
             for (size_t i = 0; i < faces->size(); ++i) {
               const auto& face = faces->at(i);
               to_return(0, i) = face.vertices.at(0);
               to_return(1, i) = face.vertices.at(1);
               to_return(2, i) = face.vertices.at(2);
             }
             return to_return;
           })
      .def("set_mesh_vertices",
           [](DynamicSceneGraph& G, const Eigen::MatrixXd& points) {
             if (points.rows() != 6) {
               std::stringstream ss;
               ss << "point rows do not match expected: " << points.rows() << " != 6";
               throw std::invalid_argument(ss.str());
             }

             DynamicSceneGraph::MeshVertices::Ptr vertices(
                 new DynamicSceneGraph::MeshVertices());
             for (int i = 0; i < points.cols(); ++i) {
               pcl::PointXYZRGBA point;
               point.x = points(0, i);
               point.y = points(1, i);
               point.z = points(2, i);
               point.r = static_cast<uint8_t>(points(3, i) * 255);
               point.g = static_cast<uint8_t>(points(4, i) * 255);
               point.b = static_cast<uint8_t>(points(5, i) * 255);
               point.a = 255;
               vertices->push_back(point);
             }
             G.setMesh(vertices, G.getMeshFaces(), false);
           })
      .def("set_mesh_faces",
           [](DynamicSceneGraph& G, const Eigen::MatrixXd& indices) {
             if (indices.rows() != 3) {
               std::stringstream ss;
               ss << "index rows do not match expected: " << indices.rows() << " != 3";
               throw std::invalid_argument(ss.str());
             }

             std::shared_ptr<DynamicSceneGraph::MeshFaces> faces(
                 new DynamicSceneGraph::MeshFaces());
             for (int i = 0; i < indices.cols(); ++i) {
               pcl::Vertices face;
               face.vertices.push_back(indices(0, i));
               face.vertices.push_back(indices(1, i));
               face.vertices.push_back(indices(2, i));
               faces->push_back(face);
             }
             G.setMesh(G.getMeshVertices(), faces, false);
           })
      .def("insert_mesh_edge",
           &DynamicSceneGraph::insertMeshEdge,
           "source"_a,
           "mesh_vertex"_a,
           "allow_invalid_mesh"_a = true)
      .def("remove_mesh_edge", &DynamicSceneGraph::removeMeshEdge)
      .def("clear_mesh_edges", &DynamicSceneGraph::clearMeshEdges)
      .def("invalidate_mesh_vertex", &DynamicSceneGraph::invalidateMeshVertex)
      .def("get_mesh_connections", &DynamicSceneGraph::getMeshConnectionIndices)
      .def_property_readonly("mesh_edges", &DynamicSceneGraph::getMeshEdges)
      .def(
          "to_binary",
          [](const DynamicSceneGraph& graph, bool include_mesh) {
            std::vector<uint8_t> buffer;
            writeGraph(graph, buffer, include_mesh);
            return py::bytes(reinterpret_cast<char*>(buffer.data()), buffer.size());
          },
          "include_mesh"_a = false)
      .def_static("from_binary", [](const py::bytes& contents) {
        const auto& view = static_cast<const std::string_view&>(contents);
        return readGraph(reinterpret_cast<const uint8_t*>(view.data()), view.size());
      });

  module.def("compute_ancestor_bounding_box",
             &computeAncestorBoundingBox,
             "G"_a,
             "node_id"_a,
             "child_layer"_a = DsgLayers::PLACES,
             "bbox_type"_a = BoundingBox::Type::AABB);

  py::implicitly_convertible<char, LayerPrefix>();
}
