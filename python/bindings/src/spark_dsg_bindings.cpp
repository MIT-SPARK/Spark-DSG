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
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/edge_container.h>
#include <spark_dsg/layer_prefix.h>
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

#include "spark_dsg/python/mesh_helpers.h"
#include "spark_dsg/python/python_layer_view.h"
#include "spark_dsg/python/quaternion.h"
#include "spark_dsg/python/scene_graph_iterators.h"

namespace py = pybind11;
using namespace py::literals;

using namespace spark_dsg;
using spark_dsg::python::DynamicLayerIter;
using spark_dsg::python::DynamicLayerView;
using spark_dsg::python::EdgeIter;
using spark_dsg::python::GlobalEdgeIter;
using spark_dsg::python::GlobalNodeIter;
using spark_dsg::python::IterSentinel;
using spark_dsg::python::LayerIter;
using spark_dsg::python::LayerView;
using spark_dsg::python::NodeIter;
using spark_dsg::python::Quaternion;

PYBIND11_MODULE(_dsg_bindings, module) {
  py::options options;
  // options.disable_function_signatures();

  /*************************************************************************************
   * Helper types
   ************************************************************************************/

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

  /**************************************************************************************
   * Enums, misc types, and other utilities
   *************************************************************************************/

  py::class_<DsgLayers>(module, "DsgLayers")
      .def_readonly_static("SEGMENTS", &DsgLayers::SEGMENTS)
      .def_readonly_static("OBJECTS", &DsgLayers::OBJECTS)
      .def_readonly_static("AGENTS", &DsgLayers::AGENTS)
      .def_readonly_static("PLACES", &DsgLayers::PLACES)
      .def_readonly_static("MESH_PLACES", &DsgLayers::MESH_PLACES)
      .def_readonly_static("STRUCTURE", &DsgLayers::STRUCTURE)
      .def_readonly_static("ROOMS", &DsgLayers::ROOMS)
      .def_readonly_static("BUILDINGS", &DsgLayers::BUILDINGS);

  py::class_<LayerPrefix>(module, "LayerPrefix")
      .def(py::init([](char key) { return LayerPrefix(key); }))
      .def(py::init([](char key, uint32_t index) { return LayerPrefix(key, index); }))
      .def(py::init([](uint32_t index) { return LayerPrefix(index); }))
      .def_property(
          "value",
          [](const LayerPrefix& prefix) { return static_cast<uint32_t>(prefix); },
          nullptr)
      .def("__repr__", [](const LayerPrefix& prefix) { return prefix.str(true); });

  py::implicitly_convertible<char, LayerPrefix>();

  py::class_<NodeSymbol>(module, "NodeSymbol")
      .def(py::init([](char key, size_t index) { return NodeSymbol(key, index); }))
      .def(py::init([](size_t value) { return NodeSymbol(value); }))
      .def_property("category_id", &NodeSymbol::categoryId, nullptr)
      .def_property("category", &NodeSymbol::category, nullptr)
      .def_property(
          "value",
          [](const NodeSymbol& symbol) { return static_cast<NodeId>(symbol); },
          nullptr)
      .def("__repr__", &NodeSymbol::getLabel)
      .def("__hash__",
           [](const NodeSymbol& symbol) { return static_cast<NodeId>(symbol); })
      .def(pybind11::self == pybind11::self)
      .def(pybind11::self != pybind11::self);

  py::implicitly_convertible<NodeId, NodeSymbol>();

  py::class_<NearestVertexInfo>(module, "NearestVertexInfo")
      .def(py::init<>())
      .def_property(
          "block",
          [](const NearestVertexInfo& info) -> Eigen::Vector3i {
            return {info.block[0], info.block[1], info.block[2]};
          },
          [](NearestVertexInfo& info, const Eigen::Vector3i& array) {
            info.block[0] = array.x();
            info.block[1] = array.y();
            info.block[2] = array.z();
          })
      .def_property(
          "voxel_pos",
          [](const NearestVertexInfo& info) -> Eigen::Vector3d {
            return {info.voxel_pos[0], info.voxel_pos[1], info.voxel_pos[2]};
          },
          [](NearestVertexInfo& info, const Eigen::Vector3d& array) {
            info.voxel_pos[0] = array.x();
            info.voxel_pos[1] = array.y();
            info.voxel_pos[2] = array.z();
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

  py::class_<Color>(module, "Color")
      .def_readwrite("r", &Color::r)
      .def_readwrite("g", &Color::g)
      .def_readwrite("b", &Color::b)
      .def_readwrite("a", &Color::a);

  py::enum_<BoundingBox::Type>(module, "BoundingBoxType")
      .value("INVALID", BoundingBox::Type::INVALID)
      .value("AABB", BoundingBox::Type::AABB)
      .value("OBB", BoundingBox::Type::OBB)
      .value("RAABB", BoundingBox::Type::RAABB);

  module.def("compute_ancestor_bounding_box",
             &computeAncestorBoundingBox,
             "G"_a,
             "node_id"_a,
             "child_layer"_a = DsgLayers::PLACES,
             "bbox_type"_a = BoundingBox::Type::AABB);

  /**************************************************************************************
   * Bounding Box
   *************************************************************************************/

  py::class_<BoundingBox>(module, "BoundingBox")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3f&>())
      .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&>())
      .def(py::init<const Eigen::Vector3f&, const Eigen::Vector3f&, float>())
      .def(py::init([](BoundingBox::Type type,
                       const Eigen::Vector3f& dim,
                       const Eigen::Vector3f& pos,
                       const Eigen::Matrix3f& trans) {
        return BoundingBox(type, dim, pos, trans);
      }))
      .def_readwrite("type", &BoundingBox::type)
      .def_readwrite("dimensions", &BoundingBox::dimensions)
      .def_readwrite("world_P_center", &BoundingBox::world_P_center)
      .def_readwrite("world_R_center", &BoundingBox::world_R_center)
      .def("is_valid", &BoundingBox::isValid)
      .def("volume", &BoundingBox::volume)
      .def("has_rotation", &BoundingBox::hasRotation)
      .def("corners", &BoundingBox::corners)
      .def("contains",
           static_cast<bool (BoundingBox::*)(const Eigen::Vector3f&) const>(
               &BoundingBox::contains))
      .def("intersects",
           static_cast<bool (BoundingBox::*)(const BoundingBox&) const>(
               &BoundingBox::intersects))
      .def("compute_iou",
           static_cast<float (BoundingBox::*)(const BoundingBox&) const>(
               &BoundingBox::computeIoU))
      .def_property_readonly("min",
                             [](const BoundingBox& box) {
                               return box.pointToWorldFrame(-box.dimensions / 2);
                             })
      .def_property_readonly("max",
                             [](const BoundingBox& box) {
                               return box.pointToWorldFrame(box.dimensions / 2);
                             })
      .def("__repr__", [](const BoundingBox& box) {
        std::stringstream ss;
        ss << box;
        return ss.str();
      });

  /**************************************************************************************
   * Node Attributes
   *************************************************************************************/

  py::class_<NodeAttributes>(module, "NodeAttributes")
      .def(py::init<>())
      .def_readwrite("position", &NodeAttributes::position)
      .def_readwrite("last_update_time_ns", &NodeAttributes::last_update_time_ns)
      .def_readwrite("is_active", &NodeAttributes::is_active)
      .def_readwrite("is_predicted", &NodeAttributes::is_predicted)
      .def("_get_metadata",
           [](const NodeAttributes& node) {
             std::stringstream ss;
             ss << node.metadata;
             return ss.str();
           })
      .def("_set_metadata",
           [](NodeAttributes& node, const std::string& data) {
             node.metadata = nlohmann::json::parse(data);
           })
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

  py::class_<KhronosObjectAttributes, ObjectNodeAttributes>(module,
                                                            "KhronosObjectAttributes")
      .def(py::init<>())
      .def_readonly("first_observed_ns", &KhronosObjectAttributes::first_observed_ns)
      .def_readonly("last_observed_ns", &KhronosObjectAttributes::last_observed_ns)
      .def(
          "mesh",
          [](const KhronosObjectAttributes& attrs) { return &attrs.mesh; },
          py::return_value_policy::reference_internal)
      .def_readonly("trajectory_timestamps",
                    &KhronosObjectAttributes::trajectory_timestamps)
      .def_readonly("trajectory_positions",
                    &KhronosObjectAttributes::trajectory_positions)
      .def_readonly("dynamic_object_points",
                    &KhronosObjectAttributes::dynamic_object_points)
      .def_readonly("details", &KhronosObjectAttributes::details);

  py::class_<RoomNodeAttributes, SemanticNodeAttributes>(module, "RoomNodeAttributes")
      .def(py::init<>())
      .def_readwrite("semantic_class_probabilities",
                     &RoomNodeAttributes::semantic_class_probabilities);

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

  /**************************************************************************************
   * Edge Attributes
   *************************************************************************************/

  py::class_<EdgeAttributes>(module, "EdgeAttributes")
      .def(py::init<>())
      .def_readwrite("weighted", &EdgeAttributes::weighted)
      .def_readwrite("weight", &EdgeAttributes::weight)
      .def("_get_metadata",
           [](const EdgeAttributes& edge) {
             std::stringstream ss;
             ss << edge.metadata;
             return ss.str();
           })
      .def("_set_metadata", [](EdgeAttributes& edge, const std::string& data) {
        edge.metadata = nlohmann::json::parse(data);
      });

  /**************************************************************************************
   * Scene graph node
   *************************************************************************************/

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

  /**************************************************************************************
   * Scene graph edge
   *************************************************************************************/

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

  /**************************************************************************************
   * Mesh
   *************************************************************************************/

  py::class_<Mesh, std::shared_ptr<Mesh>>(module, "Mesh")
      .def(py::init<bool, bool, bool, bool>(),
           "has_colors"_a = true,
           "has_timestamps"_a = true,
           "has_labels"_a = true,
           "has_first_seen_stamps"_a = true)
      .def("empty", &Mesh::empty)
      .def("clear", &Mesh::clear)
      .def("num_vertices", &Mesh::numVertices)
      .def("num_faces", &Mesh::numFaces)
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
      .def("set_face",
           [](Mesh& mesh, size_t index, const Mesh::Face& face) {
             mesh.face(index) = face;
           })
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
                    return Mesh::deserializeFromBinary(
                        reinterpret_cast<const uint8_t*>(view.data()), view.size());
                  })
      .def("save", &Mesh::save)
      .def("save",
           [](const Mesh& mesh, const std::filesystem::path& path) { mesh.save(path); })
      .def_static("load", &Mesh::load)
      .def_static("load",
                  [](const std::filesystem::path& path) { return Mesh::load(path); })
      .def("get_vertices",
           [](const Mesh& mesh) { return spark_dsg::python::getEigenVertices(mesh); })
      .def("get_faces",
           [](const Mesh& mesh) { return spark_dsg::python::getEigenFaces(mesh); })
      .def("get_labels", [](const Mesh& mesh) { return mesh.labels; })
      .def("set_vertices",
           [](Mesh& mesh, const Eigen::MatrixXd& points) {
             spark_dsg::python::setEigenVertices(mesh, points);
           })
      .def("set_faces",
           [](Mesh& mesh, const Eigen::MatrixXi& faces) {
             spark_dsg::python::setEigenFaces(mesh, faces);
           })
      .def(
          "transform",
          [](Mesh& mesh,
             const Eigen::Matrix3d& rotation,
             const Eigen::Vector3d& translation) {
            const Eigen::Isometry3d transform =
                Eigen::Translation3d(translation) * Eigen::Quaterniond(rotation);
            mesh.transform(transform.cast<float>());
          },
          "rotation"_a = Eigen::Matrix3d::Identity(),
          "translation"_a = Eigen::Vector3d::Zero())
      .def("append", &Mesh::append)
      .def(py::self += py::self);

  /**************************************************************************************
   * Scene graph layer
   *************************************************************************************/

  py::class_<IsolatedSceneGraphLayer, std::shared_ptr<IsolatedSceneGraphLayer>>(
      module, "SceneGraphLayer")
      .def(py::init<LayerId>())
      .def("add_node",
           [](IsolatedSceneGraphLayer& layer,
              NodeSymbol node,
              const NodeAttributes& attrs) { layer.emplaceNode(node, attrs.clone()); })
      .def("insert_edge",
           [](IsolatedSceneGraphLayer& layer, NodeSymbol source, NodeSymbol target) {
             return layer.insertEdge(source, target);
           })
      .def("insert_edge",
           [](IsolatedSceneGraphLayer& layer,
              NodeSymbol source,
              NodeSymbol target,
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
      .def("get_position",
           [](const IsolatedSceneGraphLayer& layer, NodeSymbol node) {
             return layer.getNode(node).attributes().position;
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
          py::return_value_policy::reference_internal)
      .def("to_binary",
           [](const IsolatedSceneGraphLayer& layer) -> py::bytes {
             std::vector<uint8_t> buffer;
             io::binary::writeLayer(layer, buffer);
             return py::bytes(reinterpret_cast<char*>(buffer.data()), buffer.size());
           })
      .def_static("from_binary", [](const py::bytes& contents) {
        const auto view = static_cast<std::string_view>(contents);
        return io::binary::readLayer(reinterpret_cast<const uint8_t*>(view.data()),
                                     view.size());
      });

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

  py::class_<DynamicSceneGraph, std::shared_ptr<DynamicSceneGraph>>(
      module, "DynamicSceneGraph", py::dynamic_attr())
      .def(py::init<>())
      .def(py::init<const DynamicSceneGraph::LayerIds&>())
      .def("clear", &DynamicSceneGraph::clear)
      .def("create_dynamic_layer", &DynamicSceneGraph::createDynamicLayer)
      .def("add_node",
           [](DynamicSceneGraph& graph,
              LayerId layer_id,
              NodeSymbol node_id,
              const NodeAttributes& attrs) {
             return graph.emplaceNode(layer_id, node_id, attrs.clone());
           })
      .def(
          "add_node",
          [](DynamicSceneGraph& graph,
             LayerId layer_id,
             LayerPrefix prefix,
             std::chrono::nanoseconds timestamp,
             const NodeAttributes& attrs,
             bool add_edge_to_previous) {
            return graph.emplaceNode(
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
             NodeSymbol source,
             NodeSymbol target,
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
             NodeSymbol source,
             NodeSymbol target,
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
      .def("has_node",
           [](const DynamicSceneGraph& graph, NodeSymbol node_id) {
             return graph.hasNode(node_id);
           })
      .def("has_edge",
           [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) {
             return graph.hasEdge(source, target);
           })
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
      .def(
          "get_node",
          [](const DynamicSceneGraph& graph, NodeSymbol node) -> const SceneGraphNode& {
            return graph.getNode(node);
          },
          py::return_value_policy::reference_internal)
      .def(
          "find_node",
          [](const DynamicSceneGraph& graph, NodeSymbol node) -> const SceneGraphNode* {
            return graph.findNode(node);
          },
          py::return_value_policy::reference_internal)
      .def(
          "get_edge",
          [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target)
              -> const SceneGraphEdge& { return graph.getEdge(source, target); },
          py::return_value_policy::reference_internal)
      .def(
          "find_edge",
          [](const DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target)
              -> const SceneGraphEdge* { return graph.findEdge(source, target); },
          py::return_value_policy::reference_internal)
      .def("remove_node",
           [](DynamicSceneGraph& graph, NodeSymbol node) -> bool {
             return graph.removeNode(node);
           })
      .def("remove_edge",
           [](DynamicSceneGraph& graph, NodeSymbol source, NodeSymbol target) -> bool {
             return graph.removeEdge(source, target);
           })
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
      .def("_get_metadata",
           [](const DynamicSceneGraph& graph) {
             std::stringstream ss;
             ss << graph.metadata;
             return ss.str();
           })
      .def("_set_metadata",
           [](DynamicSceneGraph& graph, const std::string& data) {
             graph.metadata = nlohmann::json::parse(data);
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
            const auto view = static_cast<std::string_view>(contents);
            return io::binary::updateGraph(
                graph, reinterpret_cast<const uint8_t*>(view.data()), view.size());
          },
          "contents"_a)
      .def_static("from_binary", [](const py::bytes& contents) {
        const auto view = static_cast<std::string_view>(contents);
        return io::binary::readGraph(reinterpret_cast<const uint8_t*>(view.data()),
                                     view.size());
      });

  /**************************************************************************************
   * Zmq Interface
   *************************************************************************************/

  py::class_<ZmqSender>(module, "DsgSender")
      .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
      .def("send", &ZmqSender::send, "graph"_a, "include_mesh"_a = false);

  py::class_<ZmqReceiver>(module, "DsgReceiver")
      .def(py::init<const std::string&, size_t>(), "url"_a, "num_threads"_a = 1)
      .def("recv", &ZmqReceiver::recv, "timeout_ms"_a, "recv_all"_a = false)
      .def_property_readonly("graph", [](const ZmqReceiver& receiver) {
        if (!receiver.graph()) {
          throw pybind11::value_error("no graph received yet");
        }
        return receiver.graph();
      });

  py::class_<ZmqGraph>(module, "ZmqGraph")
      .def(py::init<const std::string&, size_t, size_t>(),
           "url"_a,
           "num_threads"_a = 1,
           "poll_time_ms"_a = 100)
      .def_property_readonly(
          "has_change", [](const ZmqGraph& zmq_graph) { return zmq_graph.hasChange(); })
      .def_property_readonly(
          "graph", [](const ZmqGraph& zmq_graph) { return zmq_graph.graph(); });

  module.def("version",
             []() { return spark_dsg::io::FileHeader::current().version.toString(); });
}
