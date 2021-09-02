#include <kimera_dsg/dynamic_scene_graph.h>
#include <kimera_dsg/node_attributes.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <iostream>
#include <sstream>
#include <string>

namespace py = pybind11;
using namespace py::literals;

using namespace kimera;

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
std::ostream &operator<<(std::ostream &out, const Quaternion<Scalar> &q) {
  out << "Quaternion<w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z
      << ">";
  return out;
}

struct IterSentinel {};

class NodeIter {
 public:
  NodeIter(const SceneGraphLayer::Nodes &container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  const SceneGraphLayer::Node *operator*() const { return curr_iter_->second.get(); }

  NodeIter &operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel &) { return curr_iter_ == end_iter_; }

 private:
  typename SceneGraphLayer::Nodes::const_iterator curr_iter_;
  typename SceneGraphLayer::Nodes::const_iterator end_iter_;
};

class EdgeIter {
 public:
  EdgeIter(const SceneGraphLayer::Edges &container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  const SceneGraphLayer::Edge *operator*() const { return &(curr_iter_->second); }

  EdgeIter &operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel &) { return curr_iter_ == end_iter_; }

 private:
  typename SceneGraphLayer::Edges::const_iterator curr_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_iter_;
};

class LayerView {
 public:
  LayerView(const SceneGraphLayer &layer) : id(layer.id), layer_ref_(layer) {}

  NodeIter nodes() const { return NodeIter(layer_ref_.nodes()); }

  EdgeIter edges() const { return EdgeIter(layer_ref_.edges()); }

  const LayerId id;

 private:
  const SceneGraphLayer &layer_ref_;
};

class LayerIter {
 public:
  LayerIter(const SceneGraph::Layers &container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  LayerView operator*() const { return LayerView(*(curr_iter_->second)); }

  LayerIter &operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel &) { return curr_iter_ == end_iter_; }

 private:
  typename SceneGraph::Layers::const_iterator curr_iter_;
  typename SceneGraph::Layers::const_iterator end_iter_;
};

PYBIND11_MODULE(kimera_dsg_python_bindings, module) {
  py::enum_<BoundingBox::Type>(module, "BoundingBoxType")
      .value("INVALID", BoundingBox::Type::INVALID)
      .value("AABB", BoundingBox::Type::AABB)
      .value("OBB", BoundingBox::Type::OBB);

  py::enum_<KimeraDsgLayers>(module, "KimeraDsgLayers")
      .value("INVALID", KimeraDsgLayers::INVALID)
      .value("MESH", KimeraDsgLayers::MESH)
      .value("OBJECTS", KimeraDsgLayers::OBJECTS)
      .value("PLACES", KimeraDsgLayers::PLACES)
      .value("ROOMS", KimeraDsgLayers::ROOMS)
      .value("BUILDINGS", KimeraDsgLayers::BUILDINGS);

  py::class_<Quaternion<float>>(module, "Quaternionf")
      .def(py::init<>())
      .def(py::init<float, float, float, float>())
      .def_readwrite("w", &Quaternion<float>::w)
      .def_readwrite("x", &Quaternion<float>::x)
      .def_readwrite("y", &Quaternion<float>::y)
      .def_readwrite("z", &Quaternion<float>::z)
      .def("__repr__", [](const Quaternion<float> &q) {
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
      .def("__repr__", [](const Quaternion<double> &q) {
        std::stringstream ss;
        ss << q;
        return ss.str();
      });

  py::class_<BoundingBox>(module, "BoundingBox")
      .def(py::init<>())
      .def(py::init<const Eigen::Vector3f &, const Eigen::Vector3f &>())
      .def(py::init(
          [](const Eigen::Vector3f &min,
             const Eigen::Vector3f &max,
             const Eigen::Vector3f &pos,
             const Quaternion<float> &rot) { return BoundingBox(min, max, pos, rot); }))
      .def_readwrite("type", &BoundingBox::type)
      .def_readwrite("min", &BoundingBox::min)
      .def_readwrite("max", &BoundingBox::max)
      .def_readwrite("world_P_center", &BoundingBox::world_P_center)
      .def_readwrite("world_R_center", &BoundingBox::world_R_center)
      .def("__repr__", [](const BoundingBox &box) {
        std::stringstream ss;
        ss << box;
        return ss.str();
      });

  py::class_<NodeAttributes>(module, "NodeAttributes")
      .def(py::init<>())
      .def_readwrite("position", &NodeAttributes::position)
      .def("__repr__", [](const NodeAttributes &attrs) {
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
      .def_property("world_R_object",
                    [](const ObjectNodeAttributes &attrs) {
                      return Quaternion<double>(attrs.world_R_object);
                    },
                    [](ObjectNodeAttributes &attrs, const Quaternion<double> &rot) {
                      attrs.world_R_object = rot;
                    });

  py::class_<RoomNodeAttributes, SemanticNodeAttributes>(module, "RoomNodeAttributes")
      .def(py::init<>());

  py::class_<PlaceNodeAttributes, SemanticNodeAttributes>(module, "PlaceNodeAttributes")
      .def(py::init<>())
      .def_readwrite("distance", &PlaceNodeAttributes::distance)
      .def_readwrite("num_basis_points", &PlaceNodeAttributes::num_basis_points);

  py::class_<NodeSymbol>(module, "NodeSymbol")
      .def(py::init([](char key, size_t index) { return NodeSymbol(key, index); }))
      .def_property("category_id", &NodeSymbol::categoryId, nullptr)
      .def_property("category", &NodeSymbol::category, nullptr)
      .def_property(
          "value",
          [](const NodeSymbol &symbol) { return static_cast<NodeId>(symbol); },
          nullptr)
      .def("__repr__", &NodeSymbol::getLabel);

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
      .def_readonly("id", &SceneGraphNode::id)
      .def_readonly("layer", &SceneGraphNode::layer)
      .def("__repr__", [](const SceneGraphNode &node) {
        std::stringstream ss;
        ss << node;
        return ss.str();
      });

  py::class_<SceneGraphEdgeInfo>(module, "SceneGraphEdgeInfo")
      .def(py::init<>())
      .def_readwrite("weighted", &SceneGraphEdgeInfo::weighted)
      .def_readwrite("weight", &SceneGraphEdgeInfo::weight);

  py::class_<SceneGraphEdge>(module, "SceneGraphEdge")
      .def_readonly("source", &SceneGraphEdge::source)
      .def_readonly("target", &SceneGraphEdge::target)
      .def_property("info",
                    [](const SceneGraphEdge &edge) { return *(edge.info); },
                    [](SceneGraphEdge &edge, const SceneGraphEdgeInfo &info) {
                      *edge.info = info;
                    })
      .def("__repr__", [](const SceneGraphEdge &edge) {
        std::stringstream ss;
        ss << "Edge<source=" << NodeSymbol(edge.source).getLabel()
           << ", target=" << NodeSymbol(edge.target).getLabel() << ">";
        return ss.str();
      });

  // TODO(nathan) iterator over nodes and edges
  py::class_<SceneGraphLayer>(module, "SceneGraphLayer")
      .def(py::init<LayerId>())
      .def("insert_edge",
           [](SceneGraphLayer &layer, NodeId source, NodeId target) {
             layer.insertEdge(source, target);
           })
      .def("insert_edge",
           [](SceneGraphLayer &layer,
              NodeId source,
              NodeId target,
              const SceneGraphEdgeInfo &info) {
             SceneGraphEdgeInfo::Ptr edge_info(new SceneGraphEdgeInfo());
             *edge_info = info;
             layer.insertEdge(source, target, std::move(edge_info));
           })
      .def("has_node", &SceneGraphLayer::hasNode)
      .def("has_edge", &SceneGraphLayer::hasEdge)
      .def("get_node", &SceneGraphLayer::getNode)
      .def("get_edge", &SceneGraphLayer::getEdge)
      .def("remove_edge", &SceneGraphLayer::removeEdge)
      .def("num_nodes", &SceneGraphLayer::numNodes)
      .def("num_edges", &SceneGraphLayer::numEdges)
      .def("get_position", &SceneGraphLayer::getPosition)
      .def_readonly("id", &SceneGraphLayer::id);

  py::class_<LayerView>(module, "LayerView")
      .def_readonly("id", &LayerView::id)
      .def_property("nodes",
                    [](const LayerView &view) {
                      return py::make_iterator(view.nodes(), IterSentinel());
                    },
                    nullptr,
                    py::return_value_policy::reference_internal)
      .def_property("edges",
                    [](const LayerView &view) {
                      return py::make_iterator(view.edges(), IterSentinel());
                    },
                    nullptr,
                    py::return_value_policy::reference_internal);

#define MAKE_SPECIALIZED_NODE_ADD(AttributeClass)                   \
  def("add_node",                                                   \
      [](SceneGraph &graph,                                         \
         LayerId layer_id,                                          \
         NodeId node_id,                                            \
         const AttributeClass &attrs) {                             \
        AttributeClass::Ptr new_attrs(new AttributeClass(attrs));   \
        graph.emplaceNode(layer_id, node_id, std::move(new_attrs)); \
      })

  py::class_<SceneGraph>(module, "SceneGraph")
      .def(py::init<>())
      .def(py::init<const SceneGraph::LayerIds &>())
      .def("clear", &SceneGraph::clear)
      .MAKE_SPECIALIZED_NODE_ADD(NodeAttributes)
      .MAKE_SPECIALIZED_NODE_ADD(SemanticNodeAttributes)
      .MAKE_SPECIALIZED_NODE_ADD(ObjectNodeAttributes)
      .MAKE_SPECIALIZED_NODE_ADD(RoomNodeAttributes)
      .MAKE_SPECIALIZED_NODE_ADD(PlaceNodeAttributes)
      .def("insert_edge",
           [](SceneGraph &graph, NodeId source, NodeId target) {
             graph.insertEdge(source, target);
           })
      .def("insert_edge",
           [](SceneGraph &graph,
              NodeId source,
              NodeId target,
              const SceneGraphEdgeInfo &info) {
             SceneGraphEdgeInfo::Ptr edge_info(new SceneGraphEdgeInfo());
             *edge_info = info;
             graph.insertEdge(source, target, std::move(edge_info));
           })
      .def("has_layer",
           static_cast<bool (SceneGraph::*)(LayerId) const>(&SceneGraph::hasLayer))
      .def("has_node", &SceneGraph::hasNode)
      .def("has_edge", &SceneGraph::hasEdge)
      .def("get_layer",
           [](const SceneGraph &graph, LayerId layer_id) {
             if (!graph.hasLayer(layer_id)) {
               throw std::out_of_range("layer doesn't exist");
             }
             return LayerView(*graph.getLayer(layer_id));
           },
           py::return_value_policy::reference_internal)
      .def("get_node", &SceneGraph::getNode)
      .def("get_edge", &SceneGraph::getEdge)
      .def("remove_node", &SceneGraph::removeNode)
      .def("remove_edge", &SceneGraph::removeEdge)
      .def("num_layers", &SceneGraph::numLayers)
      .def("num_nodes", &SceneGraph::numNodes)
      .def("empty", &SceneGraph::empty)
      .def("num_edges", &SceneGraph::numEdges)
      .def("get_position", &SceneGraph::getPosition)
      .def("save", &SceneGraph::save, "filepath"_a, "force_bson"_a = false)
      .def("load", &SceneGraph::load, "filepath"_a, "force_bson"_a = false)
      .def_property("layers",
                    [](const SceneGraph &graph) {
                      return py::make_iterator(LayerIter(graph.layers()),
                                               IterSentinel());
                    },
                    nullptr,
                    py::return_value_policy::reference_internal)
      .def_property("inter_layer_edges",
                    [](const SceneGraph &graph) {
                      return py::make_iterator(EdgeIter(graph.inter_layer_edges()),
                                               IterSentinel());
                    },
                    nullptr,
                    py::return_value_policy::reference_internal);

#undef MAKE_SPECIALZIED_NODE_ADD
}
