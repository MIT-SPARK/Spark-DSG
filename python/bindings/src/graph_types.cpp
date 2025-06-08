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
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <spark_dsg/bounding_box.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/edge_container.h>
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

#include <filesystem>
#include <iomanip>
#include <sstream>

#include "spark_dsg/python/attributes.h"
#include "spark_dsg/python/mesh.h"
#include "spark_dsg/python/python_layer_view.h"
#include "spark_dsg/python/python_types.h"
#include "spark_dsg/python/scene_graph_iterators.h"
#include "spark_dsg/python/spark_types.h"

namespace py = pybind11;
using namespace py::literals;

using namespace spark_dsg;
using spark_dsg::python::EdgeIter;
using spark_dsg::python::GlobalEdgeIter;
using spark_dsg::python::GlobalNodeIter;
using spark_dsg::python::IterSentinel;
using spark_dsg::python::LayerIter;
using spark_dsg::python::LayerView;
using spark_dsg::python::NodeIter;
using spark_dsg::python::PartitionIter;
using spark_dsg::python::PythonPartitionId;
using spark_dsg::python::Quaternion;

PYBIND11_MODULE(_dsg_bindings, module) {
  py::options options;

  spark_dsg::python::init_types(module);
  spark_dsg::python::init_spark_types(module);
  spark_dsg::python::init_mesh(module);
  spark_dsg::python::init_attributes(module);

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
      .def_property("attributes",
                    &SceneGraphNode::tryAttributes<NodeAttributes>,
                    &SceneGraphNode::tryAttributes<NodeAttributes>,
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
        ss << "Edge<source=" << NodeSymbol(edge.source).str()
           << ", target=" << NodeSymbol(edge.target).str() << ">";
        return ss.str();
      });
}
