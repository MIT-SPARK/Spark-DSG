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
#pragma once
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_symbol.h>

#include "spark_dsg/python/scene_graph_iterators.h"

namespace spark_dsg::python {

class LayerView {
 public:
  LayerView(const SceneGraphLayer& layer);
  NodeIter nodes() const;
  EdgeIter edges() const;
  size_t numNodes() const;
  size_t numEdges() const;
  bool hasNode(NodeSymbol node_id) const;
  bool hasEdge(NodeSymbol source, NodeSymbol target) const;
  const SceneGraphNode& getNode(NodeSymbol node_id) const;
  const SceneGraphEdge& getEdge(NodeSymbol source, NodeSymbol target) const;
  Eigen::Vector3d getPosition(NodeSymbol node_id) const;

  const LayerId id;

 private:
  const SceneGraphLayer& layer_ref_;
};

class LayerIter {
 public:
  LayerIter(const DynamicSceneGraph::Layers& container);
  LayerView operator*() const;
  LayerIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  DynamicSceneGraph::Layers::const_iterator curr_iter_;
  DynamicSceneGraph::Layers::const_iterator end_iter_;
};

class DynamicLayerView {
 public:
  DynamicLayerView(const DynamicSceneGraphLayer& layer);
  DynamicNodeIter nodes() const;
  EdgeIter edges() const;
  size_t numNodes() const;
  size_t numEdges() const;

  const LayerId id;
  const LayerPrefix prefix;

 private:
  const DynamicSceneGraphLayer& layer_ref_;
};

class DynamicLayerIter {
 public:
  using LayerMap = std::map<LayerId, DynamicSceneGraph::DynamicLayers>;

  DynamicLayerIter(const LayerMap& container);
  void setSubIter();
  DynamicLayerView operator*() const;
  DynamicLayerIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  bool valid_;
  LayerMap::const_iterator curr_iter_;
  LayerMap::const_iterator end_iter_;
  DynamicSceneGraph::DynamicLayers::const_iterator curr_layer_iter_;
  DynamicSceneGraph::DynamicLayers::const_iterator end_layer_iter_;
};

}  // namespace spark_dsg::python
