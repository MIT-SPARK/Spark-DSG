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

namespace spark_dsg::python {

struct IterSentinel {};

class NodeIter {
 public:
  NodeIter(const SceneGraphLayer::Nodes& container);
  const SceneGraphNode* operator*() const;
  NodeIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  typename SceneGraphLayer::Nodes::const_iterator curr_iter_;
  typename SceneGraphLayer::Nodes::const_iterator end_iter_;
};

class DynamicNodeIter {
 public:
  DynamicNodeIter(const DynamicSceneGraphLayer::Nodes& container);
  const SceneGraphNode* operator*() const;
  DynamicNodeIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  typename DynamicSceneGraphLayer::Nodes::const_iterator curr_iter_;
  typename DynamicSceneGraphLayer::Nodes::const_iterator end_iter_;
};

class EdgeIter {
 public:
  EdgeIter(const SceneGraphLayer::Edges& container);
  const SceneGraphEdge* operator*() const;
  EdgeIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  typename SceneGraphLayer::Edges::const_iterator curr_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_iter_;
};

class GlobalNodeIter {
 public:
  GlobalNodeIter(const DynamicSceneGraph& dsg);
  void setNodeIter();
  const SceneGraphNode& operator*() const;
  GlobalNodeIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  bool valid_;
  typename SceneGraphLayer::Nodes::const_iterator curr_node_iter_;
  typename SceneGraphLayer::Nodes::const_iterator end_node_iter_;
  typename DynamicSceneGraph::Layers::const_iterator curr_layer_iter_;
  typename DynamicSceneGraph::Layers::const_iterator end_layer_iter_;
};

class GlobalEdgeIter {
 public:
  GlobalEdgeIter(const DynamicSceneGraph& dsg);
  const SceneGraphEdge* operator*() const;
  void setEdgeIter();
  GlobalEdgeIter& operator++();
  bool operator==(const IterSentinel&);

 private:
  bool started_interlayer_;
  typename SceneGraphLayer::Edges::const_iterator curr_edge_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_edge_iter_;
  typename DynamicSceneGraph::Layers::const_iterator curr_layer_iter_;
  typename DynamicSceneGraph::Layers::const_iterator end_layer_iter_;
  typename SceneGraphLayer::Edges::const_iterator curr_interlayer_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_interlayer_iter_;
};

}  // namespace spark_dsg::python
