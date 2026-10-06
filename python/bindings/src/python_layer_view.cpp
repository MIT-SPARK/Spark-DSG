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
#include "spark_dsg/python/python_layer_view.h"

#include <spark_dsg/printing.h>

namespace spark_dsg::python {

LayerView::LayerView(const SceneGraphLayer& layer) : id(layer.id), layer_ref(layer) {}

NodeIter LayerView::nodes() const { return NodeIter(layer_ref.nodes_); }

EdgeIter LayerView::edges() const { return EdgeIter(layer_ref.edges_.edges); }

size_t LayerView::numNodes() const { return layer_ref.numNodes(); }

size_t LayerView::numEdges() const { return layer_ref.numEdges(); }

bool LayerView::hasNode(NodeSymbol node_id) const { return layer_ref.hasNode(node_id); }

bool LayerView::hasEdge(NodeSymbol source, NodeSymbol target) const { return layer_ref.hasEdge(source, target); }

const SceneGraphNode& LayerView::getNode(NodeSymbol node_id) const { return layer_ref.getNode(node_id); }

const SceneGraphNode* LayerView::findNode(NodeSymbol node_id) const { return layer_ref.findNode(node_id); }

const SceneGraphEdge& LayerView::getEdge(NodeSymbol source, NodeSymbol target) const {
  return layer_ref.getEdge(source, target);
}

const SceneGraphEdge* LayerView::findEdge(NodeSymbol source, NodeSymbol target) const {
  return layer_ref.findEdge(source, target);
}

Eigen::Vector3d LayerView::getPosition(NodeSymbol node_id) const {
  return layer_ref.getNode(node_id).attributes().position;
}

LayerIter::LayerIter(const LayerMap& layers, bool include_partitions)
    : include_partitions_(include_partitions), curr_iter_(layers.begin()), end_iter_(layers.end()) {
  seekValid();
}

LayerView LayerIter::operator*() const { return LayerView(*(curr_iter_->second)); }

void LayerIter::seekValid() {
  if (curr_iter_ == end_iter_) {
    return;
  }

  if (include_partitions_) {
    return;
  }

  while (curr_iter_ != end_iter_) {
    if (curr_iter_->first.partition == 0) {
      return;
    }

    ++curr_iter_;
  }
}

LayerIter& LayerIter::operator++() {
  if (curr_iter_ != end_iter_) {
    ++curr_iter_;
  }

  seekValid();
  return *this;
}

bool LayerIter::operator==(const IterSentinel&) const { return curr_iter_ == end_iter_; }

GlobalNodeIter::GlobalNodeIter(const SceneGraph& dsg, bool include_partitions)
    : valid_(true), layers_(dsg.layers_, include_partitions) {
  setNodeIter();
}

void GlobalNodeIter::setNodeIter() {
  if (layers_ == IterSentinel()) {
    valid_ = false;
    return;
  }

  curr_node_iter_ = (*layers_).nodes();
  while (curr_node_iter_ == IterSentinel()) {
    ++layers_;
    if (layers_ == IterSentinel()) {
      valid_ = false;
      return;
    }

    curr_node_iter_ = (*layers_).nodes();
  }
}

const SceneGraphNode* GlobalNodeIter::operator*() const { return *curr_node_iter_; }

GlobalNodeIter& GlobalNodeIter::operator++() {
  ++curr_node_iter_;
  if (curr_node_iter_ == IterSentinel()) {
    ++layers_;
    setNodeIter();
  }

  return *this;
}

bool GlobalNodeIter::operator==(const IterSentinel&) {
  if (!valid_) {
    return true;
  }

  return curr_node_iter_ == IterSentinel() && layers_ == IterSentinel();
}

GlobalEdgeIter::GlobalEdgeIter(const SceneGraph& dsg, bool include_partitions)
    : include_partitions_(include_partitions),
      started_interlayer_(false),
      dsg_(dsg),
      layers_(dsg.layers_, include_partitions),
      interlayer_edge_iter_(dsg.interlayer_edges_.edges) {
  setEdgeIter();
}

const SceneGraphEdge* GlobalEdgeIter::operator*() const {
  return started_interlayer_ ? *interlayer_edge_iter_ : *curr_edge_iter_;
}

void GlobalEdgeIter::findNextValidEdge() {
  if (include_partitions_) {
    // every edge is valid if we include partitions
    return;
  }

  while (dsg_.edgeToPartition(*(*interlayer_edge_iter_)) && interlayer_edge_iter_ != IterSentinel()) {
    ++interlayer_edge_iter_;
  }
}

void GlobalEdgeIter::setEdgeIter() {
  if (started_interlayer_ || layers_ == IterSentinel()) {
    started_interlayer_ = true;
    findNextValidEdge();
    return;
  }

  curr_edge_iter_ = (*layers_).edges();

  while (curr_edge_iter_ == IterSentinel()) {
    ++layers_;
    if (layers_ == IterSentinel()) {
      started_interlayer_ = true;
      return;
    }

    curr_edge_iter_ = (*layers_).edges();
  }
}

GlobalEdgeIter& GlobalEdgeIter::operator++() {
  if (*this == IterSentinel()) {
    return *this;
  }

  if (started_interlayer_) {
    ++interlayer_edge_iter_;
    findNextValidEdge();
    return *this;
  }

  ++curr_edge_iter_;
  if (curr_edge_iter_ == IterSentinel()) {
    ++layers_;
    setEdgeIter();
  }

  return *this;
}

bool GlobalEdgeIter::operator==(const IterSentinel&) {
  if (!started_interlayer_) {
    return false;
  }

  return interlayer_edge_iter_ == IterSentinel();
}

}  // namespace spark_dsg::python
