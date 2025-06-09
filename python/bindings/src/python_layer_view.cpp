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

#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <iostream>

namespace spark_dsg::python {

LayerView::LayerView(const SceneGraphLayer& layer) : id(layer.id), layer_ref_(layer) {}

NodeIter LayerView::nodes() const { return NodeIter(layer_ref_.nodes()); }

EdgeIter LayerView::edges() const { return EdgeIter(layer_ref_.edges()); }

size_t LayerView::numNodes() const { return layer_ref_.numNodes(); }

size_t LayerView::numEdges() const { return layer_ref_.numEdges(); }

bool LayerView::hasNode(NodeSymbol node_id) const { return layer_ref_.hasNode(node_id); }

bool LayerView::hasEdge(NodeSymbol source, NodeSymbol target) const { return layer_ref_.hasEdge(source, target); }

const SceneGraphNode& LayerView::getNode(NodeSymbol node_id) const { return layer_ref_.getNode(node_id); }

const SceneGraphEdge& LayerView::getEdge(NodeSymbol source, NodeSymbol target) const {
  return layer_ref_.getEdge(source, target);
}

Eigen::Vector3d LayerView::getPosition(NodeSymbol node_id) const {
  return layer_ref_.getNode(node_id).attributes().position;
}

LayerIter::LayerIter(const DynamicSceneGraph::Layers& container)
    : curr_iter_(container.begin()), end_iter_(container.end()) {}

LayerView LayerIter::operator*() const { return LayerView(*(curr_iter_->second)); }

LayerIter& LayerIter::operator++() {
  ++curr_iter_;
  return *this;
}

bool LayerIter::operator==(const IterSentinel&) const { return curr_iter_ == end_iter_; }

PartitionIter::PartitionIter(const LayerMap& container)
    : valid_(true), curr_iter_(container.begin()), end_iter_(container.end()) {
  setSubIter();
}

void PartitionIter::setSubIter() {
  if (curr_iter_ == end_iter_) {
    valid_ = false;
    return;
  }

  curr_layer_iter_ = curr_iter_->second.begin();
  end_layer_iter_ = curr_iter_->second.end();

  while (curr_layer_iter_ == end_layer_iter_) {
    ++curr_iter_;
    if (curr_iter_ == end_iter_) {
      valid_ = false;
      return;
    }

    curr_layer_iter_ = curr_iter_->second.begin();
    end_layer_iter_ = curr_iter_->second.end();
  }
}

LayerView PartitionIter::operator*() const { return LayerView(*(curr_layer_iter_->second)); }

PartitionIter& PartitionIter::operator++() {
  ++curr_layer_iter_;
  if (curr_layer_iter_ == end_layer_iter_) {
    ++curr_iter_;
    setSubIter();
  }

  return *this;
}

bool PartitionIter::operator==(const IterSentinel&) const {
  if (!valid_) {
    return true;
  }

  return curr_layer_iter_ == end_layer_iter_ && curr_iter_ == end_iter_;
}

GlobalLayerIter::GlobalLayerIter(const DynamicSceneGraph& graph, bool include_partitions)
    : include_partitions_(include_partitions), layers_(graph.layers()), partitions_(graph.layer_partitions()) {}

LayerView GlobalLayerIter::operator*() const {
  if (layers_ != IterSentinel()) {
    return *layers_;
  }

  if (include_partitions_ && partitions_ != IterSentinel()) {
    return *partitions_;
  }

  throw std::runtime_error("invalid layer iterator!");
}

GlobalLayerIter& GlobalLayerIter::operator++() {
  if (layers_ != IterSentinel()) {
    ++layers_;
  } else if (include_partitions_ && partitions_ != IterSentinel()) {
    ++partitions_;
  }

  return *this;
}

bool GlobalLayerIter::operator==(const IterSentinel&) const {
  return layers_ == IterSentinel() && (!include_partitions_ || partitions_ == IterSentinel());
}

GlobalNodeIter::GlobalNodeIter(const DynamicSceneGraph& dsg, bool include_partitions)
    : valid_(true), layers_(dsg, include_partitions) {
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

GlobalEdgeIter::GlobalEdgeIter(const DynamicSceneGraph& dsg, bool include_partitions)
    : include_partitions_(include_partitions),
      started_interlayer_(false),
      dsg_(dsg),
      layers_(dsg, include_partitions),
      interlayer_edge_iter_(dsg.interlayer_edges()) {
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
