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

namespace spark_dsg::python {

LayerView::LayerView(const SceneGraphLayer& layer) : id(layer.id), layer_ref_(layer) {}

NodeIter LayerView::nodes() const { return NodeIter(layer_ref_.nodes()); }

EdgeIter LayerView::edges() const { return EdgeIter(layer_ref_.edges()); }

size_t LayerView::numNodes() const { return layer_ref_.numNodes(); }

size_t LayerView::numEdges() const { return layer_ref_.numEdges(); }

bool LayerView::hasNode(NodeSymbol node_id) const { return layer_ref_.hasNode(node_id); }

bool LayerView::hasEdge(NodeSymbol source, NodeSymbol target) const {
  return layer_ref_.hasEdge(source, target);
}

const SceneGraphNode& LayerView::getNode(NodeSymbol node_id) const {
  return layer_ref_.getNode(node_id);
}

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

bool LayerIter::operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

DynamicLayerView::DynamicLayerView(const DynamicSceneGraphLayer& layer)
    : id(layer.id), prefix(layer.prefix), layer_ref_(layer) {}

DynamicNodeIter DynamicLayerView::nodes() const {
  return DynamicNodeIter(layer_ref_.nodes());
}

EdgeIter DynamicLayerView::edges() const { return EdgeIter(layer_ref_.edges()); }

size_t DynamicLayerView::numNodes() const { return layer_ref_.numNodes(); }

size_t DynamicLayerView::numEdges() const { return layer_ref_.numEdges(); }

DynamicLayerIter::DynamicLayerIter(const LayerMap& container)
    : valid_(true), curr_iter_(container.begin()), end_iter_(container.end()) {
  setSubIter();
}

void DynamicLayerIter::setSubIter() {
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

DynamicLayerView DynamicLayerIter::operator*() const {
  return DynamicLayerView(*(curr_layer_iter_->second));
}

DynamicLayerIter& DynamicLayerIter::operator++() {
  ++curr_layer_iter_;
  if (curr_layer_iter_ == end_layer_iter_) {
    ++curr_iter_;
    setSubIter();
  }

  return *this;
}

bool DynamicLayerIter::operator==(const IterSentinel&) {
  if (!valid_) {
    return true;
  }

  return curr_layer_iter_ == end_layer_iter_ && curr_iter_ == end_iter_;
}

}  // namespace spark_dsg::python
