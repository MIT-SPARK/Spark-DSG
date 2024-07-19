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
#include "spark_dsg/python/scene_graph_iterators.h"

namespace spark_dsg::python {

NodeIter::NodeIter(const SceneGraphLayer::Nodes& container)
    : curr_iter_(container.begin()), end_iter_(container.end()) {}

const SceneGraphNode* NodeIter::operator*() const { return curr_iter_->second.get(); }

NodeIter& NodeIter::operator++() {
  ++curr_iter_;
  return *this;
}

bool NodeIter::operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

DynamicNodeIter::DynamicNodeIter(const DynamicSceneGraphLayer::Nodes& container)
    : curr_iter_(container.begin()), end_iter_(container.end()) {
  while (*curr_iter_ == nullptr && curr_iter_ != end_iter_) {
    ++curr_iter_;
  }
}

const SceneGraphNode* DynamicNodeIter::operator*() const { return curr_iter_->get(); }

DynamicNodeIter& DynamicNodeIter::operator++() {
  ++curr_iter_;
  while (*curr_iter_ == nullptr && curr_iter_ != end_iter_) {
    ++curr_iter_;
  }
  return *this;
}

bool DynamicNodeIter::operator==(const IterSentinel&) {
  return curr_iter_ == end_iter_;
}

EdgeIter::EdgeIter(const SceneGraphLayer::Edges& container)
    : curr_iter_(container.begin()), end_iter_(container.end()) {}

const SceneGraphEdge* EdgeIter::operator*() const { return &(curr_iter_->second); }

EdgeIter& EdgeIter::operator++() {
  ++curr_iter_;
  return *this;
}

bool EdgeIter::operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

GlobalNodeIter::GlobalNodeIter(const DynamicSceneGraph& dsg) : valid_(true) {
  curr_layer_iter_ = dsg.layers().begin();
  end_layer_iter_ = dsg.layers().end();

  setNodeIter();
}

void GlobalNodeIter::setNodeIter() {
  if (curr_layer_iter_ == end_layer_iter_) {
    valid_ = false;
    return;
  }

  curr_node_iter_ = curr_layer_iter_->second->nodes().begin();
  end_node_iter_ = curr_layer_iter_->second->nodes().end();
  while (curr_node_iter_ == end_node_iter_) {
    ++curr_layer_iter_;
    if (curr_layer_iter_ == end_layer_iter_) {
      valid_ = false;
      return;
    }

    curr_node_iter_ = curr_layer_iter_->second->nodes().begin();
    end_node_iter_ = curr_layer_iter_->second->nodes().end();
  }
}

const SceneGraphNode& GlobalNodeIter::operator*() const {
  return *curr_node_iter_->second;
}

GlobalNodeIter& GlobalNodeIter::operator++() {
  ++curr_node_iter_;
  if (curr_node_iter_ == end_node_iter_) {
    ++curr_layer_iter_;
    setNodeIter();
  }

  return *this;
}

bool GlobalNodeIter::operator==(const IterSentinel&) {
  if (!valid_) {
    return true;
  }

  return curr_node_iter_ == end_node_iter_ && curr_layer_iter_ == end_layer_iter_;
}

GlobalEdgeIter::GlobalEdgeIter(const DynamicSceneGraph& dsg)
    : started_interlayer_(false) {
  curr_layer_iter_ = dsg.layers().begin();
  end_layer_iter_ = dsg.layers().end();

  curr_interlayer_iter_ = dsg.interlayer_edges().begin();
  end_interlayer_iter_ = dsg.interlayer_edges().end();

  setEdgeIter();
}

const SceneGraphEdge* GlobalEdgeIter::operator*() const {
  if (started_interlayer_) {
    return &curr_interlayer_iter_->second;
  } else {
    return &curr_edge_iter_->second;
  }
}

void GlobalEdgeIter::setEdgeIter() {
  if (started_interlayer_ || curr_layer_iter_ == end_layer_iter_) {
    started_interlayer_ = true;
    return;
  }

  curr_edge_iter_ = curr_layer_iter_->second->edges().begin();
  end_edge_iter_ = curr_layer_iter_->second->edges().end();

  while (curr_edge_iter_ == end_edge_iter_) {
    ++curr_layer_iter_;
    if (curr_layer_iter_ == end_layer_iter_) {
      started_interlayer_ = true;
      return;
    }

    curr_edge_iter_ = curr_layer_iter_->second->edges().begin();
    end_edge_iter_ = curr_layer_iter_->second->edges().end();
  }
}

GlobalEdgeIter& GlobalEdgeIter::operator++() {
  if (started_interlayer_) {
    ++curr_interlayer_iter_;
    return *this;
  }

  ++curr_edge_iter_;
  if (curr_edge_iter_ == end_edge_iter_) {
    ++curr_layer_iter_;
    setEdgeIter();
  }

  return *this;
}

bool GlobalEdgeIter::operator==(const IterSentinel&) {
  if (!started_interlayer_) {
    return false;
  }

  return curr_interlayer_iter_ == end_interlayer_iter_;
}

}  // namespace spark_dsg::python
