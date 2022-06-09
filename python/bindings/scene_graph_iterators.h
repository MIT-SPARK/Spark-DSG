#pragma once
#include <spark_dsg/dynamic_scene_graph.h>

using namespace spark_dsg;

struct IterSentinel {};

class NodeIter {
 public:
  NodeIter(const SceneGraphLayer::Nodes& container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  const SceneGraphLayer::Node* operator*() const { return curr_iter_->second.get(); }

  NodeIter& operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

 private:
  typename SceneGraphLayer::Nodes::const_iterator curr_iter_;
  typename SceneGraphLayer::Nodes::const_iterator end_iter_;
};

class DynamicNodeIter {
 public:
  DynamicNodeIter(const DynamicSceneGraphLayer::Nodes& container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  const DynamicSceneGraphLayer::Node* operator*() const { return curr_iter_->get(); }

  DynamicNodeIter& operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

 private:
  typename DynamicSceneGraphLayer::Nodes::const_iterator curr_iter_;
  typename DynamicSceneGraphLayer::Nodes::const_iterator end_iter_;
};

class EdgeIter {
 public:
  EdgeIter(const SceneGraphLayer::Edges& container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  const SceneGraphLayer::Edge* operator*() const { return &(curr_iter_->second); }

  EdgeIter& operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

 private:
  typename SceneGraphLayer::Edges::const_iterator curr_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_iter_;
};

class GlobalNodeIter {
 public:
  GlobalNodeIter(const DynamicSceneGraph& dsg) : valid_(true) {
    curr_layer_iter_ = dsg.layers().begin();
    end_layer_iter_ = dsg.layers().end();

    setNodeIter();
  }

  void setNodeIter() {
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

  const SceneGraphNode& operator*() const { return *curr_node_iter_->second; }

  GlobalNodeIter& operator++() {
    ++curr_node_iter_;
    if (curr_node_iter_ == end_node_iter_) {
      ++curr_layer_iter_;
      setNodeIter();
    }

    return *this;
  }

  bool operator==(const IterSentinel&) {
    if (!valid_) {
      return true;
    }

    return curr_node_iter_ == end_node_iter_ && curr_layer_iter_ == end_layer_iter_;
  }

 private:
  bool valid_;
  typename SceneGraphLayer::Nodes::const_iterator curr_node_iter_;
  typename SceneGraphLayer::Nodes::const_iterator end_node_iter_;
  typename DynamicSceneGraph::Layers::const_iterator curr_layer_iter_;
  typename DynamicSceneGraph::Layers::const_iterator end_layer_iter_;
};

class GlobalEdgeIter {
 public:
  GlobalEdgeIter(const DynamicSceneGraph& dsg) : started_interlayer_(false) {
    curr_layer_iter_ = dsg.layers().begin();
    end_layer_iter_ = dsg.layers().end();

    curr_interlayer_iter_ = dsg.interlayer_edges().begin();
    end_interlayer_iter_ = dsg.interlayer_edges().end();

    setEdgeIter();
  }

  const SceneGraphEdge* operator*() const {
    if (started_interlayer_) {
      return &curr_interlayer_iter_->second;
    } else {
      return &curr_edge_iter_->second;
    }
  }

  void setEdgeIter() {
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

  GlobalEdgeIter& operator++() {
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

  bool operator==(const IterSentinel&) {
    if (!started_interlayer_) {
      return false;
    }

    return curr_interlayer_iter_ == end_interlayer_iter_;
  }

 private:
  bool started_interlayer_;
  typename SceneGraphLayer::Edges::const_iterator curr_edge_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_edge_iter_;
  typename DynamicSceneGraph::Layers::const_iterator curr_layer_iter_;
  typename DynamicSceneGraph::Layers::const_iterator end_layer_iter_;
  typename SceneGraphLayer::Edges::const_iterator curr_interlayer_iter_;
  typename SceneGraphLayer::Edges::const_iterator end_interlayer_iter_;
};

class LayerView {
 public:
  LayerView(const SceneGraphLayer& layer) : id(layer.id), layer_ref_(layer) {}

  NodeIter nodes() const { return NodeIter(layer_ref_.nodes()); }

  EdgeIter edges() const { return EdgeIter(layer_ref_.edges()); }

  size_t numNodes() const { return layer_ref_.numNodes(); }

  size_t numEdges() const { return layer_ref_.numEdges(); }

  const LayerId id;

 private:
  const SceneGraphLayer& layer_ref_;
};

class LayerIter {
 public:
  LayerIter(const DynamicSceneGraph::Layers& container)
      : curr_iter_(container.begin()), end_iter_(container.end()) {}

  LayerView operator*() const { return LayerView(*(curr_iter_->second)); }

  LayerIter& operator++() {
    ++curr_iter_;
    return *this;
  }

  bool operator==(const IterSentinel&) { return curr_iter_ == end_iter_; }

 private:
  typename DynamicSceneGraph::Layers::const_iterator curr_iter_;
  typename DynamicSceneGraph::Layers::const_iterator end_iter_;
};

class DynamicLayerView {
 public:
  DynamicLayerView(const DynamicSceneGraphLayer& layer)
      : id(layer.id), prefix(layer.prefix), layer_ref_(layer) {}

  DynamicNodeIter nodes() const { return DynamicNodeIter(layer_ref_.nodes()); }

  EdgeIter edges() const { return EdgeIter(layer_ref_.edges()); }

  size_t numNodes() const { return layer_ref_.numNodes(); }

  size_t numEdges() const { return layer_ref_.numEdges(); }

  const LayerId id;

  const LayerPrefix prefix;

 private:
  const DynamicSceneGraphLayer& layer_ref_;
};

class DynamicLayerIter {
 public:
  using LayerMap = std::map<LayerId, DynamicSceneGraph::DynamicLayers>;

  DynamicLayerIter(const LayerMap& container)
      : valid_(true), curr_iter_(container.begin()), end_iter_(container.end()) {
    setSubIter();
  }

  void setSubIter() {
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

  DynamicLayerView operator*() const { return DynamicLayerView(*(curr_layer_iter_->second)); }

  DynamicLayerIter& operator++() {
    ++curr_layer_iter_;
    if (curr_layer_iter_ == end_layer_iter_) {
      ++curr_iter_;
      setSubIter();
    }

    return *this;
  }

  bool operator==(const IterSentinel&) {
    if (!valid_) {
      return true;
    }

    return curr_layer_iter_ == end_layer_iter_ && curr_iter_ == end_iter_;
  }

 private:
  bool valid_;
  typename LayerMap::const_iterator curr_iter_;
  typename LayerMap::const_iterator end_iter_;
  typename DynamicSceneGraph::DynamicLayers::const_iterator curr_layer_iter_;
  typename DynamicSceneGraph::DynamicLayers::const_iterator end_layer_iter_;
};
