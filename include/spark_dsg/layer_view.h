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
#include "spark_dsg/scene_graph_layer.h"

namespace spark_dsg {

/**
 * @brief class that exposes an const iterator (with optional filter) over nodes
 *
 * Note that this assumes that no modifications are made to the underlying layer,
 * as changes to the underlying layer would invalidate the internal iterators that
 * LayerView uses.
 */
struct LayerView {
  using Ptr = std::unique_ptr<LayerView>;
  using Filter = std::function<bool(const SceneGraphNode&)>;

  struct Iter {
    friend struct LayerView;
    using NodeIter = SceneGraphLayer::Nodes::const_iterator;
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = const SceneGraphNode*;
    using pointer = const SceneGraphNode*;
    using reference = const SceneGraphNode&;

    ~Iter() = default;
    Iter(const Iter& other);
    Iter& operator=(const Iter& other);
    Iter& operator++();
    Iter operator++(int);
    reference operator*() const;
    pointer operator->() const;
    friend bool operator==(const Iter& lhs, const Iter& rhs);

   private:
    Iter(const SceneGraphLayer* layer, const Filter& filter, bool is_end);
    bool valid() const;
    void next();

    // note: pointer mostly for convenience
    const SceneGraphLayer* layer_;
    SceneGraphLayer::Nodes::const_iterator iter_;
    std::function<bool(const SceneGraphNode&)> filter_;
  };

  LayerView();
  LayerView(const SceneGraphLayer& layer, const Filter& filter = Filter());
  Iter begin() const;
  Iter end() const;

 private:
  const SceneGraphLayer* layer_;
  Filter filter_;
};

bool operator==(const LayerView::Iter& lhs, const LayerView::Iter& rhs);

inline bool operator!=(const LayerView::Iter& lhs, const LayerView::Iter& rhs) {
  return !(lhs == rhs);
}

}  // namespace spark_dsg
