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
#include <ostream>

#include "spark_dsg/scene_graph_types.h"

/**
 * @brief spark_dsg namespace
 */
namespace spark_dsg {

struct LayerKey {
  LayerId layer;
  uint32_t prefix = 0;
  bool dynamic = false;

  LayerKey();

  LayerKey(LayerId layer_id);

  LayerKey(LayerId layer_id, uint32_t prefix);

  bool isParent(const LayerKey& other) const;

  bool valid() const;

  bool operator==(const LayerKey& other) const;

  inline bool operator!=(const LayerKey& other) const {
    return !this->operator==(other);
  }
};

std::ostream& operator<<(std::ostream& out, const LayerKey& key);

class LayerPrefix {
 public:
  LayerPrefix(char key);

  LayerPrefix(char key, uint32_t index);

  LayerPrefix(uint32_t index);

  static LayerPrefix fromId(NodeId node);

  inline operator uint32_t() const { return value_.value; }

  std::string str(bool with_key = true) const;

  bool matches(NodeId node) const;

  NodeId makeId(size_t index) const;

  size_t index(NodeId node_id) const;

  inline char key() const { return value_.symbol.key; }

  inline uint32_t index() const { return value_.symbol.index; }

 private:
  union {
    uint32_t value;
    struct __attribute__((packed)) {
      uint32_t index : 24;
      char key : 8;
    } symbol;
  } value_;
};

std::ostream& operator<<(std::ostream& out, const LayerKey& key);

}  // namespace spark_dsg
