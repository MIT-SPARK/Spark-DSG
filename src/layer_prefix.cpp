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
#include "spark_dsg/layer_prefix.h"

#include <algorithm>
#include <sstream>

namespace spark_dsg {

LayerKey::LayerKey() : layer(DsgLayers::UNKNOWN) {}

LayerKey::LayerKey(LayerId layer_id) : layer(layer_id) {}

LayerKey::LayerKey(LayerId layer_id, uint32_t prefix)
    : layer(layer_id), prefix(prefix), dynamic(true) {}

bool LayerKey::isParent(const LayerKey& other) const { return layer > other.layer; }

bool LayerKey::valid() const { return layer != DsgLayers::UNKNOWN; }

bool LayerKey::operator==(const LayerKey& other) const {
  if (dynamic != other.dynamic) {
    return false;
  }

  const bool same_layer = layer == other.layer;
  if (!dynamic && same_layer) {
    return true;
  }

  return same_layer && prefix == other.prefix;
}

std::ostream& operator<<(std::ostream& out, const LayerKey& key) {
  if (key.dynamic) {
    out << key.layer << "(" << key.prefix << ")";
  } else {
    out << key.layer;
  }
  return out;
}

LayerPrefix::LayerPrefix(char key) {
  value_.symbol.key = key;
  value_.symbol.index = 0;
}

LayerPrefix::LayerPrefix(char key, uint32_t index) {
  value_.symbol.key = key;
  value_.symbol.index = index;
}

LayerPrefix::LayerPrefix(uint32_t index) { value_.value = index; }

LayerPrefix LayerPrefix::fromId(NodeId node_id) {
  // grab the 32 msb portion of the ID
  return LayerPrefix(static_cast<uint32_t>(node_id >> 32));
}

std::string LayerPrefix::str(bool with_key) const {
  if (!with_key) {
    return std::to_string(value_.value);
  }

  std::stringstream ss;
  ss << value_.symbol.key;
  if (value_.symbol.index) {
    ss << "(" << value_.symbol.index << ")";
  }

  return ss.str();
}

bool LayerPrefix::matches(NodeId node) const {
  return value_.value == static_cast<uint32_t>(node >> 32);
}

NodeId LayerPrefix::makeId(size_t index) const {
  return (static_cast<NodeId>(value_.value) << 32) + index;
}

size_t LayerPrefix::index(NodeId node_id) const {
  // grab the 32 lsb portion of the ID
  return 0xFFFF'FFFF & node_id;
}

}  // namespace spark_dsg
