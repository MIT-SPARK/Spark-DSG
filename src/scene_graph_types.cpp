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
#include "spark_dsg/scene_graph_types.h"

#include <algorithm>

namespace spark_dsg {

EdgeKey::EdgeKey(NodeId k1, NodeId k2) : k1(std::min(k1, k2)), k2(std::max(k1, k2)) {}

bool EdgeKey::operator==(const EdgeKey& other) const {
  return k1 == other.k1 && k2 == other.k2;
}

bool EdgeKey::operator<(const EdgeKey& other) const {
  if (k1 == other.k1) {
    return k2 < other.k2;
  }

  return k1 < other.k1;
}

LayerKey::LayerKey(LayerId layer_id) : LayerKey(layer_id, 0) {}

LayerKey::LayerKey(LayerId layer_id, PartitionId partition)
    : layer(layer_id), partition(partition) {}

bool LayerKey::isParentOf(const LayerKey& other) const { return layer > other.layer; }

bool LayerKey::operator==(const LayerKey& other) const {
  return layer == other.layer && partition == other.partition;
}

bool LayerKey::operator<(const LayerKey& other) const {
  if (layer == other.layer) {
    return partition < other.partition;
  }

  return layer < other.layer;
}

std::optional<LayerKey> DsgLayers::nameToLayerId(const std::string& name) {
  if (name == DsgLayers::SEGMENTS) {
    return 1;
  } else if (name == DsgLayers::OBJECTS) {
    return 2;
  } else if (name == DsgLayers::AGENTS) {
    return 2;
  } else if (name == DsgLayers::PLACES) {
    return 3;
  } else if (name == DsgLayers::MESH_PLACES) {
    return LayerKey{3, 1};
  } else if (name == DsgLayers::ROOMS) {
    return 4;
  } else if (name == DsgLayers::BUILDINGS) {
    return 5;
  } else {
    return std::nullopt;
  }
}

}  // namespace spark_dsg
