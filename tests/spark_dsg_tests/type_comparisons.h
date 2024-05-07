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
#include <spark_dsg/node_attributes.h>

namespace spark_dsg {
namespace test {

template <typename Scalar>
bool quaternionsEqual(const Eigen::Quaternion<Scalar>& lhs,
                      const Eigen::Quaternion<Scalar>& rhs) {
  return lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() &&
         lhs.z() == rhs.z();
}

}  // namespace test

inline bool operator==(const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
  return lhs.id == rhs.id && lhs.layer == rhs.layer &&
         lhs.attributes() == rhs.attributes() && lhs.timestamp == rhs.timestamp;
}

inline bool operator!=(const SceneGraphNode& lhs, const SceneGraphNode& rhs) {
  return !(lhs == rhs);
}

inline bool operator==(const SceneGraphEdge& lhs, const SceneGraphEdge& rhs) {
  return lhs.source == rhs.source && lhs.target == rhs.target &&
         lhs.attributes() == rhs.attributes();
}

inline bool operator!=(const SceneGraphEdge& lhs, const SceneGraphEdge& rhs) {
  return !(lhs == rhs);
}

inline bool isSubset(const std::map<EdgeKey, SceneGraphEdge>& lhs,
                     const std::map<EdgeKey, SceneGraphEdge>& rhs) {
  for (const auto& [key, edge] : lhs) {
    auto iter = rhs.find(key);
    if (iter == rhs.end()) {
      return false;
    }

    if (iter->second != edge) {
      return false;
    }
  }

  return true;
}

inline bool isSubset(const SceneGraphLayer& lhs, const SceneGraphLayer& rhs) {
  for (const auto& [node_id, node] : lhs.nodes()) {
    const auto rhs_node = rhs.findNode(node_id);
    if (!rhs_node) {
      return false;
    }

    if (*rhs_node != *node) {
      return false;
    }
  }

  return isSubset(lhs.edges(), rhs.edges());
}

inline bool isSubset(const DynamicSceneGraphLayer& lhs,
                     const DynamicSceneGraphLayer& rhs) {
  for (const auto& node : lhs.nodes()) {
    if (!node) {
      continue;
    }

    const auto rhs_node = rhs.findNode(node->id);
    if (!rhs_node) {
      return false;
    }

    if (*rhs_node != *node) {
      return false;
    }
  }

  return isSubset(lhs.edges(), rhs.edges());
}

inline bool operator==(const DynamicSceneGraph& lhs, const DynamicSceneGraph& rhs) {
  for (const auto& [layer_id, layer] : lhs.layers()) {
    if (!rhs.hasLayer(layer_id)) {
      return false;
    }

    const auto& rhs_layer = rhs.getLayer(layer_id);
    const auto layers_equal =
        isSubset(*layer, rhs_layer) && isSubset(rhs_layer, *layer);
    if (!layers_equal) {
      return false;
    }
  }

  for (const auto& [layer_id, layer_group] : lhs.dynamicLayers()) {
    for (const auto& [prefix, layer] : layer_group) {
      if (!rhs.hasLayer(layer_id, prefix)) {
        return false;
      }

      const auto& rhs_layer = rhs.getLayer(layer_id, prefix);
      const auto layers_equal =
          isSubset(*layer, rhs_layer) && isSubset(rhs_layer, *layer);
      if (!layers_equal) {
        return false;
      }
    }
  }

  if (!(isSubset(lhs.interlayer_edges(), rhs.interlayer_edges()) &&
        isSubset(rhs.interlayer_edges(), lhs.interlayer_edges()))) {
    return false;
  }

  if (!(isSubset(lhs.dynamic_interlayer_edges(), rhs.dynamic_interlayer_edges()) &&
        isSubset(rhs.dynamic_interlayer_edges(), lhs.dynamic_interlayer_edges()))) {
    return false;
  }

  const auto lhs_mesh = lhs.mesh();
  const auto rhs_mesh = rhs.mesh();
  if (!lhs_mesh && !rhs_mesh) {
    return true;
  }

  if (!lhs_mesh || !rhs_mesh) {
    return false;
  }

  return *lhs_mesh == *rhs_mesh;
}

}  // namespace spark_dsg
