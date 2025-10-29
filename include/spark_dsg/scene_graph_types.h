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
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

/**
 * @brief spark_dsg namespace
 */
namespace spark_dsg {

//! Node ID representation
using NodeId = uint64_t;
//! Layer ID representation
using LayerId = int64_t;
//! Partition ID within layer. 0 is reserved for the primary partition
using PartitionId = uint32_t;

//! Key type for edges
struct EdgeKey {
  EdgeKey(NodeId k1, NodeId k2);
  bool operator==(const EdgeKey& other) const;
  bool operator<(const EdgeKey& other) const;

  NodeId k1;
  NodeId k2;
};

//! Layer key specifying primary layer and optional partition
struct LayerKey {
  LayerId layer = 0;
  uint32_t partition = 0;

  LayerKey() = default;
  LayerKey(LayerId layer);
  LayerKey(LayerId layer, PartitionId partition);
  bool isParentOf(const LayerKey& other) const;
  bool operator==(const LayerKey& other) const;
  inline bool operator!=(const LayerKey& other) const {
    return !this->operator==(other);
  }
  bool operator<(const LayerKey& other) const;
};

//! @brief Common layer names
struct DsgLayers {
  //! Pre-Object node layer (static)
  inline constexpr static const char* SEGMENTS = "SEGMENTS";
  //! Object node layer (static)
  inline constexpr static const char* OBJECTS = "OBJECTS";
  //! Agents layer (dynamic)
  inline constexpr static const char* AGENTS = "AGENTS";
  //! Places node layer
  inline constexpr static const char* PLACES = "PLACES";
  //! Mesh (2D) Places node layer
  inline constexpr static const char* MESH_PLACES = "MESH_PLACES";
  //! Traversability node layer
  inline constexpr static const char* TRAVERSABILITY = "TRAVERSABILITY";
  //! Room node layer
  inline constexpr static const char* ROOMS = "ROOMS";
  //! Building node layer
  inline constexpr static const char* BUILDINGS = "BUILDINGS";

  //! Get default layer ID for each layer name
  static std::optional<LayerKey> nameToLayerId(const std::string& name);
};

namespace graph_utilities {
// TODO(nathan) make inheritance work
template <typename Graph>
struct graph_traits {};
}  // namespace graph_utilities

}  // namespace spark_dsg
