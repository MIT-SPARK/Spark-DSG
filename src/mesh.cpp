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
#include "spark_dsg/mesh.h"

#include <unordered_map>

namespace spark_dsg {

Mesh::Mesh(bool has_colors,
           bool has_timestamps,
           bool has_labels,
           bool has_first_seen_stamps,
           bool has_fusion_counts,
           bool has_temporal_island_ids)
    : has_colors(has_colors),
      has_timestamps(has_timestamps),
      has_labels(has_labels),
      has_first_seen_stamps(has_first_seen_stamps),
      has_fusion_counts(has_fusion_counts),
      has_temporal_island_ids(has_temporal_island_ids) {}

Mesh& Mesh::operator=(const Mesh& other) {
  const_cast<bool&>(has_colors) = other.has_colors;
  const_cast<bool&>(has_timestamps) = other.has_timestamps;
  const_cast<bool&>(has_labels) = other.has_labels;
  const_cast<bool&>(has_first_seen_stamps) = other.has_first_seen_stamps;
  const_cast<bool&>(has_fusion_counts) = other.has_fusion_counts;
  const_cast<bool&>(has_temporal_island_ids) = other.has_temporal_island_ids;
  points = other.points;
  colors = other.colors;
  stamps = other.stamps;
  first_seen_stamps = other.first_seen_stamps;
  labels = other.labels;
  faces = other.faces;
  fusion_counts = other.fusion_counts;
  temporal_island_ids = other.temporal_island_ids;
  return *this;
}

Mesh& Mesh::operator=(Mesh&& other) {
  const_cast<bool&>(has_colors) = other.has_colors;
  const_cast<bool&>(has_timestamps) = other.has_timestamps;
  const_cast<bool&>(has_labels) = other.has_labels;
  const_cast<bool&>(has_first_seen_stamps) = other.has_first_seen_stamps;
  const_cast<bool&>(has_fusion_counts) = other.has_fusion_counts;
  const_cast<bool&>(has_temporal_island_ids) = other.has_temporal_island_ids;
  points = std::move(other.points);
  colors = std::move(other.colors);
  stamps = std::move(other.stamps);
  first_seen_stamps = std::move(other.first_seen_stamps);
  labels = std::move(other.labels);
  faces = std::move(other.faces);
  fusion_counts = std::move(other.fusion_counts);
  temporal_island_ids = std::move(other.temporal_island_ids);
  return *this;
}

bool Mesh::empty() const { return points.empty() && faces.empty(); }

void Mesh::clear() {
  points.clear();
  colors.clear();
  stamps.clear();
  first_seen_stamps.clear();
  labels.clear();
  faces.clear();
  fusion_counts.clear();
  temporal_island_ids.clear();
}

size_t Mesh::numVertices() const { return points.size(); }

size_t Mesh::numFaces() const { return faces.size(); }

void Mesh::reserveVertices(size_t size) {
  points.reserve(size);
  if (has_colors) {
    colors.reserve(size);
  }
  if (has_timestamps) {
    stamps.reserve(size);
  }
  if (has_labels) {
    labels.reserve(size);
  }
  if (has_first_seen_stamps) {
    first_seen_stamps.reserve(size);
  }
  if (has_fusion_counts) {
    fusion_counts.reserve(size);
  }
  if (has_temporal_island_ids) {
    temporal_island_ids.reserve(size);
  }
}

void Mesh::resizeVertices(size_t size) {
  points.resize(size);
  if (has_colors) {
    colors.resize(size);
  }
  if (has_timestamps) {
    stamps.resize(size, 0);
  }
  if (has_labels) {
    labels.resize(size, 0);
  }
  if (has_first_seen_stamps) {
    first_seen_stamps.resize(size, 0);
  }
  if (has_fusion_counts) {
    fusion_counts.resize(size, 0);
  }
  if (has_temporal_island_ids) {
    temporal_island_ids.resize(size, 0);
  }
}

void Mesh::resizeFaces(size_t size) { faces.resize(size); }

Mesh::Ptr Mesh::clone() const { return std::make_shared<Mesh>(*this); }

const Mesh::Pos& Mesh::pos(size_t index) const { return points.at(index); }

void Mesh::setPos(size_t index, const Mesh::Pos& pos) { points.at(index) = pos; }

const Color& Mesh::color(size_t index) const { return colors.at(index); }

void Mesh::setColor(size_t index, const Color& color) { colors.at(index) = color; }

Mesh::Timestamp Mesh::timestamp(size_t index) const { return stamps.at(index); }

void Mesh::setTimestamp(size_t index, Mesh::Timestamp timestamp) {
  stamps.at(index) = timestamp;
}

Mesh::Timestamp Mesh::firstSeenTimestamp(size_t index) const {
  return first_seen_stamps.at(index);
}

void Mesh::setFirstSeenTimestamp(size_t index, Mesh::Timestamp timestamp) {
  first_seen_stamps.at(index) = timestamp;
}

Mesh::Label Mesh::label(size_t index) const { return labels.at(index); }

void Mesh::setLabel(size_t index, Mesh::Label label) { labels.at(index) = label; }

Mesh::FusionCount Mesh::fusionCount(size_t index) const {
  return fusion_counts.at(index);
}

void Mesh::setFusionCount(size_t index, Mesh::FusionCount count) {
  fusion_counts.at(index) = count;
}

void Mesh::incrementFusionCount(size_t index, Mesh::FusionCount amount) {
  fusion_counts.at(index) += amount;
}

Mesh::TemporalIslandId Mesh::temporalIslandId(size_t index) const {
  return temporal_island_ids.at(index);
}

void Mesh::setTemporalIslandId(size_t index, Mesh::TemporalIslandId id) {
  temporal_island_ids.at(index) = id;
}

const Mesh::Face& Mesh::face(size_t index) const { return faces.at(index); }

Mesh::Face& Mesh::face(size_t index) { return faces.at(index); }

void Mesh::eraseVertices(const std::unordered_set<size_t>& indices) {
  // Allocating new storage is faster than erasing from all old storages.
  const auto num_new_vertices = numVertices() - indices.size();
  Mesh other(has_colors,
             has_timestamps,
             has_labels,
             has_first_seen_stamps,
             has_fusion_counts,
             has_temporal_island_ids);
  other.reserveVertices(num_new_vertices);

  // Copy over the vertices that are not being removed.
  size_t new_index = 0;
  // Map old indices to new indices.
  std::unordered_map<size_t, size_t> old_to_new;
  for (size_t old_index = 0; old_index < numVertices(); ++old_index) {
    if (indices.count(old_index)) {
      continue;
    }

    old_to_new[old_index] = new_index++;
    other.points.push_back(points[old_index]);
    if (has_colors) {
      other.colors.push_back(colors[old_index]);
    }

    if (has_timestamps) {
      other.stamps.push_back(stamps[old_index]);
    }

    if (has_labels) {
      other.labels.push_back(labels[old_index]);
    }

    if (has_first_seen_stamps) {
      other.first_seen_stamps.push_back(first_seen_stamps[old_index]);
    }

    if (has_fusion_counts) {
      other.fusion_counts.push_back(fusion_counts[old_index]);
    }

    if (has_temporal_island_ids) {
      other.temporal_island_ids.push_back(temporal_island_ids[old_index]);
    }
  }

  // Update and copy over the faces.
  for (const auto& face : faces) {
    Face new_face;
    bool erase_face = false;
    for (size_t i = 0; i < 3; ++i) {
      const auto iter = old_to_new.find(face[i]);
      if (iter == old_to_new.end()) {
        erase_face = true;
        break;
      }

      new_face[i] = iter->second;
    }

    if (erase_face) {
      continue;
    }

    other.faces.push_back(new_face);
  }

  *this = std::move(other);
}

void Mesh::eraseFaces(const std::unordered_set<size_t>& indices,
                      const bool update_vertices) {
  Faces new_faces;
  new_faces.reserve(numFaces());
  for (size_t old_index = 0; old_index < numFaces(); ++old_index) {
    if (!indices.count(old_index)) {
      new_faces.push_back(faces[old_index]);
    }
  }
  faces = std::move(new_faces);
  faces.resize(faces.size());

  if (!update_vertices) {
    return;
  }

  std::unordered_set<size_t> unused_vertices;
  unused_vertices.reserve(numVertices());
  for (size_t i = 0; i < numVertices(); ++i) {
    unused_vertices.insert(i);
  }
  for (const auto& face : faces) {
    for (const auto index : face) {
      unused_vertices.erase(index);
    }
  }
  eraseVertices(unused_vertices);
}

void Mesh::transform(const Eigen::Isometry3f& transform) {
  for (auto& point : points) {
    point = transform * point;
  }
}

bool Mesh::append(const Mesh& other) {
  if (has_colors != other.has_colors && has_timestamps != other.has_timestamps &&
      has_labels != other.has_labels &&
      has_first_seen_stamps != other.has_first_seen_stamps &&
      has_fusion_counts != other.has_fusion_counts &&
      has_temporal_island_ids != other.has_temporal_island_ids) {
    return false;
  }

  size_t offset = points.size();
  points.insert(points.end(), other.points.begin(), other.points.end());
  colors.insert(colors.end(), other.colors.begin(), other.colors.end());
  stamps.insert(stamps.end(), other.stamps.begin(), other.stamps.end());
  first_seen_stamps.insert(first_seen_stamps.end(),
                           other.first_seen_stamps.begin(),
                           other.first_seen_stamps.end());
  labels.insert(labels.end(), other.labels.begin(), other.labels.end());
  fusion_counts.insert(fusion_counts.end(),
                       other.fusion_counts.begin(),
                       other.fusion_counts.end());
  temporal_island_ids.insert(temporal_island_ids.end(),
                             other.temporal_island_ids.begin(),
                             other.temporal_island_ids.end());

  for (const auto& face : other.faces) {
    faces.push_back({face[0] + offset, face[1] + offset, face[2] + offset});
  }

  return true;
}

Mesh& Mesh::operator+=(const Mesh& other) {
  append(other);
  return *this;
}

/**
 * @brief Get memory usage of the mesh in bytes.
 */
size_t Mesh::memoryUsage() const {
  // Static parts.
  size_t total_bytes = sizeof(Mesh);
  total_bytes += sizeof(Pos) * points.size();
  total_bytes += sizeof(Color) * colors.size();
  total_bytes += sizeof(Timestamp) * stamps.size();
  total_bytes += sizeof(Timestamp) * first_seen_stamps.size();
  total_bytes += sizeof(Label) * labels.size();
  total_bytes += sizeof(Face) * faces.size();
  total_bytes += sizeof(FusionCount) * fusion_counts.size();
  total_bytes += sizeof(TemporalIslandId) * temporal_island_ids.size();
  return total_bytes;
}

bool operator==(const Mesh& lhs, const Mesh& rhs) {
  return lhs.has_colors == rhs.has_colors && lhs.has_timestamps == rhs.has_timestamps &&
         lhs.has_labels == rhs.has_labels &&
         lhs.has_first_seen_stamps == rhs.has_first_seen_stamps &&
         lhs.has_fusion_counts == rhs.has_fusion_counts &&
         lhs.has_temporal_island_ids == rhs.has_temporal_island_ids &&
         lhs.points == rhs.points && lhs.colors == rhs.colors && lhs.stamps == rhs.stamps &&
         lhs.first_seen_stamps == rhs.first_seen_stamps && lhs.labels == rhs.labels &&
         lhs.faces == rhs.faces && lhs.fusion_counts == rhs.fusion_counts &&
         lhs.temporal_island_ids == rhs.temporal_island_ids; 
}

}  // namespace spark_dsg
