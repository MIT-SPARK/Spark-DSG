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

namespace spark_dsg {

Mesh::Mesh(bool has_colors, bool has_timestamps, bool has_labels)
    : has_colors(has_colors), has_timestamps(has_timestamps), has_labels(has_labels) {}

Mesh::~Mesh() {}

bool Mesh::empty() const { return points.empty() && faces.empty(); }

size_t Mesh::numVertices() const { return points.size(); }

size_t Mesh::numFaces() const { return faces.size(); }

void Mesh::resizeVertices(size_t size) {
  points.resize(size);
  if (has_colors) {
    colors.resize(size);
  }
  if (has_timestamps) {
    stamps.resize(size);
  }
  if (has_labels) {
    labels.resize(size);
  }

  first_seen_stamps.resize(size);
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

const Mesh::Face& Mesh::face(size_t index) const { return faces.at(index); }

Mesh::Face& Mesh::face(size_t index) { return faces.at(index); }

}  // namespace spark_dsg
