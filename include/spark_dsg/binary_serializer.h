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
#include "spark_dsg/attribute_factory.h"
#include "spark_dsg/binary_serialization_utils.h"
#include "spark_dsg/dynamic_scene_graph.h"

namespace spark_dsg {

using MeshVertices = DynamicSceneGraph::MeshVertices;
using MeshFaces = DynamicSceneGraph::MeshFaces;

namespace serialization {

#define THROW_SERIALIZATION_ERROR(msg)                     \
  std::stringstream ss;                                    \
  ss << "[" << __FILE__ << ":" << __LINE__ << "] " << msg; \
  throw std::domain_error(ss.str())

struct BinarySerializer {
  explicit BinarySerializer(std::vector<uint8_t>* buffer);

  void writeArrayStart();

  void writeArrayEnd();

  void startFixedArray(size_t length);

  void write_type(PackType type);

  template <typename T>
  void write(const T& value) {
    ::spark_dsg::serialization::write_binary(*this, value);
  }

  std::vector<uint8_t>* ref;
};

struct BinaryDeserializer {
  BinaryDeserializer(const uint8_t* const buffer, size_t length);

  explicit BinaryDeserializer(const std::vector<uint8_t>& buffer);

  void checkType(PackType type) const;

  void checkDynamicArray() const;

  bool isDynamicArrayEnd() const;

  void checkFixedArrayLength(size_t length) const;

  size_t readFixedArrayLength() const;

  inline PackType getCurrType() const {
    if (pos >= buffer_length) {
      throw std::out_of_range("attempt to read past end of buffer");
    }

    return static_cast<PackType>(ref[pos]);
  }

  template <typename T>
  void read(T& value) const {
    pos += ::spark_dsg::serialization::read_binary(*this, value);
  }

  template <typename T>
  const T* getReadPtr(size_t num_elements = 1) const {
    const size_t end_pos = sizeof(T) * num_elements + pos;
    if (end_pos > buffer_length) {
      throw std::out_of_range("attempt to read past end of buffer");
    }

    return reinterpret_cast<const T*>(ref + pos);
  }

  const uint8_t* const ref;
  const size_t buffer_length;
  mutable size_t pos;
};

struct BinaryConverter {
  explicit BinaryConverter(BinarySerializer* serializer);

  explicit BinaryConverter(const BinaryDeserializer* deserializer);

  ~BinaryConverter();

  void finalize() const;

  template <typename Factory>
  std::string read_type(const Factory& factory) const {
    uint8_t type_index;
    deserializer_->read(type_index);
    return factory.lookupIndexName(type_index);
  }

  template <typename Factory>
  void mark_type(const Factory& factory, const std::string& name) {
    serializer_->write(factory.lookupNameIndex(name));
  }

  template <typename T>
  void write(const std::string&, const T& value) {
    serializer_->write(value);
  }

  template <typename T>
  void read(const std::string&, T& value) const {
    deserializer_->read(value);
  }

 private:
  BinarySerializer* serializer_ = nullptr;
  const BinaryDeserializer* deserializer_ = nullptr;
};

using BinaryNodeFactory = NodeAttributeFactory<BinaryConverter>;
using BinaryEdgeFactory = EdgeAttributeFactory<BinaryConverter>;

template <>
void BinarySerializer::write<NodeAttributes>(const NodeAttributes& attrs);

template <>
void BinarySerializer::write<EdgeAttributes>(const EdgeAttributes& attrs);

template <>
void BinarySerializer::write<SceneGraphNode>(const SceneGraphNode& node);

template <>
void BinarySerializer::write<DynamicSceneGraphNode>(const DynamicSceneGraphNode& node);

template <>
void BinarySerializer::write<SceneGraphEdge>(const SceneGraphEdge& edge);

template <>
void BinarySerializer::write<MeshEdge>(const MeshEdge& edge);

template <>
void BinarySerializer::write<MeshVertices>(const MeshVertices& vertices);

template <>
void BinarySerializer::write<MeshFaces>(const MeshFaces& vertices);

}  // namespace serialization
}  // namespace spark_dsg
