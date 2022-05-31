#pragma once
#include "kimera_dsg/attribute_factory.h"
#include "kimera_dsg/binary_serialization_utils.h"
#include "kimera_dsg/dynamic_scene_graph.h"

#include <glog/logging.h>

namespace kimera {
namespace serialization {

struct BinarySerializer {
  explicit BinarySerializer(std::vector<uint8_t>* buffer);

  void writeArrayStart();

  void writeArrayEnd();

  void startFixedArray(size_t length);

  void write_type(PackType type);

  template <typename T>
  void write(const T& value) {
    ::kimera::serialization::write_binary(*this, value);
  }

  std::vector<uint8_t>* ref;
};

struct BinaryDeserializer {
  explicit BinaryDeserializer(const std::vector<uint8_t>* buffer);

  void checkType(PackType type) const;

  void checkDynamicArray() const;

  bool isDynamicArrayEnd() const;

  void checkFixedArrayLength(size_t length) const;

  size_t readFixedArrayLength() const;

  template <typename T>
  void read(T& value) const {
    pos += ::kimera::serialization::read_binary(*this, value);
  }

  template <typename T>
  const T* getReadPtr() const {
    return reinterpret_cast<const T*>(ref->data() + pos);
  }

  const std::vector<uint8_t>* ref;
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

}  // namespace serialization
}  // namespace kimera
