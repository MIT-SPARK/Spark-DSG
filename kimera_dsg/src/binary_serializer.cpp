#include "kimera_dsg/binary_serializer.h"

#include <glog/logging.h>

namespace kimera {
namespace serialization {

#define SHOW_CASE(os, enum_value) \
  case enum_value:                \
    os << #enum_value;            \
    break

std::ostream& operator<<(std::ostream& out, PackType type) {
  switch (type) {
    SHOW_CASE(out, PackType::NIL);
    SHOW_CASE(out, PackType::FALSE);
    SHOW_CASE(out, PackType::TRUE);
    SHOW_CASE(out, PackType::FLOAT32);
    SHOW_CASE(out, PackType::FLOAT64);
    SHOW_CASE(out, PackType::UINT8);
    SHOW_CASE(out, PackType::UINT16);
    SHOW_CASE(out, PackType::UINT32);
    SHOW_CASE(out, PackType::UINT64);
    SHOW_CASE(out, PackType::INT8);
    SHOW_CASE(out, PackType::INT16);
    SHOW_CASE(out, PackType::INT32);
    SHOW_CASE(out, PackType::INT64);
    SHOW_CASE(out, PackType::STR32);
    SHOW_CASE(out, PackType::ARR32);
    SHOW_CASE(out, PackType::ARRXX);
    SHOW_CASE(out, PackType::ARRYY);
    default:
      out << "INVALID";
      break;
  }
  return out;
}

BinarySerializer::BinarySerializer(std::vector<uint8_t>* buffer) : ref(buffer) {}

void BinarySerializer::writeArrayStart() {
  ref->push_back(static_cast<uint8_t>(PackType::ARRXX));
}

void BinarySerializer::writeArrayEnd() {
  ref->push_back(static_cast<uint8_t>(PackType::ARRYY));
}

void BinarySerializer::startFixedArray(size_t length) {
  if (length > std::numeric_limits<uint32_t>::max()) {
    throw std::domain_error("cannot serialize array");
  }

  ref->push_back(static_cast<uint8_t>(PackType::ARR32));
  writeWord(*ref, static_cast<uint32_t>(length));
}

void BinarySerializer::write_type(PackType type) {
  ref->push_back(static_cast<uint8_t>(type));
}

BinaryDeserializer::BinaryDeserializer(const std::vector<uint8_t>* buffer)
    : ref(buffer), pos(0) {}

void BinaryDeserializer::checkType(PackType type) const {
  PackType ref_type = static_cast<PackType>(ref->at(pos));
  if (type != ref_type) {
    LOG(FATAL) << "invalid type: " << type << " (ref is " << ref_type << ")";
    throw std::domain_error("type mismatch!");
  }
  ++pos;
}

void BinaryDeserializer::checkDynamicArray() const {
  PackType ref_type = static_cast<PackType>(ref->at(pos));
  if (ref_type != PackType::ARRXX) {
    throw std::domain_error("type mismatch!");
  }
  ++pos;
}

bool BinaryDeserializer::isDynamicArrayEnd() const {
  PackType ref_type = static_cast<PackType>(ref->at(pos));
  if (ref_type != PackType::ARRYY) {
    return false;
  }
  ++pos;
  return true;
}

void BinaryDeserializer::checkFixedArrayLength(size_t length) const {
  PackType ref_type = static_cast<PackType>(ref->at(pos));
  if (ref_type != PackType::ARR32 && ref_type != PackType::STR32) {
    throw std::domain_error("type mismatch!");
  }

  ++pos;
  uint32_t ref_length;
  readWord(*ref, pos, ref_length);
  pos += sizeof(ref_length);
  if (length != static_cast<size_t>(length)) {
    throw std::domain_error("length mismatch");
  }
}

size_t BinaryDeserializer::readFixedArrayLength() const {
  PackType ref_type = static_cast<PackType>(ref->at(pos));
  if (ref_type != PackType::ARR32 && ref_type != PackType::STR32) {
    throw std::domain_error("type mismatch!");
  }

  ++pos;
  uint32_t length;
  readWord(*ref, pos, length);
  pos += sizeof(length);
  return static_cast<size_t>(length);
}

BinaryConverter::BinaryConverter(BinarySerializer* serializer)
    : serializer_(serializer) {
  serializer_->writeArrayStart();
}

BinaryConverter::BinaryConverter(const BinaryDeserializer* deserializer)
    : deserializer_(deserializer) {
  deserializer_->checkDynamicArray();
}

BinaryConverter::~BinaryConverter() {
  if (serializer_) {
    serializer_->writeArrayEnd();
  }
}

void BinaryConverter::finalize() const {
  if (deserializer_ && !deserializer_->isDynamicArrayEnd()) {
    throw std::domain_error("attributes not fully read");
  }
}

template <>
void BinarySerializer::write<NodeAttributes>(const NodeAttributes& attrs) {
  BinaryConverter converter(this);
  BinaryNodeFactory::get_default().save(converter, attrs);
}

template <>
void BinarySerializer::write<EdgeAttributes>(const EdgeAttributes& attrs) {
  BinaryConverter converter(this);
  BinaryEdgeFactory::get_default().save(converter, attrs);
}

template <>
void BinarySerializer::write<SceneGraphNode>(const SceneGraphNode& node) {
  startFixedArray(3);
  write(node.layer);
  write(node.id);
  write(node.attributes());
}

template <>
void BinarySerializer::write<DynamicSceneGraphNode>(const DynamicSceneGraphNode& node) {
  startFixedArray(4);
  write(node.layer);
  write(node.id);
  write(node.timestamp.count());
  write(node.attributes());
}

template <>
void BinarySerializer::write<SceneGraphEdge>(const SceneGraphEdge& edge) {
  startFixedArray(3);
  write(edge.source);
  write(edge.target);
  write(*edge.info);
}

template <>
void BinarySerializer::write<MeshEdge>(const MeshEdge& edge) {
  startFixedArray(2);
  write(edge.source_node);
  write(edge.mesh_vertex);
}

}  // namespace serialization
}  // namespace kimera
