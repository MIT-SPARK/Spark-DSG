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
#include "spark_dsg/serialization/binary_serializer.h"

#include <iomanip>

#include "spark_dsg/logging.h"

namespace spark_dsg::serialization {

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
      out << "INVALID: " << std::hex << std::showbase << std::setw(2)
          << static_cast<int>(type);
      break;
  }
  return out;
}

BinarySerializer::BinarySerializer(std::vector<uint8_t>* buffer) : ref(buffer) {}

void BinarySerializer::startDynamicArray() {
  ref->push_back(static_cast<uint8_t>(PackType::ARRXX));
}

void BinarySerializer::endDynamicArray() {
  ref->push_back(static_cast<uint8_t>(PackType::ARRYY));
}

void BinarySerializer::startFixedArray(size_t length) {
  if (length > std::numeric_limits<uint32_t>::max()) {
    THROW_SERIALIZATION_ERROR("cannot serialize array: "
                              << length << " > "
                              << std::numeric_limits<uint32_t>::max());
  }

  ref->push_back(static_cast<uint8_t>(PackType::ARR32));
  writeWord(*ref, static_cast<uint32_t>(length));
}

void BinarySerializer::write_type(PackType type) {
  ref->push_back(static_cast<uint8_t>(type));
}

BinaryDeserializer::BinaryDeserializer(const uint8_t* const buffer, size_t size)
    : ref(buffer), buffer_length(size), pos(0) {}

BinaryDeserializer::BinaryDeserializer(const std::vector<uint8_t>& buffer)
    : BinaryDeserializer(buffer.data(), buffer.size()) {}

void BinaryDeserializer::checkType(PackType type) const {
  const auto ref_type = getCurrType();
  if (type != ref_type) {
    THROW_SERIALIZATION_ERROR("type mismatch: expecting " << type << " but got "
                                                          << ref_type);
  }
  ++pos;
}

void BinaryDeserializer::checkDynamicArray() const {
  const auto ref_type = getCurrType();
  if (ref_type != PackType::ARRXX) {
    THROW_SERIALIZATION_ERROR(
        "type mismatch: expecting ARRXX at dynamic array start but got " << ref_type);
  }

  ++pos;
}

bool BinaryDeserializer::isDynamicArrayEnd() const {
  const auto ref_type = getCurrType();
  if (ref_type != PackType::ARRYY) {
    return false;
  }
  ++pos;
  return true;
}

void BinaryDeserializer::checkFixedArrayLength(size_t length) const {
  const auto ref_type = getCurrType();
  if (ref_type != PackType::ARR32 && ref_type != PackType::STR32) {
    THROW_SERIALIZATION_ERROR("type mismatch: expecting ARR32 or STR32 but got "
                              << ref_type);
  }

  ++pos;
  uint32_t ref_length;
  readWord(getReadPtr<uint32_t>(), ref_length);
  pos += sizeof(ref_length);
  if (length != static_cast<size_t>(ref_length)) {
    THROW_SERIALIZATION_ERROR("fixed length mismatch: " << ref_length
                                                        << " != " << length);
  }
}

size_t BinaryDeserializer::readFixedArrayLength() const {
  const auto ref_type = getCurrType();
  if (ref_type != PackType::ARR32 && ref_type != PackType::STR32) {
    THROW_SERIALIZATION_ERROR("type mismatch: expecting ARR32 or STR32 but got "
                              << ref_type);
  }

  ++pos;
  uint32_t length;
  readWord(getReadPtr<uint32_t>(), length);
  pos += sizeof(length);
  return static_cast<size_t>(length);
}

bool BinaryDeserializer::checkIfTrue() const {
  PackType type;
  try {
    type = getCurrType();
    checkType(type);
  } catch (...) {
    return false;
  }

  return type == PackType::TRUE;
}

BinaryConverter::BinaryConverter(BinarySerializer* serializer)
    : serializer_(serializer) {
  serializer_->startDynamicArray();
}

BinaryConverter::BinaryConverter(const BinaryDeserializer* deserializer)
    : deserializer_(deserializer) {
  deserializer_->checkDynamicArray();
}

BinaryConverter::~BinaryConverter() {
  if (serializer_) {
    serializer_->endDynamicArray();
  }
}

void BinaryConverter::finalize() const {
  if (deserializer_ && !deserializer_->isDynamicArrayEnd()) {
    THROW_SERIALIZATION_ERROR("attributes not fully read");
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
  if (node.timestamp) {
    startFixedArray(4);
    write(node.layer);
    write(node.id);
    write(node.timestamp->count());
    write(node.attributes());
  } else {
    startFixedArray(3);
    write(node.layer);
    write(node.id);
    write(node.attributes());
  }
}

template <>
void BinarySerializer::write<SceneGraphEdge>(const SceneGraphEdge& edge) {
  startFixedArray(3);
  write(edge.source);
  write(edge.target);
  write(*edge.info);
}

template <>
void BinarySerializer::write<Mesh>(const Mesh& mesh) {
  std::vector<uint8_t> buffer;
  mesh.serializeToBinary(buffer);
  write(buffer);
}

template <>
void BinaryDeserializer::read<Mesh>(Mesh& mesh) const {
  std::vector<uint8_t> buffer;
  read(buffer);
  mesh = *Mesh::deserializeFromBinary(buffer.data(), buffer.size());
}

}  // namespace spark_dsg::serialization
