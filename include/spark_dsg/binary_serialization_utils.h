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
#include <Eigen/Dense>
#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "spark_dsg/node_attributes.h"

// endian swap adapted from:
// https://github.com/fraillt/bitsery/blob/master/include/bitsery/details/adapter_common.h

#define LSB16 0x00ff
#define MSB16 0xff00
#define LSB32 0x000000ff
#define USB32 0x0000ff00
#define HSB32 0x00ff0000
#define MSB32 0xff000000
#define LHB64 0x00000000ffffffff
#define MHB64 0xffffffff00000000
#define LQB64 0x0000ffff0000ffff
#define MQB64 0xffff0000ffff0000
#define L8B64 0x00ff00ff00ff00ff
#define M8B64 0xff00ff00ff00ff00

namespace spark_dsg {
namespace serialization {

struct SwapEndian {
  inline static void swap(const uint64_t& u, uint64_t& v) {
    v = (u & LHB64) << 32 | (u & MHB64) >> 32;
    v = (v & LQB64) << 16 | (v & MQB64) >> 16;
    v = (v & L8B64) << 8 | (v & M8B64) >> 8;
  }

  inline static void swap(const uint32_t& u, uint32_t& v) {
    v = (u & LSB32) << 24 | (u & USB32) << 8 | (u & HSB32) >> 8 | (u & MSB32) >> 24;
  }

  inline static void swap(const uint16_t& u, uint16_t& v) {
    v = (u & LSB16) << 8 | (u & MSB16) >> 8;
  }
};

struct EndianTest {
  static constexpr uint32_t SampleConst = 0x01020304;
  static constexpr uint8_t SampleByte = static_cast<const uint8_t&>(SampleConst);
};

constexpr bool NeedEndianSwap() { return EndianTest::SampleByte != 0x04; }

template <typename T,
          typename std::enable_if<NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void writeWord(std::vector<uint8_t>& buffer, const T& value) {
  static_assert(sizeof(T) > 1, "only required for 2 or more bytes");
  T placeholder;
  SwapEndian::swap(value, placeholder);
  const uint8_t* ptr = reinterpret_cast<uint8_t*>(&placeholder);
  buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

template <typename T,
          typename std::enable_if<!NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void writeWord(std::vector<uint8_t>& buffer, const T& value) {
  static_assert(sizeof(T) > 1, "only required for 2 or more bytes");
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
  buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

template <typename T,
          typename std::enable_if<NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void readWord(const T* buffer, T& value) {
  static_assert(sizeof(T) > 1, "only required for 2 or more bytes");
  SwapEndian::swap(*buffer, value);
}

template <typename T,
          typename std::enable_if<!NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void readWord(const T* buffer, T& value) {
  static_assert(sizeof(T) > 1, "only required for 2 or more bytes");
  value = *buffer;
}

// subset of the msgpack spec: https://github.com/msgpack/msgpack/blob/master/spec.md
enum class PackType : uint8_t {
  NIL = 0xc0,
  FALSE = 0xc2,
  TRUE = 0xc3,
  FLOAT32 = 0xca,
  FLOAT64 = 0xcb,
  UINT8 = 0xcc,
  UINT16 = 0xcd,
  UINT32 = 0xce,
  UINT64 = 0xcf,
  INT8 = 0xd0,
  INT16 = 0xd1,
  INT32 = 0xd2,
  INT64 = 0xd3,
  STR32 = 0xdb,
  ARR32 = 0xdd,
  ARRXX = 0x00,  // dynamic array start
  ARRYY = 0x01,  // dynamic array end
};

namespace detail {

template <typename Serializer>
void write_binary(Serializer& s, const bool& value) {
  s.write_type(value ? PackType::TRUE : PackType::FALSE);
}

template <typename Serializer>
void write_binary(Serializer& s, const uint8_t& value) {
  s.write_type(PackType::UINT8);
  s.ref->push_back(value);
}

template <typename Serializer>
void write_binary(Serializer& s, const int8_t& value) {
  s.write_type(PackType::INT8);
  s.ref->push_back(value);
}

template <typename Serializer>
void write_binary(Serializer& s, const uint16_t& value) {
  s.write_type(PackType::UINT16);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const int16_t& value) {
  s.write_type(PackType::INT16);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const uint32_t& value) {
  s.write_type(PackType::UINT32);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const int32_t& value) {
  s.write_type(PackType::INT32);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const uint64_t& value) {
  s.write_type(PackType::UINT64);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const int64_t& value) {
  s.write_type(PackType::INT64);
  writeWord(*s.ref, value);
}

template <typename Serializer>
void write_binary(Serializer& s, const float& value) {
  s.write_type(PackType::FLOAT32);
  writeWord(*s.ref, reinterpret_cast<const uint32_t&>(value));
}

template <typename Serializer>
void write_binary(Serializer& s, const double& value) {
  s.write_type(PackType::FLOAT64);
  writeWord(*s.ref, reinterpret_cast<const uint64_t&>(value));
}

template <typename Serializer>
void write_binary(Serializer& s, const std::string& value) {
  s.startFixedArray(value.size());
  s.ref->insert(s.ref->end(), value.begin(), value.end());
}

template <typename Serializer, typename T>
void write_binary(Serializer& s, const std::vector<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename Serializer, typename T>
void write_binary(Serializer& s, const std::list<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, bool& value) {
  PackType ref_type = s.getCurrType();
  switch (ref_type) {
    case PackType::FALSE:
      value = false;
      break;
    case PackType::TRUE:
      value = true;
      break;
    default:
      throw std::domain_error("type is not bool!");
  }
  return 1;
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, uint8_t& value) {
  s.checkType(PackType::UINT8);
  value = *s.template getReadPtr<uint8_t>();
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, int8_t& value) {
  s.checkType(PackType::INT8);
  value = *s.template getReadPtr<int8_t>();
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, uint16_t& value) {
  s.checkType(PackType::UINT16);
  readWord(s.template getReadPtr<uint16_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, int16_t& value) {
  s.checkType(PackType::INT16);
  readWord(s.template getReadPtr<int16_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, uint32_t& value) {
  s.checkType(PackType::UINT32);
  readWord(s.template getReadPtr<uint32_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, int32_t& value) {
  s.checkType(PackType::INT32);
  readWord(s.template getReadPtr<int32_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, uint64_t& value) {
  s.checkType(PackType::UINT64);
  readWord(s.template getReadPtr<uint64_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, int64_t& value) {
  s.checkType(PackType::INT64);
  readWord(s.template getReadPtr<int64_t>(), value);
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, float& value) {
  s.checkType(PackType::FLOAT32);
  readWord(s.template getReadPtr<uint32_t>(), reinterpret_cast<uint32_t&>(value));
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, double& value) {
  s.checkType(PackType::FLOAT64);
  readWord(s.template getReadPtr<uint64_t>(), reinterpret_cast<uint64_t&>(value));
  return sizeof(value);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, std::string& value) {
  const size_t length = s.readFixedArrayLength();
  value = std::string(s.template getReadPtr<char>(length), length);
  return length;
}

template <typename Deserializer, typename T>
size_t read_binary(const Deserializer& s, std::vector<T>& values) {
  const size_t length = s.readFixedArrayLength();
  values.resize(length);
  for (size_t i = 0; i < length; ++i) {
    s.read(values[i]);
  }
  return 0;
}

template <typename Deserializer, typename T>
size_t read_binary(const Deserializer& s, std::list<T>& values) {
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    T temp;
    s.read(temp);
    values.push_back(temp);
  }
  return 0;
}

template <class T>
constexpr T static_const{};

struct write_binary_fn {
  template <typename Serializer, typename T>
  constexpr auto operator()(Serializer& serializer, const T& value) const
      -> decltype(write_binary(serializer, value)) {
    return write_binary(serializer, value);
  }
};

struct read_binary_fn {
  template <typename Deserializer, typename T>
  constexpr auto operator()(const Deserializer& deserializer, T& value) const
      -> decltype(read_binary(deserializer, value)) {
    return read_binary(deserializer, value);
  }
};

}  // namespace detail

namespace {

constexpr const auto& write_binary = detail::static_const<detail::write_binary_fn>;

constexpr const auto& read_binary = detail::static_const<detail::read_binary_fn>;

}  // namespace

}  // namespace serialization
}  // namespace spark_dsg

namespace Eigen {

template <typename Serializer, typename Derived>
void write_binary(Serializer& s, const MatrixBase<Derived>& matrix) {
  s.startFixedArray(matrix.size() + 2);
  s.write(matrix.rows());
  s.write(matrix.cols());
  for (Index r = 0; r < matrix.rows(); ++r) {
    for (Index c = 0; c < matrix.cols(); ++c) {
      s.write(matrix(r, c));
    }
  }
}

template <typename Serializer, typename Scalar>
void write_binary(Serializer& s, const Quaternion<Scalar>& q) {
  s.startFixedArray(4);
  s.write(q.w());
  s.write(q.x());
  s.write(q.y());
  s.write(q.z());
}

template <typename Deserializer, typename Derived>
size_t read_binary(const Deserializer& s, MatrixBase<Derived>& matrix) {
  const size_t length = s.readFixedArrayLength();
  if (length < 2u) {
    throw std::out_of_range("array dimensions not present");
  }

  Index rows, cols;
  s.read(rows);
  s.read(cols);
  if (rows * cols + 2 != static_cast<int64_t>(length)) {
    throw std::out_of_range("array dimensions do not match length");
  }

  matrix.derived().resize(rows, cols);
  // TODO(nathan) warn about row/col mismatch
  for (Index r = 0; r < matrix.rows(); ++r) {
    for (Index c = 0; c < matrix.cols(); ++c) {
      s.read(matrix(r, c));
    }
  }
  return 0;
}

template <typename Deserializer, typename Scalar>
size_t read_binary(const Deserializer& s, Quaternion<Scalar>& q) {
  s.checkFixedArrayLength(4);
  s.read(q.w());
  s.read(q.x());
  s.read(q.y());
  s.read(q.z());
  return 0;
}

}  // namespace Eigen

namespace spark_dsg {

template <typename Serializer>
void write_binary(Serializer& s, const BoundingBox& box) {
  s.startFixedArray(5);
  s.write(static_cast<int32_t>(box.type));
  s.write(box.min);
  s.write(box.max);
  s.write(box.world_P_center);
  s.write(box.world_R_center);
}

template <typename Serializer>
void write_binary(Serializer& s, const NearestVertexInfo& info) {
  s.startFixedArray(3);
  s.startFixedArray(3);
  s.write(info.block[0]);
  s.write(info.block[1]);
  s.write(info.block[2]);
  s.startFixedArray(3);
  s.write(info.voxel_pos[0]);
  s.write(info.voxel_pos[1]);
  s.write(info.voxel_pos[2]);
  s.write(info.vertex);
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, BoundingBox& box) {
  s.checkFixedArrayLength(5);

  int32_t raw_type;
  s.read(raw_type);
  box.type = static_cast<BoundingBox::Type>(raw_type);

  s.read(box.min);
  s.read(box.max);
  s.read(box.world_P_center);
  s.read(box.world_R_center);
  return 0;
}

template <typename Deserializer>
size_t read_binary(const Deserializer& s, NearestVertexInfo& info) {
  s.checkFixedArrayLength(3);
  s.checkFixedArrayLength(3);
  s.read(info.block[0]);
  s.read(info.block[1]);
  s.read(info.block[2]);
  s.checkFixedArrayLength(3);
  s.read(info.voxel_pos[0]);
  s.read(info.voxel_pos[1]);
  s.read(info.voxel_pos[2]);
  s.read(info.vertex);
  return 0;
}

}  // namespace spark_dsg
