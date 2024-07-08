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
#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define THROW_SERIALIZATION_ERROR(msg)                     \
  std::stringstream ss;                                    \
  ss << "[" << __FILE__ << ":" << __LINE__ << "] " << msg; \
  throw std::domain_error(ss.str())

namespace spark_dsg::serialization {

class BinarySerializer;
class BinaryDeserializer;

namespace detail {

template <typename T>
void write_binary(BinarySerializer& s, const std::vector<T>& values);

template <typename T>
void read_binary(const BinaryDeserializer& s, std::vector<T>& values);

template <typename T>
void write_binary(BinarySerializer& s, const std::list<T>& values);

template <typename T>
void read_binary(const BinaryDeserializer& s, std::list<T>& values);

template <typename T, size_t N>
void write_binary(BinarySerializer& s, const std::array<T, N>& values);

template <typename T, size_t N>
void read_binary(const BinaryDeserializer& s, std::array<T, N>& values);

template <typename T>
void write_binary(BinarySerializer& s, const std::set<T>& values);

template <typename T>
void read_binary(const BinaryDeserializer& s, std::set<T>& values);

template <typename T>
void write_binary(BinarySerializer& s, const std::unordered_set<T>& values);

template <typename T>
void read_binary(const BinaryDeserializer& s, std::unordered_set<T>& values);

template <typename K, typename V>
void write_binary(BinarySerializer& s, const std::map<K, V>& values);

template <typename K, typename V>
void read_binary(const BinaryDeserializer& s, std::map<K, V>& values);

template <typename K, typename V>
void write_binary(BinarySerializer& s, const std::unordered_map<K, V>& values);

template <typename K, typename V>
void read_binary(const BinaryDeserializer& s, std::unordered_map<K, V>& values);

template <class T>
constexpr T static_const{};

struct write_binary_fn {
  template <typename T>
  constexpr auto operator()(BinarySerializer& serializer, const T& value) const
      -> decltype(write_binary(serializer, value)) {
    return write_binary(serializer, value);
  }
};

struct read_binary_fn {
  template <typename T>
  constexpr auto operator()(const BinaryDeserializer& deserializer, T& value) const
      -> decltype(read_binary(deserializer, value)) {
    return read_binary(deserializer, value);
  }
};

}  // namespace detail

namespace {

constexpr const auto& write_binary = detail::static_const<detail::write_binary_fn>;
constexpr const auto& read_binary = detail::static_const<detail::read_binary_fn>;

}  // namespace

namespace traits {

using BinaryWriterFunc = decltype(::spark_dsg::serialization::write_binary);
using BinaryReaderFunc = decltype(::spark_dsg::serialization::read_binary);

template <typename T>
using has_binary_writer =
    std::is_invocable<BinaryWriterFunc, serialization::BinarySerializer&, const T&>;

template <typename T>
using has_binary_reader =
    std::is_invocable<BinaryReaderFunc, const serialization::BinaryDeserializer&, T&>;

}  // namespace traits

namespace detail {

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

struct SwapEndian {
  inline static void swap(const uint8_t& u, uint8_t& v) { v = u; }

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
  T placeholder;
  SwapEndian::swap(value, placeholder);
  const uint8_t* ptr = reinterpret_cast<uint8_t*>(&placeholder);
  buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

template <typename T,
          typename std::enable_if<!NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void writeWord(std::vector<uint8_t>& buffer, const T& value) {
  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
  buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

template <typename T,
          typename std::enable_if<NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void readWord(const T* buffer, T& value) {
  SwapEndian::swap(*buffer, value);
}

template <typename T,
          typename std::enable_if<!NeedEndianSwap() && std::is_integral<T>::value,
                                  bool>::type = true>
void readWord(const T* buffer, T& value) {
  value = *buffer;
}

}  // namespace detail

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

struct PackTypeLookup {
  template <typename T>
  static constexpr PackType value() {
    return PackType::NIL;
  }
};

template <>
inline PackType PackTypeLookup::value<float>() {
  return PackType::FLOAT32;
};
template <>
inline PackType PackTypeLookup::value<double>() {
  return PackType::FLOAT64;
};
template <>
inline PackType PackTypeLookup::value<uint8_t>() {
  return PackType::UINT8;
};
template <>
inline PackType PackTypeLookup::value<uint16_t>() {
  return PackType::UINT16;
};
template <>
inline PackType PackTypeLookup::value<uint32_t>() {
  return PackType::UINT32;
};
template <>
inline PackType PackTypeLookup::value<uint64_t>() {
  return PackType::UINT64;
};
template <>
inline PackType PackTypeLookup::value<int8_t>() {
  return PackType::INT8;
};
template <>
inline PackType PackTypeLookup::value<int16_t>() {
  return PackType::INT16;
};
template <>
inline PackType PackTypeLookup::value<int32_t>() {
  return PackType::INT32;
};
template <>
inline PackType PackTypeLookup::value<int64_t>() {
  return PackType::INT64;
};

class BinarySerializer {
 public:
  //! Pointer to buffer to fill
  explicit BinarySerializer(std::vector<uint8_t>* buffer);
  //! Mark the start of a dynamic array
  void startDynamicArray();
  //! Mark the end of a dynamic array
  void endDynamicArray();
  //! Mark a fixed size array
  void startFixedArray(size_t length);

 public:
  //! Dispatch write for arbitrary structure
  template <typename T,
            std::enable_if_t<traits::has_binary_writer<T>::value, bool> = true>
  void write(const T& value) {
    ::spark_dsg::serialization::write_binary(*this, value);
  }

  //! Write a scalar
  template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
  void write(T value) {
    write_type(PackTypeLookup::value<T>());
    detail::writeWord(*ref_, value);
  }

  template <typename T>
  void write(const std::optional<T>& value) {
    if (value) {
      write(*value);
    } else {
      write_type(PackType::NIL);
    }
  }

  template <typename T, typename P>
  void write(const std::chrono::duration<T, P>& value) {
    const auto temp = std::chrono::duration_cast<std::chrono::nanoseconds>(value);
    write(temp.count());
  }

  //! Write specialization for bool
  void write(bool value);

  //! Write specialization for float
  void write(const float& value);

  //! Write specialization for double
  void write(const double& value);

  //! Write specialization for string
  void write(const std::string& value);

 private:
  void write_type(PackType type);

  //! Pointer reference to buffer
  std::vector<uint8_t>* ref_;
};

class BinaryDeserializer {
 public:
  //! Construct binary serializer using pointer to raw memory and size
  BinaryDeserializer(const uint8_t* const buffer, size_t length);
  //! Overloaded constructor to handle buffer of bytes instead
  explicit BinaryDeserializer(const std::vector<uint8_t>& buffer);
  //! Check if the current byte matches the expected pack type
  void checkType(PackType type) const;
  //! Check that the current byte is an array
  void checkDynamicArray() const;
  //! Check if the curent byte is the end of an array or not
  bool isDynamicArrayEnd() const;
  //! Check that we're reading a fixed size array of the right length
  void checkFixedArrayLength(size_t length) const;
  //! Get the length of a fixed size array
  size_t readFixedArrayLength() const;
  //! Get the type of the current byte
  PackType getCurrType() const;
  //! Check if a packed bool is true
  bool checkIfTrue() const;
  //! Get current position in buffer
  size_t pos() const { return pos_; }

 public:
  //! Main dispatch to read for arbitrary data type
  template <typename T,
            std::enable_if_t<traits::has_binary_reader<T>::value, bool> = true>
  void read(T& value) const {
    ::spark_dsg::serialization::read_binary(*this, value);
  }

  //! Specialization for scalars
  template <typename T, std::enable_if_t<std::is_integral_v<T>, bool> = true>
  void read(T& value) const {
    checkType(PackTypeLookup::value<T>());
    check_valid(sizeof(T));
    detail::readWord(reinterpret_cast<const T*>(ref_ + pos_), value);
    pos_ += sizeof(T);
  }

  template <typename T>
  void read(std::optional<T>& value) const {
    const auto curr_type = getCurrType();
    if (curr_type == serialization::PackType::NIL) {
      checkType(serialization::PackType::NIL);
    } else {
      T opt_value;
      read(opt_value);
      value = opt_value;
    }
  }

  template <typename T, typename P>
  void read(std::chrono::duration<T, P>& value) const {
    std::chrono::nanoseconds::rep temp;
    read(temp);
    value = std::chrono::nanoseconds(temp);
  }

  //! Specialization for bool value
  void read(bool& value) const;

  //! Read specialization for float
  void read(float& value) const;

  //! Read specialization for double
  void read(double& value) const;

  //! Specialization for string
  void read(std::string& value) const;

  //! get a pointer
  template <typename T>
  const T* getReadPtr(size_t num_elements = 1) const;

 private:
  void check_valid(size_t num_bytes, size_t num_elements = 1) const;

  const uint8_t* const ref_;
  const size_t buffer_length_;
  mutable size_t pos_;
};

template <typename T>
const T* BinaryDeserializer::getReadPtr(size_t num_elements) const {
  check_valid(sizeof(T), num_elements);
  return reinterpret_cast<const T*>(ref_ + pos_);
}

namespace detail {

template <typename T>
void write_binary(BinarySerializer& s, const std::vector<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename T>
void read_binary(const BinaryDeserializer& s, std::vector<T>& values) {
  const size_t length = s.readFixedArrayLength();
  values.resize(length);
  for (size_t i = 0; i < length; ++i) {
    s.read(values[i]);
  }
}

template <typename T>
void write_binary(BinarySerializer& s, const std::list<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename T>
void read_binary(const BinaryDeserializer& s, std::list<T>& values) {
  values.clear();
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    auto& temp = values.emplace_back();
    s.read(temp);
  }
}

template <typename T, size_t N>
void write_binary(BinarySerializer& s, const std::array<T, N>& values) {
  s.startFixedArray(N);
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename T, size_t N>
void read_binary(const BinaryDeserializer& s, std::array<T, N>& values) {
  s.checkFixedArrayLength(N);
  for (size_t i = 0; i < N; ++i) {
    s.read(values[i]);
  }
}

template <typename T>
void write_binary(BinarySerializer& s, const std::set<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename T>
void read_binary(const BinaryDeserializer& s, std::set<T>& values) {
  values.clear();
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    T temp;
    s.read(temp);
    values.insert(temp);
  }
}

template <typename T>
void write_binary(BinarySerializer& s, const std::unordered_set<T>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value);
  }
}

template <typename T>
void read_binary(const BinaryDeserializer& s, std::unordered_set<T>& values) {
  values.clear();
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    T temp;
    s.read(temp);
    values.insert(temp);
  }
}

template <typename K, typename V>
void write_binary(BinarySerializer& s, const std::map<K, V>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value.first);
    s.write(value.second);
  }
}

template <typename K, typename V>
void read_binary(const BinaryDeserializer& s, std::map<K, V>& values) {
  values.clear();
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    K key;
    s.read(key);
    auto& value = values[key];
    s.read(value);
  }
}

template <typename K, typename V>
void write_binary(BinarySerializer& s, const std::unordered_map<K, V>& values) {
  s.startFixedArray(values.size());
  for (const auto& value : values) {
    s.write(value.first);
    s.write(value.second);
  }
}

template <typename K, typename V>
void read_binary(const BinaryDeserializer& s, std::unordered_map<K, V>& values) {
  values.clear();
  const size_t length = s.readFixedArrayLength();
  for (size_t i = 0; i < length; ++i) {
    K key;
    s.read(key);
    auto& value = values[key];
    s.read(value);
  }
}

}  // namespace detail

}  // namespace spark_dsg::serialization
