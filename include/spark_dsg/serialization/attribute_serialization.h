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

#include <map>
#include <nlohmann/json.hpp>
#include <sstream>

#include "spark_dsg/serialization/attribute_registry.h"
#include "spark_dsg/serialization/binary_serialization.h"

namespace spark_dsg::serialization {

struct SerializationImpl {};

struct BinaryWriter : SerializationImpl {
  BinaryWriter(BinarySerializer* serializer) : serializer_(serializer) {}

  template <typename T>
  void write(const T& value) {
    serializer_->write(value);
  }

  BinarySerializer* serializer_;
};

struct BinaryReader : SerializationImpl {
  BinaryReader(const BinaryDeserializer* deserializer) : deserializer_(deserializer) {}

  template <typename T>
  void read(T& value) {
    deserializer_->read(value);
  }

  const BinaryDeserializer* deserializer_;
};

// for visitor set context
struct JsonWriter : SerializationImpl {
  explicit JsonWriter(nlohmann::json* record) : ref(record) {}
  ~JsonWriter() = default;

  template <typename T>
  void write(const std::string& name, const T& value) {
    (*ref)[name] = value;
  }

  nlohmann::json* ref = nullptr;
};

// for visitor set context
struct JsonReader : SerializationImpl {
  explicit JsonReader(const nlohmann::json* rec) : cref(rec) {}
  ~JsonReader() = default;

  template <typename T>
  void read(const std::string& name, T& value) const {
    if (!cref->contains(name)) {
      return;
    }

    value = cref->at(name).get<T>();
  }

  const nlohmann::json* cref = nullptr;
};

class Visitor {
 public:
  static Visitor& instance();

  template <typename T>
  static void visit(const std::string& name, T& value);

  template <typename Attrs>
  static void to(nlohmann::json& record, const Attrs& attrs);

  template <typename Attrs>
  static void to(BinarySerializer& serializer, const Attrs& attrs);

  template <typename Attrs>
  static std::unique_ptr<Attrs> from(const AttributeFactory<Attrs>& factory,
                                     const nlohmann::json& record);

  template <typename Attrs>
  static std::unique_ptr<Attrs> from(const AttributeFactory<Attrs>& factory,
                                     const BinaryDeserializer& deserializer);

 private:
  Visitor();

  enum class Type {
    BINARY_WRITE,
    BINARY_READ,
    JSON_WRITE,
    JSON_READ,
  } type_;

  std::unique_ptr<SerializationImpl> impl_;
  inline thread_local static std::unique_ptr<Visitor> s_instance_ = nullptr;
};

template <typename T>
void field(const std::string& name, T& value) {
  Visitor::visit(name, value);
}

template <typename T>
void Visitor::visit(const std::string& name, T& value) {
  auto& visitor = instance();
  if (!visitor.impl_) {
    // TODO(nathan) warn or do something here
    return;
  }

  auto impl_pointer = visitor.impl_.get();
  switch (visitor.type_) {
    case Type::BINARY_WRITE:
      static_cast<BinaryWriter*>(impl_pointer)->write(value);
      break;
    case Type::BINARY_READ:
      static_cast<BinaryReader*>(impl_pointer)->read(value);
      break;
    case Type::JSON_WRITE:
      static_cast<JsonWriter*>(impl_pointer)->write(name, value);
      break;
    case Type::JSON_READ:
      static_cast<JsonReader*>(impl_pointer)->read(name, value);
      break;
    default:
      // TODO(nathan) warn or do something here
      break;
  }
}

template <typename Attrs>
void Visitor::to(nlohmann::json& record, const Attrs& attrs) {
  auto& visitor = instance();
  visitor.type_ = Type::JSON_WRITE;
  visitor.impl_ = std::make_unique<JsonWriter>(&record);
  record["type"] = attrs.registration().name;
  attrs.serialization_info();
  visitor.impl_.reset();
}

template <typename Attrs>
void Visitor::to(BinarySerializer& serializer, const Attrs& attrs) {
  auto& visitor = instance();
  visitor.type_ = Type::BINARY_WRITE;
  visitor.impl_ = std::make_unique<BinaryWriter>(&serializer);
  serializer.write(attrs.registration().type_id);
  attrs.serialization_info();
  visitor.impl_.reset();
}

template <typename Attrs>
std::unique_ptr<Attrs> Visitor::from(const AttributeFactory<Attrs>& factory,
                                     const nlohmann::json& record) {
  auto& visitor = instance();
  visitor.type_ = Type::JSON_READ;
  visitor.impl_ = std::make_unique<JsonReader>(&record);

  auto attrs = factory.create(record.at("type").get<std::string>());
  if (!attrs) {
    return nullptr;
  }

  attrs->serialization_info();
  visitor.impl_.reset();
  return attrs;
}

template <typename Attrs>
std::unique_ptr<Attrs> Visitor::from(const AttributeFactory<Attrs>& factory,
                                     const BinaryDeserializer& deserializer) {
  auto& visitor = instance();
  visitor.type_ = Type::BINARY_READ;
  visitor.impl_ = std::make_unique<BinaryReader>(&deserializer);

  uint8_t type;
  deserializer.read(type);
  auto attrs = factory.create(type);
  if (!attrs) {
    return nullptr;
  }

  attrs->serialization_info();
  visitor.impl_.reset();
  return attrs;
}

}  // namespace spark_dsg::serialization
