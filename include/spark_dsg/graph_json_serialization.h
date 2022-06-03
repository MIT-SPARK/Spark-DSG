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

#include <nlohmann/json.hpp>

namespace spark_dsg {

struct JsonConverter {
  explicit JsonConverter(nlohmann::json* record) : ref(record) {}

  explicit JsonConverter(const nlohmann::json* rec)
      : cref(rec), attr_type(rec->at("type").get<std::string>()) {}

  ~JsonConverter() = default;

  template <typename Factory>
  inline std::string read_type(const Factory&) const {
    return attr_type;
  }

  template <typename Factory>
  void mark_type(const Factory&, const std::string& name) {
    (*ref)["type"] = name;
  }

  template <typename T>
  void write(const std::string& name, const T& value) {
    (*ref)[name] = value;
  }

  template <typename T>
  void read(const std::string& name, T& value) const {
    if (!cref->contains(name)) {
      return;
    }

    value = cref->at(name).get<T>();
  }

  const nlohmann::json* cref = nullptr;
  nlohmann::json* ref = nullptr;
  std::string attr_type;
};

using JsonNodeFactory = NodeAttributeFactory<JsonConverter>;
using JsonEdgeFactory = EdgeAttributeFactory<JsonConverter>;

void to_json(nlohmann::json& record, const NodeAttributes& attributes) {
  JsonConverter converter(&record);
  JsonNodeFactory::get_default().save(converter, attributes);
}

void to_json(nlohmann::json& record, const EdgeAttributes& attributes) {
  JsonConverter converter(&record);
  JsonEdgeFactory::get_default().save(converter, attributes);
}

}  // namespace spark_dsg
