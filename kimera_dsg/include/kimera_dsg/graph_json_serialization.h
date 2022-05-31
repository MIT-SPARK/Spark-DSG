#pragma once
#include "kimera_dsg/attribute_factory.h"

#include <nlohmann/json.hpp>

namespace kimera {

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

}  // namespace kimera
