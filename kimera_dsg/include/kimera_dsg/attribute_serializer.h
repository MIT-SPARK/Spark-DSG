#pragma once
#include <nlohmann/json.hpp>

#include <string>

namespace kimera {

struct AttributeSerializer {
  AttributeSerializer() = default;

  AttributeSerializer(const nlohmann::json& record) : record(record) {}

  ~AttributeSerializer() = default;

  inline void mark_type(const std::string& type) { record["type"] = type; }

  template <typename T>
  void write(const std::string& name, const T& value) {
    record[name] = value;
  }

  template <typename T>
  void read(const std::string& name, T& value) const {
    if (!record.contains(name)) {
      return;
    }

    value = record.at(name).get<T>();
  }

  nlohmann::json record;
};

}  // namespace kimera
