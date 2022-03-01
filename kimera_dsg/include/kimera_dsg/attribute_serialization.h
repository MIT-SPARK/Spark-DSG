#pragma once
#include "kimera_dsg/attribute_serializer.h"
#include "kimera_dsg/edge_attributes.h"
#include "kimera_dsg/node_attributes.h"

#include <nlohmann/json.hpp>

#include <sstream>

using json = nlohmann::json;

namespace kimera {

template <class T>
class AttributeFactory {
 public:
  using TypePtr = typename T::Ptr;
  using ConstructorFunc = TypePtr (*)(void);
  using ConstructorMap = std::map<std::string, ConstructorFunc>;

  virtual ~AttributeFactory() = default;

  inline void add(const std::string& type, ConstructorFunc constructor) {
    factory_map_[type] = constructor;
  }

  TypePtr create(const json& record) const {
    std::string attr_type;
    try {
      attr_type = record.at("type").get<std::string>();
    } catch (const nlohmann::detail::type_error& e) {
      std::stringstream ss;
      ss << e.what() << " when reading type: " << record;
      throw std::domain_error(ss.str());
    }

    auto map_func = factory_map_.find(attr_type);
    if (map_func == factory_map_.end()) {
      std::stringstream ss;
      ss << "no parser function for type: " << attr_type;
      throw std::domain_error(ss.str());
    }

    if (map_func->second == nullptr) {
      std::stringstream ss;
      ss << "invalid parser function for type: " << attr_type;
      throw std::domain_error(ss.str());
    }

    auto to_return = map_func->second();
    try {
      to_return->deserialize(AttributeSerializer(record));
    } catch (const nlohmann::detail::type_error& e) {
      std::stringstream ss;
      ss << e.what() << " when converting: " << record;
      throw std::domain_error(ss.str());
    }
    return to_return;
  }

  static AttributeFactory<T>& instance() {
    if (!s_instance_) {
      s_instance_.reset(new AttributeFactory());
    }

    return *s_instance_;
  }

  bool default_set;

 protected:
  AttributeFactory() : default_set(false) {}

  ConstructorMap factory_map_;

  static std::unique_ptr<AttributeFactory> s_instance_;
};

class NodeAttributeFactory : public AttributeFactory<NodeAttributes> {
 public:
  static AttributeFactory<NodeAttributes>& get_default();
};

class EdgeAttributeFactory : public AttributeFactory<EdgeAttributes> {
 public:
  static AttributeFactory<EdgeAttributes>& get_default();
};

}  // namespace kimera
