#pragma once
#include "kimera_dsg/node_attributes.h"
#include "kimera_dsg/scene_graph_layer.h"

#include <glog/logging.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace kimera {

using NodeAttributeConstructor = NodeAttributes::Ptr (*)(void);
using EdgeInfoConstructor = SceneGraphEdgeInfo::Ptr (*)(void);

template <class T>
class AttributeFactory {
 public:
  using TConstructorFunc = typename T::Ptr (*)(void);

  AttributeFactory() = default;
  virtual ~AttributeFactory() = default;

  void registerConstructor(const std::string& type, TConstructorFunc constructor) {
    factory_map_[type] = constructor;
  }

  typename T::Ptr create(const json& record) const {
    std::string attr_type;
    try {
      std::string type_key(T::TYPE_KEY);
      attr_type = record.at(type_key).get<std::string>();
    } catch (const nlohmann::detail::type_error& e) {
      LOG(FATAL) << e.what() << " when reading type: " << record;
    }

    auto map_func = factory_map_.find(attr_type);
    if (map_func == factory_map_.end()) {
      VLOG(1) << "Failed to find parser function for type: " << attr_type;
      return nullptr;
    }

    if (map_func->second == nullptr) {
      VLOG(1) << "Invalid parser for type: " << attr_type;
      return nullptr;
    }

    typename T::Ptr to_return = map_func->second();
    try {
      to_return->fillFromJson(record);
    } catch (const nlohmann::detail::type_error& e) {
      LOG(FATAL) << e.what() << " when converting: " << record;
    }
    return to_return;
  }

 private:
  using ConstructorMap = std::map<std::string, TConstructorFunc>;

  ConstructorMap factory_map_;
};

class NodeAttributeFactory : public AttributeFactory<NodeAttributes> {
 public:
  NodeAttributeFactory() = default;

  static NodeAttributeFactory Default();
};

class EdgeInfoFactory : public AttributeFactory<SceneGraphEdgeInfo> {
 public:
  EdgeInfoFactory() = default;

  static EdgeInfoFactory Default();
};

#define REGISTER_ATTR_FACTORY(factory, classname)                  \
  static_assert(std::is_base_of<NodeAttributes, classname>::value, \
                "class is not derived from NodeAttributes");       \
  factory.registerConstructor(                                     \
      #classname, [](void) { return NodeAttributes::Ptr(new classname()); })

#define REGISTER_INFO_FACTORY(factory, classname)                      \
  static_assert(std::is_base_of<SceneGraphEdgeInfo, classname>::value, \
                "class is not derived from SceneGraphEdgeInfo");       \
  factory.registerConstructor(                                         \
      #classname, [](void) { return SceneGraphEdgeInfo::Ptr(new classname()); })

}  // namespace kimera
