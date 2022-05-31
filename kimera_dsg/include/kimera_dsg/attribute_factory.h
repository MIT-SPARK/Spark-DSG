#pragma once
#include "kimera_dsg/attribute_serialization.h"

#include <sstream>
#include <typeindex>
#include <typeinfo>

namespace kimera {

template <class T, class C>
class AttributeFactory {
 public:
  using TypePtr = typename T::Ptr;
  using ConstructorFunc = TypePtr (*)(const C&);
  using SerializerFunc = void (*)(C&, const T&);
  using UpdaterFunc = void (*)(const C&, T&);
  using ConstructorMap = std::map<std::string, ConstructorFunc>;
  using SerializerMap = std::map<std::string, SerializerFunc>;
  using UpdaterMap = std::map<std::string, UpdaterFunc>;

  virtual ~AttributeFactory() = default;

  template <typename Attr>
  void add(const std::string& name,
           const ConstructorFunc& constructor,
           const SerializerFunc& serializer,
           const UpdaterFunc& updater) {
    type_names_[std::type_index(typeid(Attr))] = name;
    factory_map_[name] = constructor;
    serializer_map_[name] = serializer;
    updater_map_[name] = updater;
  }

  void setTypeIndexMap(const std::map<std::string, uint8_t>& name_to_indices) {
    name_index_map_ = name_to_indices;
    index_name_map_.clear();
    for (const auto& name_idx_pair : name_to_indices) {
      index_name_map_[name_idx_pair.second] = name_idx_pair.first;
    }
  }

  void addAlias(const std::string& n1, const std::string& n2) { aliases_[n1] = n2; }

  void update(const C& converter, T& attrs) const {
    auto attr_type = converter.read_type(*this);
    if (aliases_.count(attr_type)) {
      attr_type = aliases_.at(attr_type);
    }

    auto updater_func = updater_map_.find(attr_type);
    if (updater_func == updater_map_.end()) {
      std::stringstream ss;
      ss << "no parser function for type: " << attr_type;
      throw std::domain_error(ss.str());
    }

    if (updater_func->second == nullptr) {
      std::stringstream ss;
      ss << "invalid parser function for type: " << attr_type;
      throw std::domain_error(ss.str());
    }

    updater_func->second(converter, attrs);
  }

  TypePtr create(const C& converter) const {
    auto attr_type = converter.read_type(*this);
    if (aliases_.count(attr_type)) {
      attr_type = aliases_.at(attr_type);
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

    return map_func->second(converter);
  }

  void save(C& converter, const T& attrs) const {
    const auto name = lookupName(attrs);
    converter.mark_type(*this, name);
    if (not serializer_map_.count(name)) {
      std::stringstream ss;
      ss << "unregistered type: " << name;
      throw std::domain_error(ss.str());
    }
    serializer_map_.at(name)(converter, attrs);
  }

  std::string lookupIndexName(uint8_t index) const {
    auto iter = index_name_map_.find(index);
    if (iter == index_name_map_.end()) {
      throw std::domain_error("unregistered name!");
    }

    return iter->second;
  }

  uint8_t lookupNameIndex(const std::string& name) const {
    auto iter = name_index_map_.find(name);
    if (iter == name_index_map_.end()) {
      throw std::domain_error("unregistered name!");
    }

    return iter->second;
  }

  std::string lookupName(const T& attrs) const {
    auto iter = type_names_.find(std::type_index(typeid(attrs)));
    if (iter == type_names_.end()) {
      std::stringstream ss;
      ss << "type " << typeid(attrs).name() << " is not registered";
      throw std::domain_error(ss.str());
    }

    return iter->second;
  }

  static AttributeFactory<T, C>& instance() {
    if (!s_instance_) {
      s_instance_.reset(new AttributeFactory());
    }

    return *s_instance_;
  }

  bool default_set;

 protected:
  AttributeFactory() : default_set(false) {}

  std::map<std::string, std::string> aliases_;
  ConstructorMap factory_map_;
  SerializerMap serializer_map_;
  UpdaterMap updater_map_;
  std::map<std::type_index, std::string> type_names_;
  std::map<std::string, uint8_t> name_index_map_;
  std::map<uint8_t, std::string> index_name_map_;

  static std::unique_ptr<AttributeFactory> s_instance_;
};

#define REGISTER_ATTR(factory, type, Converter)                            \
  factory::instance().template add<type>(                                  \
      #type,                                                               \
      [](const Converter& converter) {                                     \
        typename factory::TypePtr attrs = std::make_unique<type>();        \
        attributes::deserialize(converter, static_cast<type&>(*attrs));    \
        return attrs;                                                      \
      },                                                                   \
      [](Converter& converter, const auto& attrs) {                        \
        attributes::serialize(converter, static_cast<const type&>(attrs)); \
      },                                                                   \
      [](const Converter& converter, auto& attrs) {                        \
        attributes::deserialize(converter, static_cast<type&>(attrs));     \
      })

template <typename C>
class NodeAttributeFactory : public AttributeFactory<NodeAttributes, C> {
 public:
  static AttributeFactory<NodeAttributes, C>& get_default() {
    AttributeFactory<NodeAttributes, C>& factory = NodeAttributeFactory::instance();
    if (factory.default_set) {
      return factory;
    }

    REGISTER_ATTR(NodeAttributeFactory, NodeAttributes, C);
    REGISTER_ATTR(NodeAttributeFactory, SemanticNodeAttributes, C);
    REGISTER_ATTR(NodeAttributeFactory, ObjectNodeAttributes, C);
    REGISTER_ATTR(NodeAttributeFactory, RoomNodeAttributes, C);
    REGISTER_ATTR(NodeAttributeFactory, PlaceNodeAttributes, C);
    REGISTER_ATTR(NodeAttributeFactory, AgentNodeAttributes, C);

    factory.setTypeIndexMap({{"NodeAttributes", 0},
                             {"SemanticNodeAttributes", 1},
                             {"ObjectNodeAttributes", 2},
                             {"RoomNodeAttributes", 3},
                             {"PlaceNodeAttributes", 4},
                             {"AgentNodeAttributes", 5}});

    factory.default_set = true;
    return factory;
  }
};

template <typename C>
class EdgeAttributeFactory : public AttributeFactory<EdgeAttributes, C> {
 public:
  static AttributeFactory<EdgeAttributes, C>& get_default() {
    auto& factory = EdgeAttributeFactory::instance();
    if (factory.default_set) {
      return factory;
    }

    REGISTER_ATTR(EdgeAttributeFactory, EdgeAttributes, C);
    factory.addAlias("SceneGraphEdgeInfo", "EdgeAttributes");

    factory.setTypeIndexMap({{"EdgeAttributes", 0}});
    factory.default_set = true;
    return factory;
  }
};

#undef REGISTER_ATTR

}  // namespace kimera
