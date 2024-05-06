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

#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <type_traits>
#include <vector>

namespace spark_dsg::serialization {

template <typename T>
class AttributeFactory {
 public:
  using Constructor = std::function<typename T::Ptr()>;
  using FactoryMap = std::map<std::string, Constructor>;

  AttributeFactory(const std::vector<std::string>& names, const FactoryMap& factories) {
    for (size_t i = 0; i < names.size(); ++i) {
      auto iter = factories.find(names[i]);
      if (iter == factories.end()) {
        // TODO(nathan) warn about unknown name
        continue;
      }

      factories_[i] = iter->second;
      lookup_[names[i]] = i;
    }
  }

  ~AttributeFactory() = default;

  typename T::Ptr create(uint8_t type_id) const {
    auto iter = factories_.find(type_id);
    if (iter == factories_.end()) {
      return nullptr;
    }

    return iter->second ? iter->second() : nullptr;
  }

  typename T::Ptr create(const std::string& name) const {
    auto iter = lookup_.find(name);
    if (iter == lookup_.end()) {
      return nullptr;
    }

    return create(iter->second);
  }

 private:
  std::map<std::string, uint8_t> lookup_;
  std::map<uint8_t, Constructor> factories_;
};

template <typename Attrs>
class AttributeRegistry {
 public:
  static AttributeRegistry<Attrs>& instance();

  ~AttributeRegistry() = default;

  template <typename T>
  static size_t addAttributes(const std::string& name);

  static AttributeFactory<Attrs> current();

  static AttributeFactory<Attrs> fromNames(const std::vector<std::string>& names);

  static const std::vector<std::string>& names() {
    auto& registry = instance();
    return registry.names_;
  }

 private:
  AttributeRegistry() {}

  std::vector<std::string> names_;
  std::map<std::string, std::function<std::unique_ptr<Attrs>()>> factories_;
  inline static std::unique_ptr<AttributeRegistry<Attrs>> s_instance_ = nullptr;
};

template <typename Attrs>
AttributeRegistry<Attrs>& AttributeRegistry<Attrs>::instance() {
  if (!s_instance_) {
    s_instance_.reset(new AttributeRegistry<Attrs>());
  }

  return *s_instance_;
}

template <typename Attrs>
template <typename T>
size_t AttributeRegistry<Attrs>::addAttributes(const std::string& name) {
  auto& registry = instance();
  if (registry.factories_.count(name)) {
    throw std::runtime_error("Registering two node attributes under '" + name + "'");
  }

  const auto index = registry.names_.size();
  registry.names_.push_back(name);
  registry.factories_[name] = []() { return std::make_unique<T>(); };
  return index;
}

template <typename Attrs>
AttributeFactory<Attrs> AttributeRegistry<Attrs>::current() {
  auto& registry = instance();
  return AttributeFactory<Attrs>(registry.names_, registry.factories_);
}

template <typename T>
AttributeFactory<T> AttributeRegistry<T>::fromNames(
    const std::vector<std::string>& names) {
  auto& registry = instance();
  return AttributeFactory<T>(names, registry.factories_);
}

struct RegistrationInfo {
  std::string name;
  uint8_t type_id;
};

template <typename Attrs, typename T>
struct AttributeRegistration {
  explicit AttributeRegistration(const std::string& name);
  RegistrationInfo info;
};

template <typename Attrs, typename T>
AttributeRegistration<Attrs, T>::AttributeRegistration(const std::string& name) {
  static_assert(std::is_base_of<Attrs, T>::value,
                "Derived attributes must have base NodeAttributes");
  info = {
      name,
      static_cast<uint8_t>(AttributeRegistry<Attrs>::template addAttributes<T>(name))};
}

}  // namespace spark_dsg::serialization
