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
#include "spark_dsg/serialization/attribute_registry.h"

#include <limits>
#include <stdexcept>
#include <utility>

#include "spark_dsg/edge_attributes.h"
#include "spark_dsg/node_attributes.h"

namespace spark_dsg::serialization {

using std::make_unique;

template <typename T>
AttributeFactory<T>::AttributeFactory(const std::vector<std::string>& names,
                                      const FactoryMap& factories) {
  if (names.size() > size_t{std::numeric_limits<uint8_t>::max()} + 1) {
    throw std::length_error("Too many attribute types for serialized type IDs");
  }

  for (size_t i = 0; i < names.size(); ++i) {
    const auto iter = factories.find(names[i]);
    if (iter == factories.end()) {
      continue;
    }

    const auto id = static_cast<uint8_t>(i);
    factories_[id] = iter->second;
    lookup_[names[i]] = id;
  }
}

template <typename T>
std::unique_ptr<T> AttributeFactory<T>::create(uint8_t type_id) const {
  const auto iter = factories_.find(type_id);
  if (iter == factories_.end()) {
    return nullptr;
  }

  return iter->second ? iter->second() : nullptr;
}

template <typename T>
std::unique_ptr<T> AttributeFactory<T>::create(const std::string& name) const {
  const auto iter = lookup_.find(name);
  if (iter == lookup_.end()) {
    return nullptr;
  }

  return create(iter->second);
}

template <>
AttributeRegistry<NodeAttributes>::AttributeRegistry() {
  add("NodeAttributes", [] { return make_unique<NodeAttributes>(); });
  add("SemanticNodeAttributes", [] { return make_unique<SemanticNodeAttributes>(); });
  add("ObjectNodeAttributes", [] { return make_unique<ObjectNodeAttributes>(); });
  add("RoomNodeAttributes", [] { return make_unique<RoomNodeAttributes>(); });
  add("PlaceNodeAttributes", [] { return make_unique<PlaceNodeAttributes>(); });
  add("Place2dNodeAttributes", [] { return make_unique<Place2dNodeAttributes>(); });
  add("AgentNodeAttributes", [] { return make_unique<AgentNodeAttributes>(); });
  add("KhronosObjectAttributes", [] { return make_unique<KhronosObjectAttributes>(); });
  add("TraversabilityNodeAttributes",
      [] { return make_unique<TraversabilityNodeAttributes>(); });
  add("TravNodeAttributes", [] { return make_unique<TravNodeAttributes>(); });
}

template <>
AttributeRegistry<EdgeAttributes>::AttributeRegistry() {
  add("EdgeAttributes", [] { return make_unique<EdgeAttributes>(); });
}

template <typename Attrs>
AttributeRegistry<Attrs>& AttributeRegistry<Attrs>::instance() {
  static AttributeRegistry registry;
  return registry;
}

template <typename Attrs>
size_t AttributeRegistry<Attrs>::add(
    const std::string& name,
    typename AttributeFactory<Attrs>::Constructor constructor) {
  if (factories_.count(name)) {
    throw std::runtime_error("Registering two attribute types under '" + name + "'");
  }

  const auto index = names_.size();
  if (index > std::numeric_limits<uint8_t>::max()) {
    throw std::length_error("Too many attribute types for serialized type IDs");
  }

  names_.push_back(name);
  factories_.emplace(name, std::move(constructor));
  registrations_.emplace(name, RegistrationInfo{name, static_cast<uint8_t>(index)});
  return index;
}

template <typename Attrs>
AttributeFactory<Attrs> AttributeRegistry<Attrs>::current() {
  const auto& registry = instance();
  return AttributeFactory<Attrs>(registry.names_, registry.factories_);
}

template <typename Attrs>
AttributeFactory<Attrs> AttributeRegistry<Attrs>::fromNames(
    const std::vector<std::string>& names) {
  return AttributeFactory<Attrs>(names, instance().factories_);
}

template <typename Attrs>
const std::vector<std::string>& AttributeRegistry<Attrs>::names() {
  return instance().names_;
}

template <typename Attrs>
const RegistrationInfo& AttributeRegistry<Attrs>::registration(
    const std::string& name) {
  return instance().registrations_.at(name);
}

template class AttributeFactory<NodeAttributes>;
template class AttributeFactory<EdgeAttributes>;
template class AttributeRegistry<NodeAttributes>;
template class AttributeRegistry<EdgeAttributes>;

}  // namespace spark_dsg::serialization
