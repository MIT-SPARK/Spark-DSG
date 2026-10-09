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
#include <string>
#include <type_traits>
#include <vector>

#include "spark_dsg/serialization/registration_info.h"
#include "spark_dsg/spark_dsg_fwd.h"

namespace spark_dsg::serialization {

template <typename T>
class AttributeFactory {
 public:
  using Constructor = std::function<std::unique_ptr<T>()>;
  using FactoryMap = std::map<std::string, Constructor>;

  AttributeFactory(const std::vector<std::string>& names, const FactoryMap& factories);
  std::unique_ptr<T> create(uint8_t type_id) const;
  std::unique_ptr<T> create(const std::string& name) const;

 private:
  std::map<std::string, uint8_t> lookup_;
  std::map<uint8_t, Constructor> factories_;
};

template <typename Attrs>
class AttributeRegistry {
 public:
  static AttributeRegistry& instance();

  template <typename T>
  static size_t addAttributes(const std::string& name) {
    static_assert(std::is_base_of_v<Attrs, T>);
    return instance().add(name, [] { return std::make_unique<T>(); });
  }

  static AttributeFactory<Attrs> current();
  static AttributeFactory<Attrs> fromNames(const std::vector<std::string>& names);
  static const std::vector<std::string>& names();
  static const RegistrationInfo& registration(const std::string& name);

 private:
  AttributeRegistry();
  size_t add(const std::string& name,
             typename AttributeFactory<Attrs>::Constructor constructor);

  std::vector<std::string> names_;
  typename AttributeFactory<Attrs>::FactoryMap factories_;
  std::map<std::string, RegistrationInfo> registrations_;
};

template <typename Attrs, typename T>
struct AttributeRegistration {
  explicit AttributeRegistration(const std::string& name)
      : info{name,
             static_cast<uint8_t>(
                 AttributeRegistry<Attrs>::template addAttributes<T>(name))} {}
  RegistrationInfo info;
};

template <>
AttributeRegistry<NodeAttributes>::AttributeRegistry();

template <>
AttributeRegistry<EdgeAttributes>::AttributeRegistry();

extern template class AttributeFactory<NodeAttributes>;
extern template class AttributeFactory<EdgeAttributes>;
extern template class AttributeRegistry<NodeAttributes>;
extern template class AttributeRegistry<EdgeAttributes>;

}  // namespace spark_dsg::serialization
