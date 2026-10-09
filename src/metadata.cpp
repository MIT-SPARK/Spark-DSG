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
#include "spark_dsg/metadata.h"

#include <nlohmann/json.hpp>

namespace spark_dsg {

using nlohmann::json;

namespace {

void updateNested(json& to_update, const json& to_add) {
  if (!to_add.is_object()) {
    to_update = to_add;
    return;
  }

  for (const auto& [key, value] : to_add.items()) {
    auto iter = to_update.find(key);
    if (iter == to_update.end()) {
      to_update[key] = value;
    } else {
      updateNested(*iter, value);
    }
  }
}

}  // namespace

struct Metadata::Impl {
  json contents;
};

Metadata::Metadata() = default;

Metadata::~Metadata() = default;

Metadata::Metadata(const Metadata& other) {
  if (other.impl_) {
    impl_ = std::make_unique<Impl>(*other.impl_);
  }
}

Metadata& Metadata::operator=(const Metadata& other) {
  if (this == &other) {
    return *this;
  }

  set(other.get());
  return *this;
}

Metadata::Metadata(Metadata&& other) noexcept = default;

Metadata& Metadata::operator=(Metadata&& other) noexcept = default;

Metadata::Metadata(const json& contents)
    : impl_(std::make_unique<Impl>(Impl{contents})) {}

const json& Metadata::get() const {
  if (impl_) {
    return impl_->contents;
  }

  static const auto empty = json::object();
  return empty;
}

void Metadata::set(const json& contents) {
  if (!impl_) {
    impl_ = std::make_unique<Impl>(Impl{contents});
    return;
  }

  impl_->contents = contents;
}

void Metadata::add(const json& contents) {
  if (!impl_) {
    impl_ = std::make_unique<Impl>(Impl{json::object()});
  }

  updateNested(impl_->contents, contents);
}

void Metadata::add(const Metadata& other) { add(other.get()); }

size_t Metadata::memoryUsage() const {
  // Keep the existing serialized-size estimate for the JSON payload.
  const auto total_size = sizeof(Metadata) + (impl_ ? sizeof(Impl) : 0);
  return empty() ? total_size : total_size + get().dump().size();
}

void Metadata::clear() {
  if (impl_) {
    impl_->contents.clear();
  }
}

bool Metadata::empty() const { return !impl_ || impl_->contents.empty(); }

}  // namespace spark_dsg
