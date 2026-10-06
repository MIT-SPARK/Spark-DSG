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
#include <ranges>

namespace spark_dsg::python {

template <typename T>
struct RangeWrapper {
 public:
  struct Sentinel {};

  virtual ~RangeWrapper() = default;
  virtual bool done() const = 0;
  virtual RangeWrapper& next() = 0;
  virtual T deref() const = 0;

  RangeWrapper& operator++() { return next(); }
  T operator*() const { return deref(); }
  bool operator==(const Sentinel&) const { return done(); }
  bool operator!=(const Sentinel&) const { return !(*this == Sentinel()); }
};

template <typename T, typename R>
struct RangeWrapperImpl : RangeWrapper<T> {
  using Iter = decltype(std::ranges::begin(std::declval<R>()));

  explicit RangeWrapperImpl(const R& _view);
  bool done() const override;
  T deref() const override;
  RangeWrapperImpl<T, R>& next() override;

  R view;
  Iter curr_;
  Iter end_;
};

template <typename T, typename R>
RangeWrapperImpl<T, R>::RangeWrapperImpl(const R& _view)
    : view(_view), curr_(std::ranges::begin(view)), end_(std::ranges::end(view)) {}

template <typename T, typename R>
bool RangeWrapperImpl<T, R>::done() const {
  return curr_ == end_;
}

template <typename T, typename R>
T RangeWrapperImpl<T, R>::deref() const {
  return *curr_;
}

template <typename T, typename R>
RangeWrapperImpl<T, R>& RangeWrapperImpl<T, R>::next() {
  ++curr_;
  return *this;
}

}  // namespace spark_dsg::python
