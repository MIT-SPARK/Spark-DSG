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
#include <memory>
#include <ostream>

#include "spark_dsg/serialization/attribute_registry.h"

namespace spark_dsg {
namespace serialization {
class Visitor;
}

struct EdgeAttributes;

template <typename T>
using EdgeAttributeRegistration =
    serialization::AttributeRegistration<EdgeAttributes, T>;

#define REGISTER_EDGE_ATTRIBUTES(attr_type)                                  \
  inline static const auto registration_ =                                   \
      EdgeAttributeRegistration<attr_type>(#attr_type);                      \
  const serialization::RegistrationInfo& registrationImpl() const override { \
    return registration_.info;                                               \
  }                                                                          \
  static_assert(true, "")

/**
 * @brief Collection of information for an edge
 */
struct EdgeAttributes {
  friend class serialization::Visitor;
  //! desired pointer type for the edge attributes
  using Ptr = std::unique_ptr<EdgeAttributes>;
  //! Default constructor resulting in an unweight edge
  EdgeAttributes();
  //! Constructor that make a weighted edge
  explicit EdgeAttributes(double weight);
  virtual ~EdgeAttributes();
  //! brief Get derived copy of edge attributes
  virtual EdgeAttributes::Ptr clone() const;

  //! whether or not the edge weight is valid
  bool weighted;
  //! the weight of the edge
  double weight;

  /**
   * @brief output attribute information
   * @param out output stream
   * @param attrs attributes to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const EdgeAttributes& attrs);

  bool operator==(const EdgeAttributes& other) const;

  const serialization::RegistrationInfo& registration() const {
    return registrationImpl();
  }

 protected:
  //! actually output information to the std::ostream
  virtual void fill_ostream(std::ostream& out) const;
  //! register serialization information about the attributes
  virtual void serialization_info();
  virtual void serialization_info() const;
  //! compute equality
  virtual bool is_equal(const EdgeAttributes& other) const;

  inline static const auto registration_ =
      EdgeAttributeRegistration<EdgeAttributes>("EdgeAttributes");

  //! get registration
  virtual const serialization::RegistrationInfo& registrationImpl() const {
    return registration_.info;
  }
};

}  // namespace spark_dsg
