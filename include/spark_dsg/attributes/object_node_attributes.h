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

#include <list>

#include "spark_dsg/attributes/semantic_node_attributes.h"

namespace spark_dsg {

/**
 * @brief Additional node attributes for an object
 *
 * In addition to the normal semantic properties, an object also potentially has
 * a pose, a collection of vertices that it is comprised of in the mesh, and a
 * bounding box.
 */
struct ObjectNodeAttributes : public SemanticNodeAttributes {
 public:
  //! desired pointer type of node
  using Ptr = std::unique_ptr<ObjectNodeAttributes>;

  //! Make a default set of attributes
  ObjectNodeAttributes();
  virtual ~ObjectNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;
  void transform(const Eigen::Isometry3d& transform) override;

  //! Mesh vertex connections
  std::list<size_t> mesh_connections;
  //! Whether or not the object is known (and registered)
  bool registered;
  //! Rotation of object with respect to world (only valid when registered)
  Eigen::Quaterniond world_R_object;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;

  const serialization::RegistrationInfo& registrationImpl() const override;
};

}  // namespace spark_dsg
