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
#include <Eigen/Dense>

#include "spark_dsg/serialization/binary_serialization.h"

namespace spark_dsg {

struct BoundingBox;
void read_binary(const serialization::BinaryDeserializer& s, BoundingBox& box);
void write_binary(serialization::BinarySerializer& s, const BoundingBox& box);

struct NearestVertexInfo;
void read_binary(const serialization::BinaryDeserializer& s, NearestVertexInfo& info);
void write_binary(serialization::BinarySerializer& s, const NearestVertexInfo& info);

struct Color;
void read_binary(const serialization::BinaryDeserializer& s, Color& c);
void write_binary(serialization::BinarySerializer& s, const Color& c);

class Mesh;
void read_binary(const serialization::BinaryDeserializer& s, Mesh& mesh);
void write_binary(serialization::BinarySerializer& s, const Mesh& mesh);

struct NodeAttributes;
void write_binary(serialization::BinarySerializer& s, const NodeAttributes& attrs);

struct EdgeAttributes;
void write_binary(serialization::BinarySerializer& s, const EdgeAttributes& attrs);

namespace io {

struct FileHeader;
void read_binary(const serialization::BinaryDeserializer& s, FileHeader& header);
void write_binary(serialization::BinarySerializer& s, const FileHeader& header);

}  // namespace io

}  // namespace spark_dsg

namespace Eigen {

template <typename Derived>
void write_binary(spark_dsg::serialization::BinarySerializer& s,
                  const MatrixBase<Derived>& matrix) {
  s.startFixedArray(matrix.size() + 2);
  s.write(matrix.rows());
  s.write(matrix.cols());
  for (Index r = 0; r < matrix.rows(); ++r) {
    for (Index c = 0; c < matrix.cols(); ++c) {
      s.write(matrix(r, c));
    }
  }
}

template <typename Derived>
size_t read_binary(const spark_dsg::serialization::BinaryDeserializer& s,
                   MatrixBase<Derived>& matrix) {
  const size_t length = s.readFixedArrayLength();
  if (length < 2u) {
    throw std::out_of_range("array dimensions not present");
  }

  Index rows, cols;
  s.read(rows);
  s.read(cols);
  if (rows * cols + 2 != static_cast<int64_t>(length)) {
    throw std::out_of_range("array dimensions do not match length");
  }

  matrix.derived().resize(rows, cols);
  // TODO(nathan) warn about row/col mismatch
  for (Index r = 0; r < matrix.rows(); ++r) {
    for (Index c = 0; c < matrix.cols(); ++c) {
      s.read(matrix(r, c));
    }
  }
  return 0;
}

// Specialize vector3 since this is a frequently used type.
template <typename Scalar>
void write_binary(spark_dsg::serialization::BinarySerializer& s,
                  const Matrix<Scalar, 3, 1>& vector3) {
  s.write(vector3.x());
  s.write(vector3.y());
  s.write(vector3.z());
}

template <typename Scalar>
void read_binary(const spark_dsg::serialization::BinaryDeserializer& s,
                 Matrix<Scalar, 3, 1>& vector3) {
  s.read(vector3.x());
  s.read(vector3.y());
  s.read(vector3.z());
}

template <typename Scalar>
void write_binary(spark_dsg::serialization::BinarySerializer& s,
                  const Quaternion<Scalar>& q) {
  s.startFixedArray(4);
  s.write(q.w());
  s.write(q.x());
  s.write(q.y());
  s.write(q.z());
}

template <typename Scalar>
void read_binary(const spark_dsg::serialization::BinaryDeserializer& s,
                 Quaternion<Scalar>& q) {
  s.checkFixedArrayLength(4);
  s.read(q.w());
  s.read(q.x());
  s.read(q.y());
  s.read(q.z());
}

}  // namespace Eigen
