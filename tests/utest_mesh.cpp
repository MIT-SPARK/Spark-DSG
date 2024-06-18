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
#include <gtest/gtest.h>

#include <unordered_set>

#include "spark_dsg/mesh.h"

namespace spark_dsg {

inline Mesh createDummyMesh() {
  Mesh mesh(true, true, false);
  mesh.resizeVertices(12);
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    mesh.setPos(i, Eigen::Vector3f(i, i, i));
    mesh.setTimestamp(i, i);
  }
  for (size_t i = 0; i < mesh.numVertices(); i += 3) {
    mesh.faces.push_back({i, i + 1, i + 2});
  }
  return mesh;
}

TEST(MeshTests, EraseVertices) {
  Mesh mesh = createDummyMesh();
  mesh.eraseVertices({0, 1, 2, 3, 4, 6});
  EXPECT_EQ(mesh.points.size(), 6);
  EXPECT_EQ(mesh.colors.size(), 6);
  EXPECT_EQ(mesh.stamps.size(), 6);
  EXPECT_EQ(mesh.labels.size(), 0);
  EXPECT_EQ(mesh.faces.size(), 1);
  // Note that new indices should be shifted.
  EXPECT_EQ(mesh.faces[0][0], 3);
  EXPECT_EQ(mesh.faces[0][1], 4);
  EXPECT_EQ(mesh.faces[0][2], 5);
  EXPECT_EQ(mesh.timestamp(0), 5);
  EXPECT_EQ(mesh.timestamp(1), 7);
  EXPECT_EQ(mesh.timestamp(2), 8);
}

TEST(MeshTests, EraseFaces) {
  Mesh mesh = createDummyMesh();

  mesh.eraseFaces({0, 1}, false);
  EXPECT_EQ(mesh.points.size(), 12);
  EXPECT_EQ(mesh.colors.size(), 12);
  EXPECT_EQ(mesh.stamps.size(), 12);
  EXPECT_EQ(mesh.labels.size(), 0);
  EXPECT_EQ(mesh.faces.size(), 2);
  EXPECT_EQ(mesh.faces[0][0], 6);
  EXPECT_EQ(mesh.faces[0][1], 7);
  EXPECT_EQ(mesh.faces[0][2], 8);

  mesh.eraseFaces({0}, true);
  EXPECT_EQ(mesh.points.size(), 3);
  EXPECT_EQ(mesh.colors.size(), 3);
  EXPECT_EQ(mesh.stamps.size(), 3);
  EXPECT_EQ(mesh.labels.size(), 0);
  EXPECT_EQ(mesh.faces.size(), 1);
  EXPECT_EQ(mesh.timestamp(0), 9);
  EXPECT_EQ(mesh.timestamp(1), 10);
  EXPECT_EQ(mesh.timestamp(2), 11);
}

TEST(MeshTests, transform) {
  Mesh mesh = createDummyMesh();

  // Identity transform
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  mesh.transform(transform);
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    EXPECT_EQ(mesh.pos(i), Eigen::Vector3f(i, i, i));
  }

  // Translation.
  transform.translation() = Eigen::Vector3f(1, 2, 3);
  mesh.transform(transform);
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    EXPECT_EQ(mesh.pos(i), Eigen::Vector3f(i + 1, i + 2, i + 3));
  }

  // Rotation.
  transform = Eigen::Isometry3f::Identity();
  transform.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()));
  mesh.transform(transform);
  for (size_t i = 0; i < mesh.numVertices(); ++i) {
    EXPECT_NEAR(
        (mesh.pos(i) - Eigen::Vector3f(-(i + 2.0f), i + 1, i + 3)).norm(), 0.0f, 1e-5);
  }
}

}  // namespace spark_dsg
