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
#include "spark_dsg/python/mesh_helpers.h"

namespace spark_dsg::python {

Eigen::MatrixXd getEigenVertices(const Mesh& mesh) {
  const auto num_vertices = mesh.numVertices();
  Eigen::MatrixXd to_return(6, num_vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    const auto& pos = mesh.pos(i);
    to_return(0, i) = pos.x();
    to_return(1, i) = pos.y();
    to_return(2, i) = pos.z();
    if (i < mesh.colors.size()) {
      const auto c = mesh.color(i);
      to_return(3, i) = c.r / 255.0;
      to_return(4, i) = c.g / 255.0;
      to_return(5, i) = c.b / 255.0;
    } else {
      to_return(3, i) = 0.0;
      to_return(4, i) = 0.0;
      to_return(5, i) = 0.0;
    }
  }
  return to_return;
}

void setEigenVertices(Mesh& mesh, const Eigen::MatrixXd& points) {
  if (points.rows() != 6) {
    std::stringstream ss;
    ss << "point rows do not match expected: " << points.rows() << " != 6";
    throw std::invalid_argument(ss.str());
  }

  mesh.resizeVertices(points.cols());
  for (int i = 0; i < points.cols(); ++i) {
    Eigen::Vector3f pos = points.col(i).head<3>().cast<float>();
    mesh.setPos(i, pos);
    if (mesh.has_colors) {
      Color color{static_cast<uint8_t>(points(3, i) * 255),
                  static_cast<uint8_t>(points(4, i) * 255),
                  static_cast<uint8_t>(points(5, i) * 255),
                  255};
      mesh.setColor(i, color);
    }
  }
}

Eigen::MatrixXi getEigenFaces(const Mesh& mesh) {
  const auto num_faces = mesh.numFaces();
  Eigen::MatrixXi to_return(3, num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    const auto& face = mesh.face(i);
    to_return(0, i) = face[0];
    to_return(1, i) = face[1];
    to_return(2, i) = face[2];
  }
  return to_return;
}

void setEigenFaces(Mesh& mesh, const Eigen::MatrixXi& indices) {
  if (indices.rows() != 3) {
    std::stringstream ss;
    ss << "index rows do not match expected: " << indices.rows() << " != 3";
    throw std::invalid_argument(ss.str());
  }

  mesh.resizeFaces(indices.cols());
  for (int i = 0; i < indices.cols(); ++i) {
    Mesh::Face face{{static_cast<size_t>(indices(0, i)),
                     static_cast<size_t>(indices(1, i)),
                     static_cast<size_t>(indices(2, i))}};
    mesh.face(i) = face;
  }
}

}  // namespace spark_dsg::python
