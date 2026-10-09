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
#include <optional>
#include <vector>

#include "spark_dsg/attributes/semantic_node_attributes.h"

namespace spark_dsg {

/**
 * @brief Information related to place to mesh correspondence
 */
struct NearestVertexInfo {
  int32_t block[3];
  double voxel_pos[3];
  size_t vertex;
  std::optional<uint32_t> label;
};

/**
 * @brief Additional node attributes for a place
 * In addition to the normal semantic properties, a place has the minimum
 * distance to an obstacle and the number of basis points for that vertex
 */
struct PlaceNodeAttributes : public SemanticNodeAttributes {
 public:
  //! desired pointer type of node
  using Ptr = std::unique_ptr<PlaceNodeAttributes>;

  PlaceNodeAttributes();

  /**
   * @brief make places node attributes
   * @param distance distance to nearest obstacle
   * @param num_basis_points number of basis points of the places node
   */
  PlaceNodeAttributes(double distance, unsigned int num_basis_points);
  virtual ~PlaceNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! distance to nearest obstacle
  double distance;
  //! number of equidistant obstacles
  unsigned int num_basis_points;
  //! Mesh vertices that are closest to this place
  std::vector<NearestVertexInfo> voxblox_mesh_connections;
  //! Mesh vertices that are closest to this place
  std::vector<size_t> pcl_mesh_connections;
  //! semantic labels of parents
  std::vector<uint8_t> mesh_vertex_labels;
  //! Deformation vertices that are closest to this place
  std::vector<size_t> deformation_connections;

  bool real_place = true;
  bool need_cleanup = false;
  bool active_frontier = false;
  bool anti_frontier = false;
  Eigen::Vector3d frontier_scale;
  Eigen::Quaterniond orientation;
  size_t num_frontier_voxels = 0;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;

  const serialization::RegistrationInfo& registrationImpl() const override;
};
using FrontierNodeAttributes = PlaceNodeAttributes;

/**
 * @brief Additional node attributes for a 2d (outdoor) place
 * In addition to the normal semantic properties, a 2d place has ...
 */
struct Place2dNodeAttributes : public SemanticNodeAttributes {
 public:
  //! desired pointer type of node
  using Ptr = std::unique_ptr<Place2dNodeAttributes>;

  Place2dNodeAttributes();
  virtual ~Place2dNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! Mesh vertices that are closest to this place
  std::list<size_t> mesh_connections;
  //! Mesh vertices corresponding to boundary points
  std::vector<size_t> boundary_connections;
  //! Points on boundary of place region
  std::vector<Eigen::Vector3d> boundary;
  //! Center of intersection checking ellipsoid
  Eigen::Vector3d ellipse_centroid;
  //! Shape matrix for intersection checking ellipsoid
  Eigen::Matrix<double, 2, 2> ellipse_matrix_compress;
  //! Shape matrix for plotting ellipsoid
  Eigen::Matrix<double, 2, 2> ellipse_matrix_expand;
  //! Minimum vertex index of associated mesh vertices
  size_t min_mesh_index;
  //! Max vertex index of associated mesh vertices
  size_t max_mesh_index;
  //! Tracks whether the node still needs to be cleaned up during merging
  bool need_finish_merge;
  //! Whether this node has mesh vertices in active window
  bool has_active_mesh_indices;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;

  const serialization::RegistrationInfo& registrationImpl() const override;
};

}  // namespace spark_dsg
