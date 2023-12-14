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
#include <chrono>
#include <list>
#include <memory>
#include <ostream>
#include <string>

#include "spark_dsg/bounding_box.h"
#include "spark_dsg/scene_graph_types.h"

namespace spark_dsg {

// TODO(nathan) handle this better (kimera semantics is uint8, but...)
/**
 * @brief Typedef representing the semantic class of an object or other node
 */
using SemanticLabel = uint8_t;

/**
 * @brief Base node attributes.
 *
 * All nodes have a pointer to node attributes (that contain most of the useful
 * information about the node). As every node has to be spatially consistent
 * with the quantity it represents, every node must have a position.
 */
struct NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired node pointer type
  using Ptr = std::unique_ptr<NodeAttributes>;

  //! Make a default set of attributes
  NodeAttributes();

  //! Set the node position
  explicit NodeAttributes(const Eigen::Vector3d& position);

  virtual ~NodeAttributes() = default;

  /**
   * @brief output attribute information
   * @param out output stream
   * @param attrs attributes to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs);

  virtual NodeAttributes::Ptr clone() const {
    return std::make_unique<NodeAttributes>(*this);
  }

  //! Position of the node
  Eigen::Vector3d position;
  //! last time the place was updated (while active)
  uint64_t last_update_time_ns;
  //! whether or not the node is in the active window
  bool is_active;

 protected:
  //! actually output information to the std::ostream
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

/**
 * @brief Base class for any node with additional semantic meaning.
 */
struct SemanticNodeAttributes : public NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief pointer type for node
   */
  using Ptr = std::unique_ptr<SemanticNodeAttributes>;
  /**
   * @brief alias between color type and Eigen vector of uint8_t
   */
  using ColorVector = Eigen::Matrix<uint8_t, 3, 1>;
  /**
   * @brief alias for semantic label
   */
  using Label = SemanticLabel;

  /**
   * @brief Make a default set of attributes
   */
  SemanticNodeAttributes();

  virtual ~SemanticNodeAttributes() = default;

  virtual NodeAttributes::Ptr clone() const override {
    return std::make_unique<SemanticNodeAttributes>(*this);
  }

  //! Name of the node
  std::string name;
  //! Color of the node (if it exists)
  ColorVector color;
  //! Extents of the node (if they exists)
  BoundingBox bounding_box;
  //! semantic label of object
  SemanticLabel semantic_label;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

/**
 * @brief Additional node attributes for an object
 *
 * In addition to the normal semantic properties, an object also potentially has
 * a pose, a collection of vertices that it is comprised of in the mesh, and a
 * bounding box.
 */
struct ObjectNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief desired pointer type of node
   */
  using Ptr = std::unique_ptr<ObjectNodeAttributes>;
  /**
   * @brief color type for node
   */
  using ColorVector = SemanticNodeAttributes::ColorVector;

  /**
   * @brief Make a default set of attributes
   */
  ObjectNodeAttributes();

  virtual ~ObjectNodeAttributes() = default;

  virtual NodeAttributes::Ptr clone() const override {
    return std::make_unique<ObjectNodeAttributes>(*this);
  }

  //! Mesh vertice connections
  std::list<size_t> mesh_connections;
  //! Whether or not the object is known (and registered)
  bool registered;
  //! rotation of object w.r.t. world (only valid when registerd)
  Eigen::Quaterniond world_R_object;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

/**
 * @brief Additional node attributes for a room
 * For now, a room has identical attributes to any semantic node,
 * but that may change
 */
struct RoomNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief desired pointer type of node
   */
  using Ptr = std::unique_ptr<RoomNodeAttributes>;
  /**
   * @brief color type for node
   */
  using ColorVector = SemanticNodeAttributes::ColorVector;

  /**
   * @brief Make a default set of attributes
   */
  RoomNodeAttributes();

  virtual ~RoomNodeAttributes() = default;

  virtual NodeAttributes::Ptr clone() const override {
    return std::make_unique<RoomNodeAttributes>(*this);
  }

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

/**
 * @brief Information related to place to mesh coorespondence
 */
struct NearestVertexInfo {
  int32_t block[3];
  double voxel_pos[3];
  size_t vertex;
};

/**
 * @brief Additional node attributes for a place
 * In addition to the normal semantic properties, a room has the minimum
 * distance to an obstacle and the number of basis points for that vertex in the
 * GVD
 */
struct PlaceNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired pointer type of node
  using Ptr = std::unique_ptr<PlaceNodeAttributes>;
  //! color type for node
  using ColorVector = SemanticNodeAttributes::ColorVector;

  PlaceNodeAttributes();

  /**
   * @brief make places node attributes
   * @param distance distance to nearest obstalce
   * @param num_basis_points number of basis points of the places node
   */
  PlaceNodeAttributes(double distance, unsigned int num_basis_points);

  virtual ~PlaceNodeAttributes() = default;

  virtual NodeAttributes::Ptr clone() const override {
    return std::make_unique<PlaceNodeAttributes>(*this);
  }

  //! distance to nearest obstacle
  double distance;
  //! number of equidistant obstacles
  unsigned int num_basis_points;
  //! voxblox mesh vertices that are closest to this place
  std::vector<NearestVertexInfo> voxblox_mesh_connections;
  //! pcl mesh vertices that are closest to this place
  std::vector<size_t> pcl_mesh_connections;
  //! semantic labels of parents
  std::vector<uint8_t> mesh_vertex_labels;
  //! deformation vertices that are closest to this place
  std::vector<size_t> deformation_connections;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

struct AgentNodeAttributes : public NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<AgentNodeAttributes>;
  using BowIdVector = Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>;

  AgentNodeAttributes();

  AgentNodeAttributes(const Eigen::Quaterniond& world_R_body,
                      const Eigen::Vector3d& world_P_body,
                      NodeId external_key);

  virtual ~AgentNodeAttributes() = default;

  virtual NodeAttributes::Ptr clone() const override {
    return std::make_unique<AgentNodeAttributes>(*this);
  }

  Eigen::Quaterniond world_R_body;

  NodeId external_key;

  BowIdVector dbow_ids;
  Eigen::VectorXf dbow_values;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const override;
};

}  // namespace spark_dsg
