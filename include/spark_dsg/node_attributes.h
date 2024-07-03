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
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <string>

#include "spark_dsg/bounding_box.h"
#include "spark_dsg/color.h"
#include "spark_dsg/mesh.h"
#include "spark_dsg/scene_graph_types.h"
#include "spark_dsg/serialization/attribute_registry.h"

namespace spark_dsg {
namespace serialization {
class Visitor;
}

struct NodeAttributes;

template <typename T>
using NodeAttributeRegistration =
    serialization::AttributeRegistration<NodeAttributes, T>;

#define REGISTER_NODE_ATTRIBUTES(attr_type)                                  \
  inline static const auto registration_ =                                   \
      NodeAttributeRegistration<attr_type>(#attr_type);                      \
  const serialization::RegistrationInfo& registrationImpl() const override { \
    return registration_.info;                                               \
  }                                                                          \
  static_assert(true, "")

// TODO(nathan) handle this better
/**
 * @brief Typedef representing the semantic class of an object or other node
 */
using SemanticLabel = uint32_t;

/**
 * @brief Information related to place to mesh coorespondence
 */
struct NearestVertexInfo {
  int32_t block[3];
  double voxel_pos[3];
  size_t vertex;
  std::optional<uint32_t> label;
};

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
  friend class serialization::Visitor;

  //! desired node pointer type
  using Ptr = std::unique_ptr<NodeAttributes>;

  //! Make a default set of attributes
  NodeAttributes();
  //! Set the node position
  explicit NodeAttributes(const Eigen::Vector3d& position);
  virtual ~NodeAttributes() = default;
  virtual NodeAttributes::Ptr clone() const;

  //! Position of the node
  Eigen::Vector3d position;
  //! last time the place was updated (while active)
  uint64_t last_update_time_ns;
  //! whether or not the node is in the active window
  bool is_active;
  //! whether the node was observed by Hydra, or added as a prediction
  bool is_predicted;

  /**
   * @brief output attribute information
   * @param out output stream
   * @param attrs attributes to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs);

  bool operator==(const NodeAttributes& other) const;

  const serialization::RegistrationInfo& registration() const {
    return registrationImpl();
  }

 protected:
  //! actually output information to the std::ostream
  virtual std::ostream& fill_ostream(std::ostream& out) const;
  //! dispatch function for serialization
  virtual void serialization_info();
  //! dispatch function for serialization
  void serialization_info() const;
  //! compute equality
  virtual bool is_equal(const NodeAttributes& other) const;

  inline static const auto registration_ =
      NodeAttributeRegistration<NodeAttributes>("NodeAttributes");

  //! get registration
  virtual const serialization::RegistrationInfo& registrationImpl() const {
    return registration_.info;
  }
};

/**
 * @brief Base class for any node with additional semantic meaning.
 */
struct SemanticNodeAttributes : public NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! pointer type for node
  using Ptr = std::unique_ptr<SemanticNodeAttributes>;

  //! alias for semantic label
  using Label = SemanticLabel;
  // !flag for whether or not semantic label should be considered valid
  inline static constexpr Label NO_SEMANTIC_LABEL = std::numeric_limits<Label>::max();

  SemanticNodeAttributes();
  virtual ~SemanticNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  bool hasLabel() const;
  bool hasFeature() const;

  //! Name of the node
  std::string name;
  //! Color of the node (if it exists)
  Color color;
  //! Extents of the node (if they exists)
  BoundingBox bounding_box;
  //! semantic label of object
  SemanticLabel semantic_label;
  //! semantic feature of object
  Eigen::MatrixXd semantic_feature;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(SemanticNodeAttributes);
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
  //! desired pointer type of node
  using Ptr = std::unique_ptr<ObjectNodeAttributes>;

  //! Make a default set of attributes
  ObjectNodeAttributes();
  virtual ~ObjectNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! Mesh vertice connections
  std::list<size_t> mesh_connections;
  //! Whether or not the object is known (and registered)
  bool registered;
  //! rotation of object w.r.t. world (only valid when registerd)
  Eigen::Quaterniond world_R_object;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(ObjectNodeAttributes);
};

/**
 * @brief Additional node attributes for a room
 * For now, a room has identical attributes to any semantic node,
 * but that may change
 */
struct RoomNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired pointer type of node
  using Ptr = std::unique_ptr<RoomNodeAttributes>;

  //!  Make a default set of attributes
  RoomNodeAttributes();
  virtual ~RoomNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  std::map<std::string, double> semantic_class_probabilities;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(RoomNodeAttributes);
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

  PlaceNodeAttributes();

  /**
   * @brief make places node attributes
   * @param distance distance to nearest obstalce
   * @param num_basis_points number of basis points of the places node
   */
  PlaceNodeAttributes(double distance, unsigned int num_basis_points);
  virtual ~PlaceNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

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

  bool real_place = true;
  bool need_cleanup = false;
  bool active_frontier = false;
  Eigen::Vector3d frontier_scale;
  Eigen::Quaterniond orientation;
  size_t num_frontier_voxels = 0;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(PlaceNodeAttributes);
};
using FrontierNodeAttributes = PlaceNodeAttributes;

/**
 * @brief Additional node attributes for a 2d (outdoor) place
 * In addition to the normal semantic properties, a 2d place has ...
 */
struct Place2dNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired pointer type of node
  using Ptr = std::unique_ptr<Place2dNodeAttributes>;

  Place2dNodeAttributes();

  /**
   * @brief make places node attributes
   * @param boundary Boundary points surrounding place
   */
  Place2dNodeAttributes(std::vector<Eigen::Vector3d> boundary);
  virtual ~Place2dNodeAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  //! points on boundary of place region
  std::vector<Eigen::Vector3d> boundary;
  //! center of intersection checking ellipsoid
  Eigen::Vector3d ellipse_centroid;
  //! shape matrix for intersection checking ellipsoid
  Eigen::Matrix<double, 2, 2> ellipse_matrix_compress;
  //! shape matrix for plotting ellipsoid
  Eigen::Matrix<double, 2, 2> ellipse_matrix_expand;
  //! pcl mesh vertices corresponding to boundary points
  std::vector<size_t> pcl_boundary_connections;
  //! voxblox mesh vertices that are closest to this place
  std::vector<NearestVertexInfo> voxblox_mesh_connections;
  //! pcl mesh vertices that are closest to this place
  std::vector<size_t> pcl_mesh_connections;
  //! min vertex index of associated mesh vertices
  size_t pcl_min_index;
  //! max vertex index of associated mesh vertices
  size_t pcl_max_index;
  //! semantic labels of parents
  std::vector<uint8_t> mesh_vertex_labels;
  //! deformation vertices that are closest to this place
  std::vector<size_t> deformation_connections;
  //! tracks whether the node still needs to be cleaned up during merging
  bool need_finish_merge;
  //! whether this node has been merged to while in current active window
  bool need_cleanup_splitting;
  //! whether this node has mesh vertices in active window
  bool has_active_mesh_indices;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(Place2dNodeAttributes);
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
  NodeAttributes::Ptr clone() const override;

  Eigen::Quaterniond world_R_body;
  NodeId external_key;
  BowIdVector dbow_ids;
  Eigen::VectorXf dbow_values;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(AgentNodeAttributes);
};

/**
 * @brief Attributes for khronos object nodes.
 */
struct KhronosObjectAttributes : public ObjectNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! desired pointer type of node
  using Ptr = std::unique_ptr<KhronosObjectAttributes>;

  KhronosObjectAttributes();
  virtual ~KhronosObjectAttributes() = default;
  NodeAttributes::Ptr clone() const override;

  // Attributes.
  // Sequence of observation starts and ends.
  std::vector<uint64_t> first_observed_ns;
  std::vector<uint64_t> last_observed_ns;

  // Mesh of the object. Positions of vertices are relative to the object bounding box
  // origin.
  Mesh mesh;

  // If the object is considered dynamic, store the trajectory of the object.
  // NOTE(lschmid): Currently dynamic and static objects just have the
  // khronos-attributes. Could change in the future.
  std::vector<uint64_t> trajectory_timestamps;
  std::vector<Eigen::Vector3f> trajectory_positions;
  // Store per frame the 3D dynamic points of the object in world frame.
  std::vector<std::vector<Eigen::Vector3f>> dynamic_object_points;

  // Optionally store additional detailed infos if needed.
  std::map<std::string, std::vector<size_t>> details;

 protected:
  std::ostream& fill_ostream(std::ostream& out) const override;
  void serialization_info() override;
  bool is_equal(const NodeAttributes& other) const override;
  // registers derived attributes
  REGISTER_NODE_ATTRIBUTES(KhronosObjectAttributes);
};

}  // namespace spark_dsg
