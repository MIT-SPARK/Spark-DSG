#pragma once
#include "kimera_dsg/bounding_box.h"
#include "kimera_dsg/scene_graph_node.h"

#include <ostream>
#include <string>

namespace kimera {

// TODO(nathan) handle this better (kimera semantics is uint8, but...)
/**
 * @brief Typedef representing the semantic class of an object or other node
 */
typedef uint8_t SemanticLabel;

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
   * @brief Make a default set of attributes
   */
  SemanticNodeAttributes();

  virtual ~SemanticNodeAttributes() = default;

  //! Name of the node
  std::string name = "";
  //! Color of the node (if it exists)
  ColorVector color;
  //! Extents of the node (if they exists)
  BoundingBox bounding_box;
  //! semantic label of object
  SemanticLabel semantic_label;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
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

  //! Whether or not the object is known (and registered)
  bool registered;
  //! rotation of object w.r.t. world (only valid when registerd)
  Eigen::Quaterniond world_R_object;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
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

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
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

  /**
   * @brief make places node attributes
   * @param distance distance to nearest obstalce
   * @param num_basis_points number of basis points of the places node
   */
  PlaceNodeAttributes(double distance, unsigned int num_basis_points);

  virtual ~PlaceNodeAttributes() = default;

  //! distance to nearest obstacle
  double distance;
  //! number of equidistant obstacles
  unsigned int num_basis_points;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

}  // namespace kimera
