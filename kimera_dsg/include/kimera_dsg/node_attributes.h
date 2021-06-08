#pragma once
#include "kimera_dsg/bounding_box.h"
#include "kimera_dsg/pcl_types.h"
#include "kimera_dsg/scene_graph_node.h"

#include <ostream>
#include <string>

namespace kimera {

// TODO(nathan) handle this better (kimera semantics is uint8, but...)
typedef uint8_t SemanticLabel;

struct SemanticNodeAttributes : public NodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<SemanticNodeAttributes>;
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

struct ObjectNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<ObjectNodeAttributes>;
  using ColorVector = SemanticNodeAttributes::ColorVector;

  /**
   * @brief Make a default set of attributes
   */
  ObjectNodeAttributes();

  virtual ~ObjectNodeAttributes() = default;

  //! Whether or not the object is known (and registered)
  bool registered;
  //! rotation of object w.r.t. world (makes world_T_object with position)
  Eigen::Quaterniond world_R_object;
  //! Point cloud that comprises object (will be null for registered objects)
  ColorPointCloud::Ptr points;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

struct RoomNodeAttributes : public SemanticNodeAttributes {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::unique_ptr<RoomNodeAttributes>;
  using ColorVector = SemanticNodeAttributes::ColorVector;

  /**
   * @brief Make a default set of attributes
   */
  RoomNodeAttributes();

  virtual ~RoomNodeAttributes() = default;

  //! Point cloud that comprises room
  ColorPointCloud::Ptr points;

 protected:
  virtual std::ostream& fill_ostream(std::ostream& out) const;
};

}  // namespace kimera
