#pragma once
#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <ostream>
#include <string>

/**
 * @brief kimera namespace
 */
namespace kimera {

using NodeId = uint64_t;   //!< Node label
using LayerId = uint64_t;  //!< Layer label

struct LayerKey {
  static constexpr LayerId UNKNOWN_LAYER = std::numeric_limits<LayerId>::max();
  LayerId layer;
  uint32_t prefix = 0;
  bool dynamic = false;

  LayerKey();

  LayerKey(LayerId layer_id);

  LayerKey(LayerId layer_id, uint32_t prefix);

  bool isParent(const LayerKey& other) const;

  bool operator==(const LayerKey& other) const;

  inline bool operator!=(const LayerKey& other) const {
    return !this->operator==(other);
  }

  inline operator bool() const { return layer != UNKNOWN_LAYER; }
};

inline std::ostream& operator<<(std::ostream& out, const LayerKey& key) {
  if (key.dynamic) {
    out << key.layer << "(" << key.prefix << ")";
  } else {
    out << key.layer;
  }
  return out;
}

class LayerPrefix {
 public:
  LayerPrefix(char key);

  LayerPrefix(char key, uint32_t index);

  LayerPrefix(uint32_t index);

  static LayerPrefix fromId(NodeId node);

  inline operator uint32_t() const { return value_.value; }

  std::string str(bool with_key = true) const;

  bool matches(NodeId node) const;

  NodeId makeId(size_t index) const;

  size_t index(NodeId node_id) const;

 private:
  union {
    uint32_t value;
    struct __attribute__((packed)) {
      uint32_t index : 24;
      char key : 8;
    } symbol;
  } value_;
};

/**
 * @brief Layer enum hierarchy corresponding to original DSG paper(s)
 * @note A higher layer id corresponds to parents for interlayer edges
 */
struct KimeraDsgLayers {
  inline const static LayerId MESH = 1;     //< Mesh layer
  inline const static LayerId OBJECTS = 2;  //< Object node layer (static)
  inline const static LayerId AGENTS = 2;   //< Agents layer (dynamic)
  inline const static LayerId PLACES = 3;   //< Places node layer (as well as structure)
  inline const static LayerId STRUCTURE = 3;  //< Struct node layer (as well as places)
  inline const static LayerId ROOMS = 4;      //< Room node layer
  inline const static LayerId BUILDINGS = 5;  //< Building node layer
  inline const static LayerId UNKNOWN = LayerKey::UNKNOWN_LAYER;  //< Catchall layer ID

  static std::string LayerIdToString(LayerId id);
  static LayerId StringToLayerId(const std::string& id_str);
};

inline Eigen::IOFormat getDefaultVectorFormat() {
  return Eigen::IOFormat(
      Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");
}

}  // namespace kimera
