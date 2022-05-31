#include "kimera_dsg/scene_graph_types.h"

#include <algorithm>
#include <sstream>

namespace kimera {

LayerPrefix::LayerPrefix(char key) {
  value_.symbol.key = key;
  value_.symbol.index = 0;
}

LayerPrefix::LayerPrefix(char key, uint32_t index) {
  value_.symbol.key = key;
  value_.symbol.index = index;
}

LayerPrefix::LayerPrefix(uint32_t index) { value_.value = index; }

LayerPrefix LayerPrefix::fromId(NodeId node_id) {
  // grab the 32 msb portion of the ID
  return LayerPrefix(static_cast<uint32_t>(node_id >> 32));
}

std::string LayerPrefix::str(bool with_key) const {
  if (!with_key) {
    return std::to_string(value_.value);
  }

  std::stringstream ss;
  ss << value_.symbol.key;
  if (value_.symbol.index) {
    ss << "(" << value_.symbol.index << ")";
  }

  return ss.str();
}

bool LayerPrefix::matches(NodeId node) const {
  return value_.value == static_cast<uint32_t>(node >> 32);
}

NodeId LayerPrefix::makeId(size_t index) const {
  return (static_cast<NodeId>(value_.value) << 32) + index;
}

size_t LayerPrefix::index(NodeId node_id) const {
  // grab the 32 lsb portion of the ID
  return 0xFFFF'FFFF & node_id;
}

LayerKey::LayerKey() : layer(LayerKey::UNKNOWN_LAYER) {}

LayerKey::LayerKey(LayerId layer_id) : layer(layer_id) {}

LayerKey::LayerKey(LayerId layer_id, uint32_t prefix)
    : layer(layer_id), prefix(prefix), dynamic(true) {}

bool LayerKey::operator==(const LayerKey& other) const {
  if (dynamic != other.dynamic) {
    return false;
  }

  const bool same_layer = layer == other.layer;
  if (!dynamic && same_layer) {
    return true;
  }

  return same_layer && prefix == other.prefix;
}

bool LayerKey::isParent(const LayerKey& other) const { return layer > other.layer; }

std::string KimeraDsgLayers::LayerIdToString(LayerId id) {
  switch (id) {
    case MESH:
      return "MESH";
    case OBJECTS:
      return "OBJECTS";  // we default to the static labels
    case PLACES:
      return "PLACES";
    case ROOMS:
      return "ROOMS";
    case BUILDINGS:
      return "BUILDINGS";
    default:
      return "UNKNOWN";
  }
}

LayerId KimeraDsgLayers::StringToLayerId(const std::string& id_str) {
  std::string to_check = id_str;
  std::transform(
      to_check.begin(), to_check.end(), to_check.begin(), [](unsigned char c) {
        return std::toupper(c);
      });
  if (to_check == "MESH") {
    return KimeraDsgLayers::MESH;
  } else if (to_check == "OBJECTS" || to_check == "AGENTS") {
    return KimeraDsgLayers::OBJECTS;
  } else if (to_check == "PLACES" || to_check == "STRUCTURE") {
    return KimeraDsgLayers::PLACES;
  } else if (to_check == "ROOMS") {
    return KimeraDsgLayers::ROOMS;
  } else if (to_check == "BUILDINGS") {
    return KimeraDsgLayers::BUILDINGS;
  }

  return KimeraDsgLayers::UNKNOWN;
}

}  // namespace kimera
