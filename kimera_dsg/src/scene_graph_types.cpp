#include "kimera_dsg/scene_graph_types.h"

#include <algorithm>

namespace kimera {

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
