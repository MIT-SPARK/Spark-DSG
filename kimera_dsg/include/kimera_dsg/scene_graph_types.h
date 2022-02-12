#pragma once
#include <cstdint>
#include <string>

/**
 * @brief kimera namespace
 */
namespace kimera {

using NodeId = uint64_t;     //!< Node label
using LayerId = uint64_t;    //!< Layer label
using Timestamp = uint64_t;  //!< Timestamp type

/**
 * @brief Layer enum hierarchy corresponding to original DSG paper(s)
 * @note The numbering is important, as it determines the
 * layer hierarchy: a higher layer id corresponds to nodes being labeled as parents for
 * interlayer edges
 */
struct KimeraDsgLayers {
  inline const static LayerId MESH = 1;       //< Mesh layer
  inline const static LayerId OBJECTS = 2;    //< Object node layer (static) as well as agents
  inline const static LayerId AGENTS = 2;     //< Agents layer (dynamic)
  inline const static LayerId PLACES = 3;     //< Places node layer (as well as structure)
  inline const static LayerId STRUCTURE = 3;  //< Struct node layer (as well as places)
  inline const static LayerId ROOMS = 4;      //< Room node layer
  inline const static LayerId BUILDINGS = 5;  //< Building node layer
  inline const static LayerId UNKNOWN = 0;

  static std::string LayerIdToString(LayerId id);
  static LayerId StringToLayerId(const std::string& id_str);
};

}  // namespace kimera
