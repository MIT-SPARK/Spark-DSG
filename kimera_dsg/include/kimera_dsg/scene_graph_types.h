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
 * layer hierarchy: a higher number corresponds to parents.
 */
enum class KimeraDsgLayers : LayerId {
  INVALID = 0,
  MESH = 1,       //< Mesh layer
  OBJECTS = 2,    //< Object node layer (static) as well as agents
  AGENTS = 2,     //< Agents layer (dynamic)
  PLACES = 3,     //< Places node layer (as well as structure)
  STRUCTURE = 3,  //< Struct node layer (as well as places)
  ROOMS = 4,      //< Room node layer
  BUILDINGS = 5   //< Building node layer
};

// TODO(nathan) this is awkward to use, reconsider maybe
/**
 * @brief Coerce enum value to something that might be formatted correctly
 */
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

}  // namespace kimera
