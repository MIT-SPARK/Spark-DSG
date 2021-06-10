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
  OBJECTS = 1, //< Object node layer (static)
  AGENTS = 2, //< Agent node layer (dynamic)
  PLACES = 3, //< Places node layer (as well as structure)
  ROOMS = 4, //< Room node layer
  BUILDINGS = 5 //< Building node layer
};

// TODO(nathan) this belongs in the builder probably
/**
 * @brief helper function to with layer id to string correspondence
 */
inline std::string getStringFromLayerId(KimeraDsgLayers layer_id) {
  switch (layer_id) {
    case KimeraDsgLayers::BUILDINGS:
      return "B";
    case KimeraDsgLayers::ROOMS:
      return "R";
    case KimeraDsgLayers::PLACES:
      return "P";
    case KimeraDsgLayers::OBJECTS:
      return "O";
    default:
      return "NA";
  }
}

/**
 * @brief Coerce enum value to something that might be formatted correctly
 */
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

}  // namespace kimera
