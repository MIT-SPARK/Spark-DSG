#pragma once
#include "kimera_dsg/scene_graph_types.h"
#include <sstream>
#include <ostream>

namespace kimera {

/**
 * @brief A more human readable way to specify node ids
 *
 * Modeled after gtsam::Symbol, this class uses a character and a index to
 * create a unique key for the node. This is accomplished by setting the first 8
 * MSB to the character and the other 56 LSB to the index (through a union
 * between a NodeId and a bitfield). For the most part, this class is
 * transparent and supports implicit casting to NodeId (i.e. you can use this
 * anywhere you need a NodeId).
 */
class NodeSymbol {
 public:
  //! Make a node symbol in a human readable form
  NodeSymbol(char key, NodeId idx);

  //! Make a node id directly from a node ID
  NodeSymbol(NodeId value);

  //! cast the symobl directly to a node ID
  inline operator NodeId() const { return value_.value; }

  /**
   * @brief Get the index of the node in the specific category
   * @returns x where (_, x) were the arguments to create the symbol
   */
  inline NodeId categoryId() const { return value_.symbol.index; }

  /**
   * @brief Get the category of the node
   * @returns c where (c, _) were the arguments to create the symbol
   */
  inline char category() const { return value_.symbol.key; }

  //! pre-increment the index portion of the symbol
  NodeSymbol& operator++() {
    value_.symbol.index++;
    return *this;
  }

  //! post-increment the index portion of the symbol
  NodeSymbol operator++(int) {
    NodeSymbol old = *this;
    value_.symbol.index++;
    return old;
  }

  //! get a string representation of the symbol
  inline std::string getLabel() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  /**
   * @brief output node symbol information
   * @param out output stream
   * @param symbol symbol to print
   * @returns original output stream
   */
  friend std::ostream& operator<<(std::ostream& out, const NodeSymbol& symbol);

 private:
  union {
    NodeId value;
    struct __attribute__((packed)) {
      NodeId index : 56;
      char key : 8;
    } symbol;
  } value_;
};

}  // namespace kimera
