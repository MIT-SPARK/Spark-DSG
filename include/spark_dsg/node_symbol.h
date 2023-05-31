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
#include <ostream>
#include <sstream>

#include "spark_dsg/scene_graph_types.h"

namespace spark_dsg {

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

inline NodeSymbol operator"" _id(const char* str, size_t size) {
  if (size < 1) {
    throw std::domain_error("invalid literal: must have at least two characters");
  }

  char prefix = str[0];
  std::string number(str + 1, size - 1);
  size_t index = std::stoull(number);
  return NodeSymbol(prefix, index);
}

template <typename Container>
std::string displayNodeSymbolContainer(const Container& set) {
  std::stringstream ss;
  ss << "[";
  auto iter = set.begin();
  while (iter != set.end()) {
    ss << NodeSymbol(*iter).getLabel();
    ++iter;
    if (iter != set.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

}  // namespace spark_dsg
