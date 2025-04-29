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
#include "spark_dsg/node_symbol.h"

#include <ostream>
#include <sstream>

namespace spark_dsg {

NodeSymbol::NodeSymbol(char key, NodeId index) {
  value_.symbol.key = key;
  value_.symbol.index = index;
}

NodeSymbol::NodeSymbol(NodeId value) { value_.value = value; }

NodeSymbol::operator NodeId() const { return value_.value; }

NodeId NodeSymbol::categoryId() const { return value_.symbol.index; }

char NodeSymbol::category() const { return value_.symbol.key; }

NodeSymbol& NodeSymbol::operator++() {
  value_.symbol.index++;
  return *this;
}

NodeSymbol NodeSymbol::operator++(int) {
  NodeSymbol old = *this;
  value_.symbol.index++;
  return old;
}

std::string NodeSymbol::str(bool literal) const {
  if (literal) {
    const auto idx = std::to_string(value_.symbol.index);
    return std::isalpha(value_.symbol.key) ? value_.symbol.key + idx : idx;
  }

  std::stringstream ss;
  ss << *this;
  return ss.str();
}

std::string NodeSymbol::getLabel() const { return str(); }

NodeSymbol operator"" _id(const char* str, size_t size) {
  if (size < 1) {
    throw std::domain_error("invalid literal: must have at least two characters");
  }

  char prefix = str[0];
  std::string number(str + 1, size - 1);
  size_t index = std::stoull(number);
  return NodeSymbol(prefix, index);
}

std::ostream& operator<<(std::ostream& out, const NodeSymbol& symbol) {
  if (std::isalpha(symbol.value_.symbol.key)) {
    out << symbol.value_.symbol.key << "(" << symbol.value_.symbol.index << ")";
  } else {
    out << symbol.value_.value;
  }
  return out;
}

}  // namespace spark_dsg
