#include "kimera_dsg/node_symbol.h"

namespace kimera {

NodeSymbol::NodeSymbol(char key, NodeId index) {
  value_.symbol.key = key;
  value_.symbol.index = index;
}

NodeSymbol::NodeSymbol(NodeId value) { value_.value = value; }

std::ostream& operator<<(std::ostream& out, const NodeSymbol& symbol) {
  out << symbol.value_.symbol.key << "(" << symbol.value_.symbol.index << ")";
  return out;
}

}  // namespace kimera
