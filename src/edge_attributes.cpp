#include "kimera_dsg/edge_attributes.h"

#include <iomanip>

namespace kimera {

EdgeAttributes::EdgeAttributes() : weighted(false), weight(1.0) {}

EdgeAttributes::EdgeAttributes(double weight) : weighted(true), weight(weight) {}

EdgeAttributes::~EdgeAttributes() = default;

std::ostream& operator<<(std::ostream& out, const EdgeAttributes& attrs) {
  out << "{";
  attrs.fill_ostream(out);
  out << "}";
  return out;
}

void EdgeAttributes::fill_ostream(std::ostream& out) const {
  out << std::boolalpha << "weighted: " << weighted << ", weight: " << weight;
}

EdgeAttributes::Ptr EdgeAttributes::clone() const {
  return std::make_unique<EdgeAttributes>(*this);
}

}  // namespace kimera
