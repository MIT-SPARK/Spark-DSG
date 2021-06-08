#include "kimera_dsg/node_attributes.h"

namespace kimera {

SemanticNodeAttributes::SemanticNodeAttributes()
    : NodeAttributes(),
      name(""),
      color(ColorVector::Zero()),
      semantic_label(0u) {}

std::ostream& SemanticNodeAttributes::fill_ostream(std::ostream& out) const {
  // TODO(nathan) think about printing out rotation here
  NodeAttributes::fill_ostream(out);
  out << "  - Color : " << color << std::endl
      << "  - Name: " << name << std::endl
      << "  - Bounding Box: " << bounding_box << std::endl
      << "  - Semantic Label: " << std::to_string(semantic_label) << std::endl;
  return out;
}

ObjectNodeAttributes::ObjectNodeAttributes()
    : SemanticNodeAttributes(), points(nullptr) {}

std::ostream& ObjectNodeAttributes::fill_ostream(std::ostream& out) const {
  // TODO(nathan) think about printing out rotation here
  SemanticNodeAttributes::fill_ostream(out);
  out << "  - Is Registered?: " << (registered ? "yes" : "no") << std::endl;
  return out;
}

RoomNodeAttributes::RoomNodeAttributes()
    : SemanticNodeAttributes(), points(nullptr) {}

std::ostream& RoomNodeAttributes::fill_ostream(std::ostream& out) const {
  SemanticNodeAttributes::fill_ostream(out);
  return out;
}

}  // namespace kimera
