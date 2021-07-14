#include "kimera_dsg/attribute_serialization.h"

namespace kimera {

NodeAttributeFactory NodeAttributeFactory::Default() {
  NodeAttributeFactory factory;
  REGISTER_ATTR_FACTORY(factory, NodeAttributes);
  REGISTER_ATTR_FACTORY(factory, SemanticNodeAttributes);
  REGISTER_ATTR_FACTORY(factory, ObjectNodeAttributes);
  REGISTER_ATTR_FACTORY(factory, RoomNodeAttributes);
  REGISTER_ATTR_FACTORY(factory, PlaceNodeAttributes);
  return factory;
}

EdgeInfoFactory EdgeInfoFactory::Default() {
  EdgeInfoFactory factory;
  REGISTER_INFO_FACTORY(factory, SceneGraphEdgeInfo);
  return factory;
}

}  // namespace kimera
