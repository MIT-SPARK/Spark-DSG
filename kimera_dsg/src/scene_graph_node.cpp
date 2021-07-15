#include "kimera_dsg/scene_graph_node.h"
#include "kimera_dsg/node_symbol.h"
#include "kimera_dsg/serialization_helpers.h"

#include <glog/logging.h>

using nlohmann::json;

namespace kimera {

NodeAttributes::NodeAttributes() : position(Eigen::Vector3d::Zero()) {}

NodeAttributes::NodeAttributes(const Eigen::Vector3d& pos) : position(pos) {}

std::ostream& NodeAttributes::fill_ostream(std::ostream& out) const {
  out << "Attributes: " << std::endl
      << "  - Position : " << position.transpose() << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const NodeAttributes& attrs) {
  return attrs.fill_ostream(out);
}

json NodeAttributes::toJson() const {
  json to_return;
  REGISTER_JSON_ATTR_TYPE(NodeAttributes, to_return);
  to_return["position"] = position;
  return to_return;
}

void NodeAttributes::fillFromJson(const json& record) {
  position = record.at("position").get<decltype(position)>();
}

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerId layer_id,
                               NodeAttributes::Ptr&& attrs)
    : id(node_id), layer(layer_id), attributes_(std::move(attrs)), has_parent_(false) {}

std::ostream& SceneGraphNode::fill_ostream(std::ostream& out) const {
  out << "Node<id=" << NodeSymbol(id).getLabel() << ", layer=" << layer << ">";
  return out;
}

std::ostream& operator<<(std::ostream& out, const SceneGraphNode& node) {
  return node.fill_ostream(out);
}

}  // namespace kimera
