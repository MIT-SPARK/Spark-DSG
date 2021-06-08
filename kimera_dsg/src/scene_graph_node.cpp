#include "kimera_dsg/scene_graph_node.h"

#include <glog/logging.h>

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

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerId layer_id,
                               NodeAttributes::Ptr&& attrs)
    : id(node_id),
      layer(layer_id),
      attributes_(std::move(attrs)),
      has_parent_(false),
      siblings(siblings_),
      children(children_) {}

std::ostream& SceneGraphNode::fill_ostream(std::ostream& out) const {
  out << " Node <id=" << id << ", layer=" << layer << ">" << std::endl;
  return out;
}

std::ostream& operator<<(std::ostream& out, const SceneGraphNode& node) {
  return node.fill_ostream(out);
}

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
