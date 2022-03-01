#include "kimera_dsg/scene_graph_node.h"
#include "kimera_dsg/node_attributes.h"
#include "kimera_dsg/node_symbol.h"

namespace kimera {

SceneGraphNode::SceneGraphNode(NodeId node_id,
                               LayerId layer_id,
                               SceneGraphNode::AttributesPtr&& attrs)
    : id(node_id), layer(layer_id), attributes_(std::move(attrs)), has_parent_(false) {}

SceneGraphNode::~SceneGraphNode() = default;

std::ostream& SceneGraphNode::fill_ostream(std::ostream& out) const {
  out << "Node<id=" << NodeSymbol(id).getLabel() << ", layer=" << layer << ">";
  return out;
}

std::ostream& operator<<(std::ostream& out, const SceneGraphNode& node) {
  return node.fill_ostream(out);
}

}  // namespace kimera
