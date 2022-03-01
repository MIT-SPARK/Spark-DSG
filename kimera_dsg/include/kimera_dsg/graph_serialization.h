#pragma once
#include "kimera_dsg/attribute_serialization.h"

namespace nlohmann {

template <>
struct adl_serializer<SceneGraphNode> {
  static SceneGraphNode from_json(const json& record) {
    auto node_id = node.at("id").get<NodeId>();
    auto layer = node.at("layer").get<LayerId>();
    auto attrs = node_factory.create(node.at("attributes"));
    return {node_id, layer, std::move(attrs)};
  }

  static void to_json(json& record, SceneGraphNode node) {
    record = {{"id", node.id},
              {"layer", node.layer},
              {"attributes", attributes::to_json(node.attributes())}};
  }
};

}  // namespace nlohmann
