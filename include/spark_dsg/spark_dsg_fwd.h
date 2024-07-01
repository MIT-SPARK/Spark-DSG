#pragma once
#include <memory>

namespace spark_dsg {
class Mesh;
struct NodeAttributes;
struct EdgeAttributes;
struct SceneGraphEdge;
struct EdgeContainer;

class DynamicSceneGraph;
using DynamicSceneGraphPtr = std::shared_ptr<DynamicSceneGraph>;

}  // namespace spark_dsg
