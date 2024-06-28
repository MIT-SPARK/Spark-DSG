#pragma once
#include <memory>

namespace spark_dsg {
struct NodeAttributes;

class DynamicSceneGraph;
using DynamicSceneGraphPtr = std::shared_ptr<DynamicSceneGraph>;
}  // namespace spark_dsg
