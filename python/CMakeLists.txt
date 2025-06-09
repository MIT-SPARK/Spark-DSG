find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(
  _dsg_bindings
  bindings/src/attributes.cpp
  bindings/src/bounding_box.cpp
  bindings/src/color.cpp
  bindings/src/graph_types.cpp
  bindings/src/mesh.cpp
  bindings/src/metadata.cpp
  bindings/src/python_layer_view.cpp
  bindings/src/python_types.cpp
  bindings/src/scene_graph.cpp
  bindings/src/scene_graph_iterators.cpp
  bindings/src/scene_graph_layer.cpp
  bindings/src/spark_dsg_bindings.cpp
  bindings/src/spark_types.cpp
)
target_link_libraries(_dsg_bindings PRIVATE spark_dsg)
target_include_directories(_dsg_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include)

install(TARGETS _dsg_bindings DESTINATION ${PROJECT_NAME})
