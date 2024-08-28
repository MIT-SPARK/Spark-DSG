cmake_minimum_required(VERSION 3.16)
project(spark_dsg_python)

find_package(pybind11 REQUIRED)

pybind11_add_module(
  _dsg_bindings
  bindings/src/mesh_helpers.cpp
  bindings/src/python_layer_view.cpp
  bindings/src/scene_graph_iterators.cpp
  bindings/src/spark_dsg_bindings.cpp
)
target_link_libraries(_dsg_bindings PRIVATE spark_dsg)
# NOTE(nathan) required on 20.04 when compiling with clang
target_compile_options(_dsg_bindings PRIVATE -fsized-deallocation)
target_include_directories(
  _dsg_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include
)
