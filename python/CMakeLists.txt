cmake_minimum_required(VERSION 3.16)
project(spark_dsg_python)

include(FetchContent)
FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git)
FetchContent_GetProperties(pybind11)
if(NOT pybind11_POPULATED)
  FetchContent_Populate(pybind11)
endif()

add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR} EXCLUDE_FROM_ALL)

pybind11_add_module(
  _dsg_bindings
  bindings/src/mesh_helpers.cpp
  bindings/src/python_layer_view.cpp
  bindings/src/scene_graph_iterators.cpp
  bindings/src/spark_dsg_bindings.cpp
)
target_link_libraries(_dsg_bindings PRIVATE spark_dsg)
target_include_directories(
  _dsg_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings/include
)
