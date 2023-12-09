cmake_minimum_required(VERSION 3.16)
project(spark_dsg_python)

include(FetchContent)
FetchContent_Declare(pybind11 GIT_REPOSITORY https://github.com/pybind/pybind11.git)

FetchContent_GetProperties(pybind11)
if(NOT googletest_POPULATED)
  FetchContent_Populate(pybind11)
endif()

add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR} EXCLUDE_FROM_ALL)

set(ZMQ_INTERFACE_ENABLED 0)
if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  set(ZMQ_INTERFACE_ENABLED 1)
endif()
configure_file(bindings/zmq_bindings.h.in bindings/zmq_bindings.h)
pybind11_add_module(
  _dsg_bindings bindings/spark_dsg_bindings.cpp bindings/zmq_bindings.cpp
)
target_link_libraries(_dsg_bindings PRIVATE spark_dsg)
target_include_directories(
  _dsg_bindings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bindings
                        ${CMAKE_CURRENT_BINARY_DIR}/bindings
)
