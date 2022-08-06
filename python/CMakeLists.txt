cmake_minimum_required(VERSION 3.1)
project(spark_dsg_python)

# TODO(nathan) fetch content when possible on 20.04
configure_file(pybind11.CMakeLists.txt.in pybind11-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pybind11-download" OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/pybind11-download" OUTPUT_QUIET
)

add_subdirectory(
  ${CMAKE_CURRENT_BINARY_DIR}/pybind11-src ${CMAKE_CURRENT_BINARY_DIR}/pybind11-build
)

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
