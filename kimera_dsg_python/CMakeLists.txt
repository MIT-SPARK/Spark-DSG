cmake_minimum_required(VERSION 3.1)
project(kimera_dsg_python)

add_compile_options(-Wall -Wextra)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED)
find_package(kimera_dsg REQUIRED)

add_subdirectory(third_party/pybind11)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}_bindings
)

pybind11_add_module(${PROJECT_NAME}_bindings bindings/kimera_dsg_bindings.cpp)
target_link_libraries(${PROJECT_NAME}_bindings PUBLIC kimera_dsg::kimera_dsg)
# TODO(nathan) this is brittle (depends on libkimera_dsg.so being in the devel
# workspace)
set_target_properties(
  ${PROJECT_NAME}_bindings
  PROPERTIES LIBRARY_OUTPUT_DIRECTORY
             ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
