cmake_minimum_required(VERSION 3.14)
project(spark_dsg VERSION 1.0.4)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

option(SPARK_DSG_BUILD_EXAMPLES "Build examples" ON)
option(SPARK_DSG_BUILD_TESTS "Build tests" ON)
option(SPARK_DSG_BUILD_PYTHON "Build python bindings" ON)
option(SPARK_DSG_BUILD_ZMQ "Build zmq message interface" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)

configure_file(cmake/spark_dsg_version.h.in include/spark_dsg_version.h)

find_package(nlohmann_json REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(zmq libzmq)

add_library(
  ${PROJECT_NAME}
  src/adjacency_matrix.cpp
  src/bounding_box_extraction.cpp
  src/bounding_box.cpp
  src/color.cpp
  src/dynamic_scene_graph_layer.cpp
  src/dynamic_scene_graph.cpp
  src/edge_attributes.cpp
  src/edge_container.cpp
  src/layer_view.cpp
  src/mesh.cpp
  src/node_attributes.cpp
  src/node_symbol.cpp
  src/scene_graph_layer.cpp
  src/scene_graph_logger.cpp
  src/scene_graph_node.cpp
  src/scene_graph_types.cpp
  src/scene_graph_utilities.cpp
  src/serialization/attribute_serialization.cpp
  src/serialization/binary_conversions.cpp
  src/serialization/binary_serialization.cpp
  src/serialization/file_io.cpp
  src/serialization/graph_binary_serialization.cpp
  src/serialization/graph_json_serialization.cpp
  src/serialization/json_conversions.cpp
  src/serialization/versioning.cpp
  src/serialization/mesh_serialization.cpp
)

if(NOT BUILD_SHARED_LIBS)
  set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json
  PRIVATE Threads::Threads
)

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${zmq_LIBRARIES})
  target_include_directories(${PROJECT_NAME} PRIVATE ${zmq_INCLUDE_DIRS})
  target_sources(
    ${PROJECT_NAME}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/zmq_interface.cpp>"
  )

  if(SPARK_DSG_BUILD_EXAMPLES)
    add_subdirectory(examples)
  endif()
endif()

if(SPARK_DSG_BUILD_PYTHON OR SPARK_DSG_BUILD_TESTS)
  add_subdirectory(python)
endif()

if(SPARK_DSG_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(
  spark_dsg::${PROJECT_NAME} ALIAS ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} EXPORT spark_dsg-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT spark_dsg-targets FILE spark_dsgTargets.cmake NAMESPACE spark_dsg::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfigVersion.cmake VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/spark_dsgConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg
)
