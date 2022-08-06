cmake_minimum_required(VERSION 3.10)
project(spark_dsg VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

option(SPARK_DSG_BUILD_TESTS "Build tests" ON)
option(SPARK_DSG_INSTALL "Install C++ package" ON)
option(SPARK_DSG_INSTALL_TESTS "Install tests to catkin location" ON)
option(SPARK_DSG_BUILD_PYTHON "Build python bindings" OFF)
option(SPARK_DSG_BUILD_ZMQ "Build zmq message interface" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(zmq IMPORTED_TARGET libzmq)

# TODO(nathan) consider removing pcl
set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

# TODO(nathan) fetch content when possible on 20.04
configure_file(cmake/json.CMakeLists.txt.in json-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download" OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download" OUTPUT_QUIET
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_BINARY_DIR}/json-src ${CMAKE_BINARY_DIR}/json-build)

add_library(
  ${PROJECT_NAME}
  src/adjacency_matrix.cpp
  src/binary_serializer.cpp
  src/bounding_box.cpp
  src/dynamic_scene_graph.cpp
  src/dynamic_scene_graph_layer.cpp
  src/edge_attributes.cpp
  src/edge_container.cpp
  src/graph_binary_serialization.cpp
  src/graph_json_serialization.cpp
  src/node_attributes.cpp
  src/node_symbol.cpp
  src/scene_graph_node.cpp
  src/scene_graph_layer.cpp
  src/scene_graph_types.cpp
  src/scene_graph_utilities.cpp
  src/serialization_helpers.cpp
  src/scene_graph_logger.cpp
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen
  PRIVATE nlohmann_json::nlohmann_json
)

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE PkgConfig::zmq)
  target_sources(
    ${PROJECT_NAME}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/zmq_interface.cpp>"
  )
endif()

if(SPARK_DSG_BUILD_PYTHON)
  add_subdirectory(python)
endif()

if(SPARK_DSG_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(SPARK_DSG_INSTALL)
  add_library(spark_dsg::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
  install(TARGETS ${PROJECT_NAME} EXPORT spark_dsg-targets
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
          ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT spark_dsg-targets FILE spark_dsgTargets.cmake NAMESPACE spark_dsg::
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg
  )

  include(CMakePackageConfigHelpers)
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
endif()
