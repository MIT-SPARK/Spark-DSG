cmake_minimum_required(VERSION 3.22)
project(spark_dsg VERSION 1.1.3)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # from: https://www.kitware.com//cmake-and-the-default-build-type/
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE STRING "Turn on -flto" FORCE)
endif()

option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(SPARK_DSG_BUILD_EXAMPLES "Build examples" ON)
option(SPARK_DSG_BUILD_PYTHON "Build python bindings" OFF)
option(SPARK_DSG_BUILD_ZMQ "Build zmq message interface" ON)

set(FIND_MODE REQUIRED)
if(DEFINED SKBUILD AND "${SKBUILD}" GREATER 0)
  set(SPARK_DSG_BUILD_ZMQ OFF)
  set(FIND_MODE QUIET)
endif()

include(FetchContent)

find_package(nlohmann_json ${FIND_MODE})
if(NOT ${nlohmann_json_FOUND})
  FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
  )

  FetchContent_GetProperties(json)
  if(NOT json_POPULATED)
    FetchContent_Populate(json)
  endif()

  add_library(nlohmann_json::nlohmann_json INTERFACE IMPORTED)
  target_include_directories(nlohmann_json::nlohmann_json INTERFACE ${json_SOURCE_DIR}/single_include)
endif()

find_package(Eigen3 ${FIND_MODE})
if(NOT ${Eigen3_FOUND})
  FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
    URL_HASH MD5=a83cb9a2cbba2dd52c137ac62d33d847
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )

  FetchContent_GetProperties(eigen)
  if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
  endif()

  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE "${eigen_SOURCE_DIR}")
endif()

set(SPARK_DSG_USE_ZMQ 0)
if(SPARK_DSG_BUILD_ZMQ)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(zmq libzmq)
  if(zmq_FOUND)
    set(SPARK_DSG_USE_ZMQ 1)
  else()
    message(WARNING "ZMQ not found! Disabling ZMQ interface")
  endif()
endif()

configure_file(cmake/spark_dsg_version.h.in include/spark_dsg_version.h)

add_library(
  ${PROJECT_NAME}
  src/adjacency_matrix.cpp
  src/bounding_box_extraction.cpp
  src/bounding_box.cpp
  src/color.cpp
  src/colormaps.cpp
  src/dynamic_scene_graph.cpp
  src/edge_attributes.cpp
  src/edge_container.cpp
  src/labelspace.cpp
  src/layer_view.cpp
  src/mesh.cpp
  src/metadata.cpp
  src/node_attributes.cpp
  src/node_symbol.cpp
  src/printing.cpp
  src/scene_graph_layer.cpp
  src/scene_graph_logger.cpp
  src/scene_graph_node.cpp
  src/scene_graph_types.cpp
  src/scene_graph_utilities.cpp
  src/traversability_boundary.cpp
  src/zmq_interface.cpp
  src/serialization/attribute_serialization.cpp
  src/serialization/binary_conversions.cpp
  src/serialization/binary_serialization.cpp
  src/serialization/file_io.cpp
  src/serialization/graph_binary_serialization.cpp
  src/serialization/graph_json_serialization.cpp
  src/serialization/json_conversions.cpp
  src/serialization/mesh_serialization.cpp
  src/serialization/versioning.cpp
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen nlohmann_json::nlohmann_json)

if(NOT BUILD_SHARED_LIBS)
  set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
endif()

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  find_package(Threads REQUIRED)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${zmq_LIBRARIES} Threads::Threads)
  target_include_directories(${PROJECT_NAME} PRIVATE ${zmq_INCLUDE_DIRS})
endif()

add_library(spark_dsg::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

if(SPARK_DSG_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(SPARK_DSG_BUILD_PYTHON)
  add_subdirectory(python)
endif()

include(CTest)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

if(NOT DEFINED SKBUILD OR NOT "${SKBUILD}")
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion
  )

  configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/spark_dsgConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfig.cmake
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg"
  )

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT spark_dsg-targets
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  )

  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

  install(
    EXPORT spark_dsg-targets
    FILE spark_dsgTargets.cmake
    NAMESPACE spark_dsg::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg"
  )

  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/spark_dsgConfigVersion.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/spark_dsg"
  )
endif()
