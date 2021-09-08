cmake_minimum_required(VERSION 3.1)
project(kimera_dsg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# TODO(nathan) remove when json actually fixes fallthrough warning
add_compile_options(-Wall -Wextra -Wno-implicit-fallthrough)

# TODO(nathan) only required for pcl
find_package(Boost REQUIRED)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(
    Boost::boost PROPERTIES INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
                            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
  )
endif()

find_package(catkin REQUIRED COMPONENTS pcl_ros)
find_package(Eigen3 REQUIRED)

# TODO(nathan) clean this up / handle glog_catkin
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED IMPORTED_TARGET libglog)
# TODO(nathan) export this into catkin

# TODO(nathan) move this to separate cmake file
# TODO(nathan) custom targets instead of execute_process
configure_file(cmake/json.CMakeLists.txt.in json-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download"
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download"
)

# TODO(nathan) export of json is pretty ugly. We hijack the cmake install target to
# place headers in the include path of the dsg package
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" ${CMAKE_BINARY_DIR}/json-src
          -DJSON_BuildTests=OFF -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-build"
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build . --target install
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-build"
)

find_package(nlohmann_json)

catkin_package(
  CATKIN_DEPENDS pcl_ros
  DEPENDS Boost
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/attribute_serialization.cpp
  src/bounding_box.cpp
  src/dynamic_scene_graph.cpp
  src/dynamic_scene_graph_layer.cpp
  src/node_attributes.cpp
  src/node_symbol.cpp
  src/scene_graph_node.cpp
  src/scene_graph_layer.cpp
  src/scene_graph.cpp
  src/serialization_helpers.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  ${PROJECT_NAME} ${catkin_LIBRARIES} Boost::boost PkgConfig::glog
  nlohmann_json::nlohmann_json
)

if(CATKIN_ENABLE_TESTING)
  # TODO(nathan) clean this up maybe
  pkg_check_modules(gflags REQUIRED IMPORTED_TARGET gflags)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp
    tests/utest_bounding_box.cpp
    tests/utest_dynamic_scene_graph.cpp
    tests/utest_dynamic_scene_graph_layer.cpp
    tests/utest_graph_utilities_layer.cpp
    tests/utest_node_symbol.cpp
    tests/utest_scene_graph_node.cpp
    tests/utest_scene_graph_layer.cpp
    tests/utest_scene_graph.cpp
    tests/utest_serialization.cpp
  )
  target_link_libraries(
    utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} PkgConfig::gflags
  )
endif()

# TODO(nathan) handle install
