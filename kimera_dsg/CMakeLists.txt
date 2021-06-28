cmake_minimum_required(VERSION 3.1)
project(kimera_dsg)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Boost REQUIRED serialization)
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

catkin_package(
  CATKIN_DEPENDS pcl_ros
  DEPENDS Boost
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/bounding_box.cpp src/dynamic_scene_graph.cpp src/node_attributes.cpp
  src/scene_graph_node.cpp src/scene_graph_layer.cpp src/scene_graph.cpp
)
target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Boost::boost PkgConfig::glog)

if(CATKIN_ENABLE_TESTING)
  # TODO(nathan) clean this up maybe
  pkg_check_modules(gflags REQUIRED IMPORTED_TARGET gflags)
  catkin_add_gtest(
    utest_${PROJECT_NAME}
    tests/utest_main.cpp tests/utest_bounding_box.cpp
    tests/utest_dynamic_scene_graph.cpp tests/utest_scene_graph_node.cpp
    tests/utest_scene_graph_layer.cpp tests/utest_scene_graph.cpp
  )
  target_link_libraries(
    utest_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES} PkgConfig::gflags
  )
endif()

# TODO(nathan) handle install
