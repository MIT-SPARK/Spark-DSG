cmake_minimum_required(VERSION 3.1)
project(kimera_dsg_visualizer)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for HSL conversion
find_package(OpenCV REQUIRED)

set(CATKIN_COMPONENTS roscpp dynamic_reconfigure kimera_dsg tf2_eigen
                      visualization_msgs
)

find_package(kimera_pgmo QUIET)
if(NOT ${kimera_pgmo_FOUND})
  message(
    WARNING
      "kimera_pgmo not found in the catkin workspace. Features related to publishing meshes will be disabled"
  )
  find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})
else()
  find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS} kimera_pgmo)
  # TODO(nathan) it's weird that gtsam is still required (should be inherited from pgmo
  # and rpgo?)
  find_package(GTSAM REQUIRED)
endif()

set(PGMO_FOUND ${kimera_pgmo_FOUND})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/kimera_dsg_visualizer/feature_config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_dsg_visualizer/feature_config.h
)

generate_dynamic_reconfigure_options(
  cfg/VisualizerConfig.cfg cfg/LayerVisualizerConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure kimera_dsg tf2_eigen visualization_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/dynamic_scene_graph_visualizer.cpp src/scene_graph_visualizer.cpp
  src/visualizer_types.cpp src/visualizer_utils.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
if(${kimera_pgmo_FOUND})
  target_link_libraries(${PROJECT_NAME} gtsam)
endif()

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/kimera_dsg_visualizer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# TODO(nathan) handle install
