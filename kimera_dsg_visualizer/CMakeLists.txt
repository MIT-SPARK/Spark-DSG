cmake_minimum_required(VERSION 3.1)
project(kimera_dsg_visualizer)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for HSL conversion
find_package(OpenCV REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure kimera_dsg tf2_eigen
                             visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ColormapConfig.cfg cfg/DynamicLayerVisualizerConfig.cfg
  cfg/LayerVisualizerConfig.cfg cfg/VisualizerConfig.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp dynamic_reconfigure kimera_dsg tf2_eigen visualization_msgs
  DEPENDS OpenCV
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME}
  src/dsg_mesh_plugin.cpp src/dynamic_scene_graph_visualizer.cpp
  src/scene_graph_visualizer.cpp src/visualizer_types.cpp src/visualizer_utils.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/kimera_dsg_visualizer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

add_executable(rotate_tf_node src/rotate_tf_node.cpp)
target_link_libraries(rotate_tf_node ${catkin_LIBRARIES})
target_include_directories(rotate_tf_node PUBLIC ${catkin_INCLUDE_DIRS})

# TODO(nathan) handle install
