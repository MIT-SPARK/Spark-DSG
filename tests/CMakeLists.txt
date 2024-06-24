find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

add_executable(
  utest_${PROJECT_NAME}
  utest_adjacency_matrix.cpp
  utest_bounding_box_extraction.cpp
  utest_bounding_box.cpp
  utest_color.cpp
  utest_dynamic_scene_graph_layer.cpp
  utest_dynamic_scene_graph.cpp
  utest_edge_container.cpp
  utest_graph_utilities_layer.cpp
  utest_mesh.cpp
  utest_node_attributes.cpp
  utest_node_symbol.cpp
  utest_scene_graph_layer.cpp
  utest_scene_graph_node.cpp
  utest_scene_graph_types.cpp
  utest_scene_graph_utilities.cpp
  serialization/utest_attribute_serialization.cpp
  serialization/utest_binary_serialization.cpp
  serialization/utest_binary_conversions.cpp
  serialization/utest_graph_serialization.cpp
  serialization/utest_file_io.cpp
  serialization/utest_json_conversions.cpp
  serialization/utest_mesh_serialization.cpp
)
target_include_directories(utest_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(utest_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET utest_${PROJECT_NAME})

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  target_sources(
    utest_${PROJECT_NAME}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utest_zmq_interface.cpp>"
  )
endif()
