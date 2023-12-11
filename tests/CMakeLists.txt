if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG v1.13.0
  )

  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
  endif()

  set(BUILD_SHARED_LIBS_THIS ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "")
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
  set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_THIS} CACHE BOOL "")
endif()

include(GoogleTest)
enable_testing()

add_executable(
  utest_${PROJECT_NAME}
  utest_adjacency_matrix.cpp
  utest_attribute_serialization.cpp
  utest_binary_serialization.cpp
  utest_bounding_box.cpp
  utest_dynamic_scene_graph_layer.cpp
  utest_dynamic_scene_graph.cpp
  utest_edge_container.cpp
  utest_file_io.cpp
  utest_graph_utilities_layer.cpp
  utest_json_serialization.cpp
  utest_node_symbol.cpp
  utest_scene_graph_layer.cpp
  utest_scene_graph_node.cpp
  utest_scene_graph_types.cpp
  utest_scene_graph_utilities.cpp
)
target_include_directories(
  utest_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(
  utest_${PROJECT_NAME} PRIVATE ${PROJECT_NAME} GTest::gtest_main
                                nlohmann_json::nlohmann_json
)
gtest_add_tests(TARGET utest_${PROJECT_NAME})

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  target_sources(
    utest_${PROJECT_NAME}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utest_zmq_interface.cpp>"
  )
endif()
