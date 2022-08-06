# from teaser-plusplus
configure_file(GoogleTest.CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/googletest-download" OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/googletest-download" OUTPUT_QUIET
)

# We use the global cmake option to set the library type for spark_dsg. Unfortunately,
# this also means that gtest follows the same setting, and when building the shared
# library (default), gtest picks this up and the test executable tries to load the gtest
# library which doesn't get installed. Instead, we override the setting for gtest so we
# always have static gtest linkage.
set(BUILD_SHARED_LIBS OFF)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(
  "${CMAKE_CURRENT_BINARY_DIR}/googletest-src"
  "${CMAKE_CURRENT_BINARY_DIR}/googletest-build" EXCLUDE_FROM_ALL
)

add_executable(
  utest_${PROJECT_NAME}
  utest_main.cpp
  utest_adjacency_matrix.cpp
  utest_attribute_serialization.cpp
  utest_bounding_box.cpp
  utest_dynamic_scene_graph.cpp
  utest_dynamic_scene_graph_layer.cpp
  utest_edge_container.cpp
  utest_graph_utilities_layer.cpp
  utest_binary_serialization.cpp
  utest_json_serialization.cpp
  utest_node_symbol.cpp
  utest_scene_graph_node.cpp
  utest_scene_graph_layer.cpp
  utest_scene_graph_types.cpp
  utest_scene_graph_utilities.cpp
)
target_include_directories(utest_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
  utest_${PROJECT_NAME} ${PROJECT_NAME} gtest nlohmann_json::nlohmann_json
)

if(SPARK_DSG_BUILD_ZMQ AND zmq_FOUND)
  target_sources(
    utest_${PROJECT_NAME}
    PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utest_zmq_interface.cpp>"
  )
endif()

if(${SPARK_DSG_INSTALL_TESTS})
  # note that ros uses libdir to handle finding executables by package, but this isn't
  # an ideal install location normally
  install(TARGETS utest_${PROJECT_NAME}
          RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
  )
endif()
