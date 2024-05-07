get_filename_component(spark_dsg_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

find_dependency(Eigen3 REQUIRED)
find_dependency(nlohmann_json REQUIRED)

if(NOT TARGET spark_dsg::spark_dsg)
  include("${spark_dsg_CMAKE_DIR}/spark_dsgTargets.cmake")
endif()

set(spark_dsg_LIBRARIES spark_dsg::spark_dsg)
set(spark_dsg_FOUND_CATKIN_PROJECT TRUE)
