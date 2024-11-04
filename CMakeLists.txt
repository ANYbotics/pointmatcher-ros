cmake_minimum_required(VERSION 3.16.3)
project(pointmatcher_ros)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if("$ENV{ROS_VERSION}" STREQUAL "1")

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  eigen_conversions
  libpointmatcher
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf_conversions
  visualization_msgs
)

find_package(Boost REQUIRED)

# Find catkin macros and libraries
find_package(catkin
  REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PCL REQUIRED
  COMPONENTS
    common
    io
)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_definitions(-DHAVE_YAML_CPP)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/helper_functions.cpp
  src/serialization.cpp
  src/transform.cpp
  src/IcpMatchesPublisher.cpp
  src/PmTf.cpp
  src/PointMatcherFilterInterface.cpp
  src/StampedPointCloudPublisher.cpp
  src/RosPointCloud2Deserializer.cpp
  src/StampedPointCloud.cpp
  src/visualization_utils.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/RosPointCloud2DeserializerTest.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME}
    )
  endif()
endif()

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGETS
      ${PROJECT_NAME}
    SOURCE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_FILTER 
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_CONFIG_FILE
      ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

else() # ROS version 2

set(PACKAGE_DEPENDENCIES
  visualization_msgs
  pcl_conversions
  libpointmatcher
  tf2_eigen
  sensor_msgs
  nav_msgs
  rclcpp
  Boost
  PCL
  tf2
  )

find_package(ament_cmake REQUIRED)

foreach(PKG ${PACKAGE_DEPENDENCIES})
  find_package(${PKG} REQUIRED)
endforeach()

########################
## Library definition ##
########################

add_library(${PROJECT_NAME} SHARED
  src/helper_functions.cpp
  src/serialization.cpp
  src/transform.cpp
  src/IcpMatchesPublisher.cpp
  src/PmTf.cpp
  src/PointMatcherFilterInterface.cpp
  src/StampedPointCloudPublisher.cpp
  src/RosPointCloud2Deserializer.cpp
  src/StampedPointCloud.cpp
  src/visualization_utils.cpp
  )

target_link_libraries(${PROJECT_NAME}
  pcl_common
  pcl_io
  )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  )

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

#############
## Testing ##
#############

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_${PROJECT_NAME}
    test/RosPointCloud2DeserializerTest.cpp
    )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    )

  ###################
  ## Code_coverage ##
  ###################

  find_package(cmake_code_coverage QUIET)

  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS test_${PROJECT_NAME})
  endif()

endif()

ament_package()

endif()
