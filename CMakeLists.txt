cmake_minimum_required(VERSION 3.5.1)
project(pointmatcher_ros)

# Set compiler flags
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)

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
)

find_package(Boost REQUIRED)

# Find catkin macros and libraries
find_package(catkin
  REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PCL 1.7 REQUIRED
  COMPONENTS
    common
    io
)

# Catkin package macro
catkin_package(
  INCLUDE_DIRS 
    include
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

add_definitions(-DHAVE_YAML_CPP)
include_directories(
  include
  SYSTEM
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_IO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/point_cloud.cpp
  src/transform.cpp
  src/StampedPointCloud.cpp
  src/PmTf.cpp
  src/PointMatcherFilterInterface.cpp
  src/helper_functions.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/empty_test.cpp
  )
  target_include_directories(test_${PROJECT_NAME}
    PRIVATE
      include
    PUBLIC SYSTEM
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