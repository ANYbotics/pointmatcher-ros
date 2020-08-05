cmake_minimum_required(VERSION 3.1.3)
project(pointmatcher_ros)

# Set compiler flags
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  eigen_conversions
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

# Find catkin macros and libraries
find_package(catkin
  REQUIRED COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(libpointmatcher REQUIRED)
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# Catkin package macro
catkin_package(
  INCLUDE_DIRS 
    include
    ${PCL_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_definitions(-DHAVE_YAML_CPP)
include_directories(
  include
  SYSTEM
    ${LIBPOINTMATCHER_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
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
  ${catkin_LIBRARIES}
  ${LIBPOINTMATCHER_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
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

  include_directories(
    include
    SYSTEM
      ${catkin_INCLUDE_DIRS}
      ${LIBPOINTMATCHER_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
  )

  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()


###################
## Code_coverage ##
###################
find_package(cmake_code_coverage QUIET)
if(cmake_code_coverage_FOUND)
  add_gtest_coverage(
    TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
  )
endif(cmake_code_coverage_FOUND)
