cmake_minimum_required(VERSION 2.8.3)

project(pointmatcher_ros)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  nav_msgs
  pcl_conversions
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(libpointmatcher REQUIRED)
find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS 
    include
    ${PCL_INCLUDE_DIRS}
  LIBRARIES
    pointmatcher_ros
  CATKIN_DEPENDS
    eigen_conversions
    nav_msgs
    pcl_conversions
    roscpp
    sensor_msgs
    tf
    tf_conversions
)

# c++ 0x
#if("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang") 
#	set(CMAKE_COMPILER_IS_CLANGXX)
#endif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "clang")
if(CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
    add_definitions(-std=gnu++0x)
  else (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
    message(SEND_ERROR "You need GCC version 4.4 or greater to compile this library")
  endif (GCC_VERSION VERSION_GREATER 4.4 OR GCC_VERSION VERSION_EQUAL 4.4)
  else(CMAKE_COMPILER_IS_GNUCXX)
  message(SEND_ERROR "You need partial C++0x support for N2351, N2672, N1984. Currently this build toolchain supports only GCC >= 4.4. If your compiler supports these specifications, please send us a patch.")
endif(CMAKE_COMPILER_IS_GNUCXX)

add_definitions(-DHAVE_YAML_CPP)
include_directories(
  include
  ${LIBPOINTMATCHER_INCLUDE_DIRS}
)

add_library(pointmatcher_ros
  src/point_cloud.cpp
  src/transform.cpp
  src/StampedPointCloud.cpp
  src/PmTf.cpp
  src/PointMatcherFilterIface.cpp
  src/helper_functions.cpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pointmatcher_ros
  ${catkin_LIBRARIES}
  ${LIBPOINTMATCHER_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

install(TARGETS pointmatcher_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
