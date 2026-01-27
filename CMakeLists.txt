cmake_minimum_required(VERSION 3.16.3)
project(pointmatcher_ros)

# Set compiler flags
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set catkin package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  libpointmatcher
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

# Catkin package macro
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

########################
## Library definition ##
########################
add_library(${PROJECT_NAME}
  src/serialization.cpp
  src/RosPointCloud2Deserializer.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
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

endif()

