cmake_minimum_required(VERSION 3.0.2)
project(romea_common_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++ -Wall -Wextra -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#######################
## Find dependencies ##
#######################
find_package(catkin REQUIRED COMPONENTS
 roscpp
 tf2_ros
 tf2_eigen
 nav_msgs
 romea_common_msgs
 diagnostic_updater
 rviz_visual_tools)

find_package(romea_core_common REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## Build ##
###########

#include_directories(
#    include
#    SYSTEM ${catkin_INCLUDE_DIRS}
#    SYSTEM ${EIGEN3_INCLUDE_DIRS}
#    SYSTEM ${romea_common_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/conversions/diagnostic_conversions.cpp
  src/conversions/time_conversions.cpp
  src/conversions/geometry_conversions.cpp
  src/conversions/transform_conversions.cpp
  src/conversions/pose2d_conversions.cpp
  src/conversions/pose3d_conversions.cpp
  src/conversions/pose_and_twist2d_conversions.cpp
  src/conversions/pose_and_twist3d_conversions.cpp
  src/conversions/position2d_conversions.cpp
  src/conversions/twist2d_conversions.cpp
  src/conversions/twist3d_conversions.cpp
  src/pid_factory.cpp
  src/rviz_display.cpp
  )

target_include_directories( ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )

target_include_directories( ${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${romea_core_common_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  romea_core_common::romea_core_common)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS romea
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    FILES_MATCHING PATTERN "*.hpp"
#    PATTERN ".svn" EXCLUDE
#    )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)
