cmake_minimum_required(VERSION 2.8.3)
project(romea_common_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++ -Wall -Wextra -std=c++17 -O3")

#######################
## Find dependencies ##
#######################
find_package(catkin REQUIRED COMPONENTS roscpp romea_common tf2_ros tf2_eigen nav_msgs diagnostic_updater)
find_package(Boost REQUIRED COMPONENTS system)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )


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

include_directories(
    include
    SYSTEM ${catkin_INCLUDE_DIRS}
    SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_library(romea_common_utils
    src/conversions/TimeConversions.cpp
    src/conversions/GeometryConversions.cpp
    src/conversions/TransformConversions.cpp
    src/diagnostics/DiagnosticReliability.cpp
    src/diagnostics/DiagnosticNLSE.cpp
    )

target_link_libraries(romea_common_utils ${catkin_LIBRARIES})


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

get_filename_component(TEST_WITH_DATA_TEST_DIR "test/data" ABSOLUTE)
configure_file(test/test_helper.h.in test/test_helper.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)
if(CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)

    catkin_add_gtest(${PROJECT_NAME}_test_transform test/test_transform.cpp )
    target_link_libraries(${PROJECT_NAME}_test_transform ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_time test/test_time.cpp )
    target_link_libraries(${PROJECT_NAME}_test_time ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_pointset test/test_pointset.cpp )
    target_link_libraries(${PROJECT_NAME}_test_pointset ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_math_algorithm test/test_math_algorithm.cpp )
    target_link_libraries(${PROJECT_NAME}_test_math_algorithm ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_math_euler test/test_math_euler.cpp )
    target_link_libraries(${PROJECT_NAME}_test_math_euler ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_math_interval test/test_math_interval.cpp )
    target_link_libraries(${PROJECT_NAME}_test_math_interval ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_math_matrix test/test_math_matrix.cpp )
    target_link_libraries(${PROJECT_NAME}_test_math_matrix ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_coordinates test/test_coordinates.cpp )
    target_link_libraries(${PROJECT_NAME}_test_coordinates ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_geodesy test/test_geodesy.cpp )
    target_link_libraries(${PROJECT_NAME}_test_geodesy ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_containers_bb test/test_containers_bb.cpp )
    target_link_libraries(${PROJECT_NAME}_test_containers_bb ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_containers_grid test/test_containers_grid.cpp )
    target_link_libraries(${PROJECT_NAME}_test_containers_grid ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_containers_ray test/test_containers_ray.cpp )
    target_link_libraries(${PROJECT_NAME}_test_containers_ray ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_monitoring test/test_monitoring.cpp )
    target_link_libraries(${PROJECT_NAME}_test_monitoring ${PROJECT_NAME})

    catkin_add_gtest(${PROJECT_NAME}_test_ros_conversions test/test_ros_conversions.cpp )
    target_link_libraries(${PROJECT_NAME}_test_ros_conversions ${PROJECT_NAME})

    add_rostest_gtest(${PROJECT_NAME}_test_ros_params test/test_ros_param.test test/test_ros_param_test.cpp)
    target_link_libraries(${PROJECT_NAME}_test_ros_params ${PROJECT_NAME})

    add_rostest_gtest(${PROJECT_NAME}_test_ros_transform test/test_ros_transform.test test/test_ros_transform_test.cpp)
    target_link_libraries(${PROJECT_NAME}_test_ros_transform ${PROJECT_NAME})

    add_rostest_gtest(${PROJECT_NAME}_test_ros_publisher test/test_ros_publisher.test test/test_ros_publisher_test.cpp)
    target_link_libraries(${PROJECT_NAME}_test_ros_publisher ${PROJECT_NAME})


endif(CATKIN_ENABLE_TESTING)
