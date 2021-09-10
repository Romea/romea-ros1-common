cmake_minimum_required(VERSION 3.0.2)
project(romea_common_msgs)

set(${PROJECT_NAME}_CATKIN_DEPS std_msgs geometry_msgs message_generation)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_message_files(
  DIRECTORY msg
  FILES
  Position2D.msg
  Pose2D.msg
  Twist2D.msg
  PoseAndTwist2D.msg
  Position2DStamped.msg
  Pose2DStamped.msg
  Twist2DStamped.msg
  PoseAndTwist2DStamped.msg
  Vector2.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
  )
