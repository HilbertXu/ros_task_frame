cmake_minimum_required(VERSION 2.8.12)

project(robot_vision_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    genmsg
    sensor_msgs
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    std_msgs
    message_generation
)

add_message_files(
  FILES
    Joint.msg
    HumanPose.msg
    HumanPoses.msg
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
)

add_action_files(
  FILES
    CheckForHumanPoses.action
    CheckForObjects.action
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    message_runtime
    std_msgs
)
