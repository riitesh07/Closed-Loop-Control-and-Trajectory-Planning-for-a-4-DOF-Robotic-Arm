cmake_minimum_required(VERSION 3.8)
project(asrs_robot_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(capture_tcp_pose src/capture_tcp_pose.cpp)
ament_target_dependencies(capture_tcp_pose
  rclcpp
  moveit_ros_planning_interface
)

add_executable(pick_place_from_states src/pick_place_from_states.cpp)
ament_target_dependencies(pick_place_from_states
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  shape_msgs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

install(TARGETS
  capture_tcp_pose
  pick_place_from_states
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

