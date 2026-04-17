cmake_minimum_required(VERSION 3.8)
project(web_moveit_gateway_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(gateway src/gateway.cpp)
target_compile_features(gateway PUBLIC cxx_std_17)

ament_target_dependencies(gateway
  rclcpp std_msgs geometry_msgs moveit_ros_planning_interface
)

install(TARGETS gateway
  DESTINATION lib/${PROJECT_NAME})

ament_package()

