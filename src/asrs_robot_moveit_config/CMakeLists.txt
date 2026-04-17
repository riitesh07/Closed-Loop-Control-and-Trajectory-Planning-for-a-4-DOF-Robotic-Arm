
cmake_minimum_required(VERSION 3.22)
project(asrs_robot_moveit_config)

find_package(ament_cmake REQUIRED)

# Install launch files (exclude setup_assistant if needed)
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

# Install config files (including URDFs, YAMLs)
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

# Install the MoveIt setup assistant marker file
install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

