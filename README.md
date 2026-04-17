This repository demonstrates the control of a custom-built 4-DOF robotic arm through the micro-ROS framework, executed on a Teensy 4.1 microcontroller and integrated with ROS 2. It employs RViz for robot visualization and MoveIt for trajectory and motion planning, with the system applied to automate pick-and-place tasks in an Industry 4.0 benchmark bottle-filling application.

## Additional Documentation

Detailed information about the robot hardware (mechanical structure, actuators, sensors) and the software architecture (ROS 2 nodes, micro-ROS on the Teensy 4.1, and motion-planning stack) is provided in the PDF included in this repository.

The complete electrical design of the system, including wiring diagrams, control cabinet layout, and panel documentation, is provided separately in the **EPLAN Electrical Documentation** PDF. These schematics were created using EPLAN Electric P8, a dedicated CAE/CAD environment for electrical engineering and panel design.


## QUICK START GUIDE:
1. Install micro-ROS dependencies
2. Clone this repository into ROS 2 workspace
3. Build the workspace
4. Source the workspace
5. Start the micro-ROS Agent (Teensy 4.1)  ``` ros2 run micro_ros_agent micro_ros_agent serial \
     --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_17541650-if00 -v4```
6. Launch RViz and MoveIt ```ros2 launch asrs_robot_moveit_config demo_microros.launch.py ```
7. Verify Teensy Communication and Optional Resets
8. Run the Pick-and-Place Application ```ros2 run asrs_robot_apps pick_place_from_states```
9. Trigger the Pick-and-Place Sequence ```ros2 service call /pick_place_from_states/run_sequence std_srvs/srv/Trigger "{}"```

 Upon a successful service call, the robot will execute the corresponding pick-and-place trajectory using MoveIt for motion planning and the Teensy 4.1 (via micro-ROS) for hardware control.
