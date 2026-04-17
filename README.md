This repository demonstrates the control of a custom-built 4-DOF robotic arm through the micro-ROS framework, executed on a Teensy 4.1 microcontroller and integrated with ROS 2. It employs RViz for robot visualization and MoveIt for trajectory and motion planning, with the system applied to automate pick-and-place tasks in an Industry 4.0 benchmark bottle-filling application.


QUICK START GUIDE:
1. Install micro-ROS dependencies
2. Clone this repository into your ROS 2 workspace
3. Build the workspace
4. Source the workspace
5. Start the micro-ROS Agent (Teensy 4.1) ros2 run micro_ros_agent micro_ros_agent serial \
     --dev /dev/serial/by-id/usb-Teensyduino_USB_Serial_17541650-if00 -v4
