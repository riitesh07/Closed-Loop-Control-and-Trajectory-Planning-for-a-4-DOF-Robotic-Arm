from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    abs_path = os.path.join(package_path, file_path)
    with open(abs_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_name = "asrs_robot_moveit_config"
    robot_pkg = "asrs_robot"

    pkg_path = get_package_share_directory(pkg_name)
    robot_pkg_path = get_package_share_directory(robot_pkg)

    # Files
    xacro_file = os.path.join(pkg_path, "config", "asrs_robot_moveit.urdf.xacro")
    control_xacro = os.path.join(pkg_path, "config", "asrs_robot.ros2_control.xacro")
    robot_xacro = os.path.join(robot_pkg_path, "urdf", "asrs_robot.urdf.xacro")
    initial_positions = os.path.join(pkg_path, "config", "initial_positions.yaml")

    # Load joint init values
    with open(initial_positions, "r") as f:
        initial_vals = yaml.safe_load(f)["initial_positions"]

    doc = xacro.process_file(
        xacro_file,
        mappings={
            "continuous_init": str(initial_vals["continuous"]),
            "revolute1_init": str(initial_vals["revolute1"]),
            "revolute2_init": str(initial_vals["revolute2"]),
            "revolute3_init": str(initial_vals["revolute3"]),
            "robot_xacro_file": robot_xacro,
            "control_xacro_file": control_xacro,
        }
    )
    robot_description = {"robot_description": doc.toxml()}

    # Load fixed controller config (FLATTENED)
    controller_yaml = load_yaml(pkg_name, "config/ros2_controllers.yaml")

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description]
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                controller_yaml
            ],
            output="both"
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        )
    ])

