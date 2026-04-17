from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml
from launch.actions import ExecuteProcess


def _load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _load_moveit_yaml(path: str):
    data = _load_yaml(path)
    if not isinstance(data, dict):
        return {}

    if "move_group" in data and isinstance(data["move_group"], dict):
        mg = data["move_group"]
        if "ros__parameters" in mg and isinstance(mg["ros__parameters"], dict):
            return mg["ros__parameters"]
        return mg

    return data


def generate_launch_description():
    pkg_cfg = get_package_share_directory("asrs_robot_moveit_config")
    pkg_robot = get_package_share_directory("asrs_robot")

    xacro_robot = os.path.join(pkg_robot, "urdf", "asrs_robot.urdf.xacro")
    srdf_path = os.path.join(pkg_cfg, "config", "asrs_robot.srdf")
    rviz_path = os.path.join(pkg_cfg, "launch", "moveit.rviz")

    kinematics_yaml = _load_moveit_yaml(os.path.join(pkg_cfg, "config", "kinematics.yaml"))
    ompl_planning_yaml = _load_moveit_yaml(os.path.join(pkg_cfg, "config", "ompl_planning.yaml"))
    moveit_ctrls_yaml = _load_moveit_yaml(os.path.join(pkg_cfg, "config", "moveit_controllers.yaml"))
    traj_exec_yaml = _load_moveit_yaml(os.path.join(pkg_cfg, "config", "trajectory_execution.yaml"))
    joint_limits_yaml = _load_moveit_yaml(os.path.join(pkg_cfg, "config", "joint_limits.yaml"))

    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_cfg = ompl_planning_yaml.setdefault("ompl", {})
    ompl_cfg.setdefault("planning_plugin", "ompl_interface/OMPLPlanner")
    ompl_cfg.setdefault(
        "request_adapters",
        "default_planner_request_adapters/AddTimeOptimalParameterization "
        "default_planner_request_adapters/FixWorkspaceBounds "
        "default_planner_request_adapters/FixStartStateBounds "
        "default_planner_request_adapters/FixStartStateCollision "
        "default_planner_request_adapters/FixStartStatePathConstraints",
    )
    ompl_cfg.setdefault("start_state_max_bounds_error", 0.1)
    ompl_planning_yaml.setdefault("default_planning_pipeline", "ompl")
    ompl_planning_yaml.setdefault("planning_pipelines", ["ompl"])

    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    urdf_xml = xacro.process_file(xacro_robot, mappings={"use_ros2_control": "false"}).toxml()
    robot_description = {"robot_description": urdf_xml}

    no_sim_time = {"use_sim_time": False}

    # If you ALREADY publish map->base_link from somewhere else, remove this node.
    map_to_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_base_link_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"],
        output="screen",
    )
    
    joint_state_relay = ExecuteProcess(
        cmd=['ros2', 'topic', 'relay', '/teensy/joint_states', '/joint_states'],
        output='screen',
        name='joint_state_relay',
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, no_sim_time],
        output="screen",
    )

    bridge_node = Node(
        package="trajectory_bridge",
        executable="trajectory_bridge",
        name="moveit_teensy_bridge",
        output="screen",
        parameters=[{
            "teensy_joint_states_topic": "/teensy/joint_states",
            "joint_states_topic": "/joint_states",
            "teensy_joint_targets_topic": "/teensy_joint_targets",
            "teensy_gripper_topic": "/teensy_gripper_angle_target",

            "arm_joints": ["continuous", "revolute1", "revolute2"],
            "gripper_joints": ["revolute3"],

            "continuous_limit_rad": 3.0,
            "continuous_feedback_sign": -1.0,

            "goal_tolerance_rad": 0.08,
            "arm_goal_tolerances": [0.08, 0.15, 0.12],

            "settle_sec": 0.3,
            "check_dt": 0.02,

            "reach_timeout_scaling": 10.0,
            "reach_timeout_margin_sec": 20.0,

            "startup_wait_sec": 2.0,
            "reject_without_joint_state": False,
        }],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            ompl_planning_yaml,
            traj_exec_yaml,
            moveit_ctrls_yaml,
            no_sim_time,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            no_sim_time,
        ],
        output="screen",
    )

    gateway_node = Node(
        package="web_moveit_gateway_cpp",
        executable="gateway",
        name="web_moveit_gateway_cpp",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            no_sim_time,
        ],
    )

    pick_place_node = Node(
        package="asrs_robot_apps",
        executable="pick_place_from_states",
        name="pick_place_from_states",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            kinematics_yaml,
            no_sim_time,
            {
                "move_group_name": "arm4",
                "world_frame": "map",
                "eef_link": "tool0",
                "attach_link": "tool0",

                "run_on_startup": False,
                "keep_alive": True,
                "start_delay_sec": 2.0,

                # your SRDF must contain these named targets
                "state_sequence": ["home", "pregrasp", "grasp", "lift", "preplace", "place", "prehome", "home"],

                # jaws control (your firmware topic is /teensy_jaw_target)
                "jaws_topic": "/teensy_jaw_target",
                "jaws_open": 1.0,
                "jaws_close": 0.0,

                # simple scene
                "spawn_scene": False,
                "table_size_xyz": [0.50, 0.80, 0.05],
                "table_pose_xyzw": [0.35, 0.0, 0.10, 0.0, 0.0, 0.0, 1.0],  

                "bottle_radius": 0.03,
                "bottle_height": 0.20,
                "bottle_pose_xyzw": [0.35, 0.10, 0.225, 0.0, 0.0, 0.0, 1.0], 
                "bottle_attach_offset_xyzw": [0.0, 0.0, -0.10, 0.7071, 0.0, 0.0, 0.7071],
                "trail_marker_size": 0.08,
            }
        ],
    )

    return LaunchDescription([
        map_to_base_tf,
        robot_state_publisher_node,
        joint_state_relay,
        bridge_node,
        move_group_node,
        rviz_node,
        gateway_node,
        pick_place_node,
    ])

