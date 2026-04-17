#!/usr/bin/env python3
"""
MoveIt <-> Teensy bridge (ROS 2 Humble)

- Subscribes:  /teensy/joint_states   (sensor_msgs/JointState)
- Republishes: /joint_states         (sensor_msgs/JointState) stamped with host ROS clock

- Action servers:
    /arm_controller/follow_joint_trajectory
    /gripper_controller/follow_joint_trajectory

- Publishes to Teensy:
    /teensy_joint_targets        (std_msgs/Float64MultiArray)
    /teensy_gripper_angle_target (std_msgs/Float64)
    /teensy_emergency_stop       (std_msgs/Bool)
    /teensy_executed             (std_msgs/Bool) - True when all joints stopped

IMPORTANT:
- Base joint "continuous" is treated as BOUNDED REVOLUTE => NO wrap_to_pi.
- Keep your convention: "-P is correct for continuous" => we negate continuous FEEDBACK once here.

NEW (this version):
- arm_command_mode:
    * "final_only" (recommended): publish ONLY final goal once, then wait until reached+stopped
    * "stream_decimated": stream trajectory but publish at low rate (stream_rate_hz) and only when changed

- Emergency stop support:
    * Subscribes to /moveit_stop (or configurable topic)
    * Forwards to /teensy_emergency_stop for immediate firmware halt
    * LATCHES emergency stop - blocks all future actions until /teensy_factory_reset
    * Action CANCEL also triggers emergency stop

- Executed status:
    * Publishes /teensy_executed (True when all joints stopped, False when moving)
    * 10 Hz update rate for MoveIt feedback
"""

import math
import time
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Bool


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class MoveItTeensyBridge(Node):
    def __init__(self):
        super().__init__("moveit_teensy_bridge")

        # ---- params ----
        self.declare_parameter("goal_tolerance_rad", 0.08)
        self.declare_parameter("arm_goal_tolerances", [0.08, 0.07, 0.15])
        self.declare_parameter("settle_sec", 0.3)
        self.declare_parameter("check_dt", 0.02)
        self.declare_parameter("reach_timeout_scaling", 10.0)
        self.declare_parameter("reach_timeout_margin_sec", 20.0)

        self.declare_parameter("startup_wait_sec", 2.0)
        self.declare_parameter("reject_without_joint_state", False)

        # topics
        self.declare_parameter("teensy_joint_states_topic", "/teensy/joint_states")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("teensy_joint_targets_topic", "/teensy_joint_targets")
        self.declare_parameter("teensy_gripper_topic", "/teensy_gripper_angle_target")
        self.declare_parameter("moveit_stop_topic", "/moveit_stop")

        self.declare_parameter("arm_joints", ["continuous", "revolute1", "revolute2"])
        self.declare_parameter("gripper_joints", ["revolute3"])

        # bounded base limit
        self.declare_parameter("continuous_limit_rad", 3.0)

        # Extra multiplier for continuous feedback (set +1.0 normally; use -1.0 only if needed)
        self.declare_parameter("continuous_feedback_sign", 1.0)

        # ---- stop/settle gating ----
        self.declare_parameter("require_stopped", True)
        self.declare_parameter("stop_window_sec", 0.30)
        self.declare_parameter("stop_confirm_sec", 0.25)
        self.declare_parameter("settle_vel_eps_rad_s", 0.05)
        self.declare_parameter("stop_noise_eps_rad", 0.004)
        self.declare_parameter("verify_debug_every_sec", 0.0)

        # ---- NEW: command publishing mode / rate ----
        # final_only: publish only final goal once
        # stream_decimated: publish at low rate + only on meaningful change
        self.declare_parameter("arm_command_mode", "final_only")  # "final_only" or "stream_decimated"
        self.declare_parameter("stream_rate_hz", 8.0)             # only used in stream_decimated
        self.declare_parameter("stream_change_eps_rad", 0.01)     # publish if any joint changes >= this
        self.declare_parameter("min_final_timeout_sec", 45.0)     # ensures enough time if final_only

        # ---- read params ----
        self.goal_tol = float(self.get_parameter("goal_tolerance_rad").value)
        self.arm_goal_tols = [float(x) for x in self.get_parameter("arm_goal_tolerances").value]
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.check_dt = float(self.get_parameter("check_dt").value)
        self.reach_timeout_scaling = float(self.get_parameter("reach_timeout_scaling").value)
        self.reach_timeout_margin = float(self.get_parameter("reach_timeout_margin_sec").value)
        self.startup_wait_sec = float(self.get_parameter("startup_wait_sec").value)
        self.reject_without_js = bool(self.get_parameter("reject_without_joint_state").value)
        self.cont_limit = float(self.get_parameter("continuous_limit_rad").value)

        self.cont_fb_sign = float(self.get_parameter("continuous_feedback_sign").value)
        if abs(self.cont_fb_sign) < 0.5:
            self.get_logger().warn("continuous_feedback_sign too small; forcing to +1.0")
            self.cont_fb_sign = 1.0

        self.require_stopped = bool(self.get_parameter("require_stopped").value)
        self.stop_window_sec = float(self.get_parameter("stop_window_sec").value)
        self.stop_confirm_sec = float(self.get_parameter("stop_confirm_sec").value)
        self.vel_eps = float(self.get_parameter("settle_vel_eps_rad_s").value)
        self.stop_noise_eps = float(self.get_parameter("stop_noise_eps_rad").value)
        self.verify_debug_every = float(self.get_parameter("verify_debug_every_sec").value)

        self.arm_command_mode = str(self.get_parameter("arm_command_mode").value).strip()
        self.stream_rate_hz = float(self.get_parameter("stream_rate_hz").value)
        self.stream_change_eps = float(self.get_parameter("stream_change_eps_rad").value)
        self.min_final_timeout = float(self.get_parameter("min_final_timeout_sec").value)

        if self.stream_rate_hz < 1.0:
            self.get_logger().warn("stream_rate_hz too low; forcing to 1.0")
            self.stream_rate_hz = 1.0
        if self.stream_change_eps < 0.0:
            self.stream_change_eps = 0.0

        self.moveit_stop_topic = str(self.get_parameter("moveit_stop_topic").value)
        self.teensy_js_topic = str(self.get_parameter("teensy_joint_states_topic").value)
        self.js_topic = str(self.get_parameter("joint_states_topic").value)
        self.cmd_topic = str(self.get_parameter("teensy_joint_targets_topic").value)
        self.grip_topic = str(self.get_parameter("teensy_gripper_topic").value)

        self.arm_joints = [str(x) for x in self.get_parameter("arm_joints").value]
        self.gripper_joints = [str(x) for x in self.get_parameter("gripper_joints").value]

        # ---- state cache ----
        self._last_js: Optional[JointState] = None
        self._pos: Dict[str, float] = {}

        # ---- Emergency stop state ----
        self._emergency_stopped = False
        self._estop_lock = threading.Lock()

        cbg = ReentrantCallbackGroup()

        # ---- pub/sub ----
        self._js_repub = self.create_publisher(JointState, self.js_topic, qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self._grip_pub = self.create_publisher(Float64, self.grip_topic, 10)
        self._estop_pub = self.create_publisher(Bool, '/teensy_emergency_stop', 10)
        self._executed_pub = self.create_publisher(Bool, '/teensy_executed', 10)

        self.create_subscription(
            JointState,
            self.teensy_js_topic,
            self._on_teensy_js,
            qos_profile_sensor_data,
            callback_group=cbg,
        )

        self.create_subscription(
            Bool,
            self.moveit_stop_topic,
            self._on_emergency_stop,
            10,
            callback_group=cbg,
        )

        self.create_subscription(
            Bool,
            '/teensy_factory_reset',
            self._on_factory_reset,
            10,
            callback_group=cbg,
        )

        # ---- Executed status publisher timer (10 Hz) ----
        self._executed_timer = self.create_timer(0.1, self._publish_executed_status, callback_group=cbg)

        # ---- action servers ----
        self._arm_as = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._exec_arm,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cbg,
        )

        self._grip_as = ActionServer(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            execute_callback=self._exec_gripper,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cbg,
        )

        self.get_logger().info(
            "Bridge up.\n"
            f"  JS in : {self.teensy_js_topic}\n"
            f"  JS out: {self.js_topic}\n"
            f"  CMD   : {self.cmd_topic}\n"
            f"  GRIP  : {self.grip_topic}\n"
            f"  ESTOP : {self.moveit_stop_topic} → /teensy_emergency_stop\n"
            f"  EXEC  : /teensy_executed (10 Hz)\n"
            f"  RESET : /teensy_factory_reset → clears emergency stop\n"
            f"  Arm joints: {self.arm_joints}\n"
            f"  Gripper joints: {self.gripper_joints}\n"
            f"  continuous_limit_rad: {self.cont_limit}\n"
            f"  continuous_feedback_sign: {self.cont_fb_sign}\n"
            f"  (continuous FEEDBACK negated here: '-P is correct')\n"
            f"  require_stopped: {self.require_stopped}\n"
            f"  stop_window_sec: {self.stop_window_sec}, stop_confirm_sec: {self.stop_confirm_sec}\n"
            f"  vel_eps: {self.vel_eps} rad/s, stop_noise_eps: {self.stop_noise_eps} rad\n"
            f"  arm_command_mode: {self.arm_command_mode}\n"
            f"  stream_rate_hz: {self.stream_rate_hz}, stream_change_eps_rad: {self.stream_change_eps}\n"
            f"  min_final_timeout_sec: {self.min_final_timeout}"
        )

    # ---------- callbacks ----------
    def _on_teensy_js(self, msg: JointState):
        pos_out = list(msg.position) if msg.position else []
        vel_out = list(msg.velocity) if msg.velocity else []
        eff_out = list(msg.effort) if msg.effort else []

        for i, name in enumerate(msg.name):
            if i >= len(pos_out):
                continue

            p = float(pos_out[i])

            # Keep your convention: "-P is correct for continuous" (FEEDBACK mapping)
            if name == "continuous":
                p = -p * self.cont_fb_sign
                pos_out[i] = p
                if i < len(vel_out):
                    vel_out[i] = -float(vel_out[i]) * self.cont_fb_sign

            # cache must match what we publish
            self._pos[name] = float(pos_out[i])

        self._last_js = msg

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.name = list(msg.name)
        out.position = pos_out
        out.velocity = vel_out
        out.effort = eff_out
        self._js_repub.publish(out)

    def _on_emergency_stop(self, msg: Bool):
        """Forward MoveIt STOP to firmware + latch locally"""
        if msg.data:
            with self._estop_lock:
                self._emergency_stopped = True
            self.get_logger().warn('EMERGENCY STOP ACTIVE → All actions blocked until factory_reset')
            self._estop_pub.publish(Bool(data=True))

    def _on_factory_reset(self, msg: Bool):
        """Clear emergency stop on factory reset"""
        if msg.data:
            with self._estop_lock:
                self._emergency_stopped = False
            self.get_logger().info('Factory reset → Emergency stop cleared, actions allowed')

    def _publish_executed_status(self):
        """Publish 'executed' when ALL joints stopped (10 Hz)"""
        if self._last_js is None:
            return

        # Check all arm joints stopped
        all_stopped = True

        for j in self.arm_joints:
            # Get current velocity
            try:
                idx = self._last_js.name.index(j)
                if idx < len(self._last_js.velocity):
                    vel = abs(self._last_js.velocity[idx])
                    if vel > self.vel_eps:
                        all_stopped = False
                        break
            except (ValueError, IndexError):
                pass

        # Executed = stopped AND NOT emergency stopped
        with self._estop_lock:
            executed = all_stopped and not self._emergency_stopped

        msg = Bool()
        msg.data = executed
        self._executed_pub.publish(msg)

    def _goal_cb(self, goal_request: FollowJointTrajectory.Goal) -> int:
        # Check emergency stop first
        with self._estop_lock:
            if self._emergency_stopped:
                self.get_logger().error('Goal REJECTED: Emergency stop active. Run factory_reset + caphome.')
                return GoalResponse.REJECT
        
        if self.reject_without_js and self._last_js is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle) -> int:
        self._estop_pub.publish(Bool(data=True))
        with self._estop_lock:
            self._emergency_stopped = True
        self.get_logger().info('Action CANCEL → Emergency stop')
        return CancelResponse.ACCEPT

    # ---------- helpers ----------
    def _wait_for_joint_state(self) -> bool:
        t_end = time.monotonic() + self.startup_wait_sec
        while time.monotonic() < t_end and rclpy.ok():
            if self._last_js is not None:
                return True
            time.sleep(0.02)
        return self._last_js is not None

    def _get_pos(self, name: str, default: float = 0.0) -> float:
        return float(self._pos.get(name, default))

    def _publish_arm_cmd(self, cont: float, r1: float, r2: float, r3: Optional[float] = None):
        cont = clamp(cont, -self.cont_limit, self.cont_limit)
        msg = Float64MultiArray()
        msg.data = [float(cont), float(r1), float(r2)] if r3 is None else [float(cont), float(r1), float(r2), float(r3)]
        self._cmd_pub.publish(msg)

    def _publish_grip(self, r3: float):
        m = Float64()
        m.data = float(r3)
        self._grip_pub.publish(m)

    def _interp(self, t: float, t0: float, p0: float, t1: float, p1: float) -> float:
        if t1 <= t0:
            return p1
        u = (t - t0) / (t1 - t0)
        if u <= 0.0:
            return p0
        if u >= 1.0:
            return p1
        return p0 + u * (p1 - p0)

    def _within(self, desired: Dict[str, float], tol_map: Dict[str, float]) -> Tuple[bool, Dict[str, float]]:
        errs: Dict[str, float] = {}
        ok = True
        for j, des in desired.items():
            act = self._get_pos(j, 0.0)
            e = act - des
            errs[j] = e
            if abs(e) > tol_map.get(j, self.goal_tol):
                ok = False
        return ok, errs

    def _build_tol_map(self, joints: List[str]) -> Dict[str, float]:
        tol_map: Dict[str, float] = {}
        for i, j in enumerate(self.arm_joints):
            if i < len(self.arm_goal_tols):
                tol_map[j] = self.arm_goal_tols[i]
        for j in joints:
            tol_map.setdefault(j, self.goal_tol)
        return tol_map

    def _max_speed_over_window(self, joints: List[str], samples: List[Tuple[float, Dict[str, float]]]) -> float:
        if len(samples) < 2:
            return float("inf")
        t0, p0 = samples[0]
        t1, p1 = samples[-1]
        dt = max(1e-6, t1 - t0)

        vmax = 0.0
        for j in joints:
            a = p0.get(j, 0.0)
            b = p1.get(j, 0.0)
            dp = b - a
            if abs(dp) < self.stop_noise_eps:
                dp = 0.0
            v = abs(dp) / dt
            if v > vmax:
                vmax = v
        return vmax

    def _should_publish(self, last_cmd: Optional[Dict[str, float]], cmd: Dict[str, float]) -> bool:
        if last_cmd is None:
            return True
        for k, v in cmd.items():
            if abs(v - last_cmd.get(k, 0.0)) >= self.stream_change_eps:
                return True
        return False

    # ---------- action execution ----------
    def _exec_arm(self, goal_handle):
        # Check emergency stop at start
        with self._estop_lock:
            if self._emergency_stopped:
                self.get_logger().error('Execution aborted: Emergency stop active')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        if not self._wait_for_joint_state():
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        points = list(traj.points)

        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        include_r3 = "revolute3" in names

        pt_list: List[Tuple[float, Dict[str, float]]] = []
        for pt in points:
            t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            d: Dict[str, float] = {}
            for i, jn in enumerate(names):
                if i < len(pt.positions):
                    d[jn] = float(pt.positions[i])
            pt_list.append((t, d))

        total_t = pt_list[-1][0]

        # Timeout: keep based on original traj, but ensure a minimum when final_only
        timeout = total_t * self.reach_timeout_scaling + self.reach_timeout_margin
        if self.arm_command_mode == "final_only":
            timeout = max(timeout, self.min_final_timeout)

        # Current fallback (used if some joint missing in points)
        cur = {
            "continuous": self._get_pos("continuous", 0.0),
            "revolute1": self._get_pos("revolute1", 0.0),
            "revolute2": self._get_pos("revolute2", 0.0),
            "revolute3": self._get_pos("revolute3", 0.0),
        }

        # Final desired
        last_des = pt_list[-1][1]
        cont_f = last_des.get("continuous", cur["continuous"])
        r1_f = last_des.get("revolute1", cur["revolute1"])
        r2_f = last_des.get("revolute2", cur["revolute2"])
        r3_f = last_des.get("revolute3", cur["revolute3"]) if include_r3 else None

        # ---- COMMAND PHASE ----
        t_start_wall = time.monotonic()

        if self.arm_command_mode == "final_only":
            # Publish ONLY final goal once
            self._publish_arm_cmd(cont_f, r1_f, r2_f, r3_f)
        else:
            # stream_decimated: publish interpolated targets but at low rate
            pub_period = 1.0 / max(1.0, self.stream_rate_hz)
            last_pub = 0.0
            last_cmd: Optional[Dict[str, float]] = None
            t0_wall = time.monotonic()

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()

                now = time.monotonic()
                t = now - t0_wall
                if t > total_t:
                    break

                # find segment
                idx = 0
                while idx + 1 < len(pt_list) and pt_list[idx + 1][0] < t:
                    idx += 1

                tA, dA = pt_list[idx]
                tB, dB = pt_list[idx + 1] if idx + 1 < len(pt_list) else (tA, dA)

                def get_des(jn: str) -> float:
                    a = dA.get(jn, cur.get(jn, 0.0))
                    b = dB.get(jn, a)
                    return self._interp(t, tA, a, tB, b)

                cmd = {
                    "continuous": get_des("continuous"),
                    "revolute1": get_des("revolute1"),
                    "revolute2": get_des("revolute2"),
                }
                if include_r3:
                    cmd["revolute3"] = get_des("revolute3")

                do_pub = False
                if (now - last_pub) >= pub_period:
                    # publish at fixed low rate, but only if meaningful change
                    do_pub = self._should_publish(last_cmd, cmd)

                if do_pub:
                    self._publish_arm_cmd(
                        cmd["continuous"],
                        cmd["revolute1"],
                        cmd["revolute2"],
                        cmd.get("revolute3", None),
                    )
                    last_cmd = cmd
                    last_pub = now

                if (now - t_start_wall) > timeout:
                    self.get_logger().error("Arm trajectory timed out (bridge-side). Aborting goal.")
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()

                # Sleep lightly; do not force publish rate here
                time.sleep(min(self.check_dt, pub_period * 0.5))

            # Ensure final is sent
            self._publish_arm_cmd(cont_f, r1_f, r2_f, r3_f)

        # ---- VERIFY PHASE (within tol AND stopped) ----
        verify_joints = ["continuous", "revolute1", "revolute2"] + (["revolute3"] if include_r3 else [])
        tol_map = self._build_tol_map(verify_joints)

        desired = {"continuous": cont_f, "revolute1": r1_f, "revolute2": r2_f}
        if include_r3:
            desired["revolute3"] = float(r3_f) if r3_f is not None else self._get_pos("revolute3", 0.0)

        t_verify_start = time.monotonic()
        ok_since: Optional[float] = None
        stopped_since: Optional[float] = None
        last_debug = 0.0

        samples: List[Tuple[float, Dict[str, float]]] = []

        while rclpy.ok():
            now = time.monotonic()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            ok, errs = self._within(desired, tol_map)
            if ok:
                if ok_since is None:
                    ok_since = now
            else:
                ok_since = None

            # rolling samples for stop detection
            pos_snapshot = {j: self._get_pos(j, 0.0) for j in verify_joints}
            samples.append((now, pos_snapshot))
            t_cut = now - self.stop_window_sec
            while len(samples) > 2 and samples[0][0] < t_cut:
                samples.pop(0)

            vmax = self._max_speed_over_window(verify_joints, samples)
            stopped = (vmax <= self.vel_eps) if self.require_stopped else True

            if stopped:
                if stopped_since is None:
                    stopped_since = now
            else:
                stopped_since = None

            if self.verify_debug_every > 0.0 and (now - last_debug) >= self.verify_debug_every:
                last_debug = now
                self.get_logger().info(
                    "VERIFY: "
                    f"ok={ok} ok_for={(0.0 if ok_since is None else (now-ok_since)):.2f}s, "
                    f"stopped={stopped} vmax={vmax:.4f}rad/s stopped_for={(0.0 if stopped_since is None else (now-stopped_since)):.2f}s, "
                    + " ".join([f"{k}:err={errs[k]:+.3f}rad" for k in errs])
                )

            ok_long_enough = (ok_since is not None) and ((now - ok_since) >= self.settle_sec)
            stopped_long_enough = (not self.require_stopped) or (
                (stopped_since is not None) and ((now - stopped_since) >= self.stop_confirm_sec)
            )

            if ok_long_enough and stopped_long_enough:
                goal_handle.succeed()
                return FollowJointTrajectory.Result()

            if (now - t_verify_start) > timeout:
                self.get_logger().error(
                    "Goal not reached. "
                    + " ".join([f"{k}: err={errs[k]:+.4f}rad ({math.degrees(errs[k]):+.1f}deg)" for k in errs])
                    + f" | vmax={vmax:.4f}rad/s"
                )
                goal_handle.abort()
                return FollowJointTrajectory.Result()

            time.sleep(self.check_dt)

        goal_handle.abort()
        return FollowJointTrajectory.Result()

    def _exec_gripper(self, goal_handle):
        # Check emergency stop at start
        with self._estop_lock:
            if self._emergency_stopped:
                self.get_logger().error('Gripper execution aborted: Emergency stop active')
                goal_handle.abort()
                return FollowJointTrajectory.Result()

        if not self._wait_for_joint_state():
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        traj = goal_handle.request.trajectory
        points = list(traj.points)
        if not points:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        last = points[-1]
        if not last.positions:
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        r3 = float(last.positions[0])
        self._publish_grip(r3)

        t0 = time.monotonic()
        while (time.monotonic() - t0) < (self.settle_sec + 0.2):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            time.sleep(self.check_dt)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = MoveItTeensyBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

