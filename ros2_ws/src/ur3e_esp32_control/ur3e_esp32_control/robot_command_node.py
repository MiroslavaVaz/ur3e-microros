#!/usr/bin/env python3
"""
robot_command_node.py
Bridges /execute_motion (triggered by PB7 long press) to the UR3e
Gazebo Classic simulation. Only moves the robot when explicitly triggered.

Target controller:
  /joint_trajectory_controller/joint_trajectory
  (trajectory_msgs/JointTrajectory)

Usage:
  ros2 run ur3e_esp32_control robot_command_node
  ros2 run ur3e_esp32_control robot_command_node --ros-args -p motion_duration:=3.0
  ros2 run ur3e_esp32_control robot_command_node --ros-args -p effort_threshold:=50.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Duration


UR3E_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_LIMITS = [
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-3.1416, 3.1416),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
    (-6.2832, 6.2832),
]

CONTROLLER_TOPIC = "/joint_trajectory_controller/joint_trajectory"


class RobotCommandNode(Node):

    def __init__(self):
        super().__init__("robot_command_node")

        self.declare_parameter("motion_duration", 3.0)
        self.declare_parameter("effort_threshold", 40.0)

        self.motion_duration = (
            self.get_parameter("motion_duration").get_parameter_value().double_value
        )
        self.effort_threshold = (
            self.get_parameter("effort_threshold").get_parameter_value().double_value
        )

        self.safety_stop = False
        self.last_efforts = [0.0] * 6

        # ── Publishers ────────────────────────────────────────────────────────
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            CONTROLLER_TOPIC,
            10,
        )
        self.safety_pub = self.create_publisher(
            String,
            "/robot_safety_status",
            10,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Only fires on PB7 long press — this is what moves the robot
        self.execute_sub = self.create_subscription(
            JointState,
            "/execute_motion",
            self.execute_callback,
            qos,
        )

        # Gazebo joint feedback for effort monitoring
        self.gazebo_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.gazebo_state_callback,
            qos,
        )

        # Safety reset
        self.reset_sub = self.create_subscription(
            Bool,
            "/reset_safety",
            self.reset_callback,
            10,
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("robot_command_node started")
        self.get_logger().info(f"  Waiting for: /execute_motion (PB7 long press)")
        self.get_logger().info(f"  Sending to : {CONTROLLER_TOPIC}")
        self.get_logger().info(f"  Duration   : {self.motion_duration}s per move")
        self.get_logger().info(f"  Effort limit: {self.effort_threshold} Nm")
        self.get_logger().info("=" * 60)

    # ── Safety monitoring ─────────────────────────────────────────────────────

    def gazebo_state_callback(self, msg: JointState):
        if len(msg.effort) == 0:
            return

        name_to_effort = dict(zip(msg.name, msg.effort))

        for i, joint in enumerate(UR3E_JOINT_NAMES):
            if joint not in name_to_effort:
                continue
            effort = abs(name_to_effort[joint])
            self.last_efforts[i] = effort

            if effort > self.effort_threshold and not self.safety_stop:
                self.safety_stop = True
                self._trigger_safety_stop(joint, effort)

    def _trigger_safety_stop(self, joint: str, effort: float):
        lines = [
            "\n" + "!" * 60,
            "  SAFETY STOP TRIGGERED",
            f"  Joint    : {joint}",
            f"  Effort   : {effort:.2f} Nm  (limit: {self.effort_threshold:.1f} Nm)",
            "  All commands BLOCKED until reset.",
            "  To reset: ros2 topic pub --once /reset_safety std_msgs/msg/Bool 'data: true'",
            "!" * 60,
        ]
        text = "\n".join(lines)
        self.get_logger().error(text)
        msg = String()
        msg.data = text
        self.safety_pub.publish(msg)

    def reset_callback(self, msg: Bool):
        if msg.data:
            self.safety_stop = False
            self.get_logger().info(
                "\n" + "=" * 60 +
                "\n  SAFETY RESET — commands re-enabled." +
                "\n" + "=" * 60
            )
            status = String()
            status.data = "SAFETY RESET — commands re-enabled."
            self.safety_pub.publish(status)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _clamp_positions(self, positions: list[float]) -> list[float]:
        clamped = []
        for i, pos in enumerate(positions):
            lo, hi = JOINT_LIMITS[i]
            c = max(lo, min(hi, pos))
            if abs(c - pos) > 1e-6:
                self.get_logger().warn(
                    f"  CLAMP {UR3E_JOINT_NAMES[i]}: {pos:.4f} -> {c:.4f} rad"
                )
            clamped.append(c)
        return clamped

    def _extract_positions(self, msg: JointState) -> list[float] | None:
        if len(msg.name) == 0:
            if len(msg.position) != 6:
                self.get_logger().warn(
                    f"Anonymous JointState has {len(msg.position)} positions "
                    f"(expected 6) — skipping."
                )
                return None
            return list(msg.position)

        name_to_pos = dict(zip(msg.name, msg.position))
        positions = []
        for joint in UR3E_JOINT_NAMES:
            if joint not in name_to_pos:
                self.get_logger().warn(
                    f"Joint '{joint}' missing from execute_motion message."
                )
                return None
            positions.append(name_to_pos[joint])
        return positions

    def _build_trajectory(self, positions: list[float]) -> JointTrajectory:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = UR3E_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6

        total_ns = int(self.motion_duration * 1e9)
        point.time_from_start = Duration(
            sec=int(self.motion_duration),
            nanosec=total_ns % 1_000_000_000,
        )

        msg.points = [point]
        return msg

    # ── Execute callback — only fires on PB7 long press ──────────────────────

    def execute_callback(self, msg: JointState):
        if self.safety_stop:
            self.get_logger().warn(
                "Execute BLOCKED — safety stop active. "
                "Publish 'true' to /reset_safety to resume."
            )
            return

        self.get_logger().info('\n' + '★' * 60)
        self.get_logger().info('  SENDING COMMAND TO GAZEBO — ROBOT MOVING')
        self.get_logger().info('★' * 60 + '\n')

        positions = self._extract_positions(msg)
        if positions is None:
            return

        positions = self._clamp_positions(positions)
        trajectory = self._build_trajectory(positions)
        self.trajectory_pub.publish(trajectory)

        short_names = ["pan", "lift", "elbow", "w1", "w2", "w3"]
        pos_str = "  ".join(
            f"{n}={p:+.3f}" for n, p in zip(short_names, positions)
        )
        self.get_logger().info(f"-> Gazebo EXECUTE | {pos_str}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()