#!/usr/bin/env python3
"""
direct_control_node.py
----------------------
Direct pushbutton control of the UR3e Gazebo simulation.
Every button press immediately moves the corresponding joint in Gazebo.
No accumulation, no long press needed.

Button mapping:
  PB1 (value=1) -> shoulder_pan_joint
  PB2 (value=2) -> shoulder_lift_joint
  PB3 (value=3) -> elbow_joint
  PB4 (value=4) -> wrist_1_joint
  PB5 (value=5) -> wrist_2_joint
  PB6 (value=6) -> wrist_3_joint
  PB7 short (value=0) -> toggle forward/reverse direction
  PB7 long  (value=9) -> go to safe home position

Subscriptions
  /joint_button  (std_msgs/Int32)
  /joint_states  (sensor_msgs/JointState) - Gazebo feedback

Publications
  /joint_trajectory_controller/joint_trajectory
  /current_joint_state  (sensor_msgs/JointState)
  /direction_mode       (std_msgs/String)

Parameters
  joint_step       (float, default 0.1)  - radians per press
  motion_duration  (float, default 0.1)  - seconds per move

Usage
  ros2 run ur3e_esp32_control direct_control_node
  ros2 run ur3e_esp32_control direct_control_node --ros-args -p joint_step:=0.2
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

NUM_JOINTS = len(JOINT_NAMES)

JOINT_LIMITS = [
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-math.pi,     math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
]

# Safe home — arm raised up and away from table
SAFE_HOME = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

CONTROLLER_TOPIC = "/joint_trajectory_controller/joint_trajectory"


def clamp(value, low, high):
    return max(low, min(high, value))


class DirectControlNode(Node):

    def __init__(self):
        super().__init__('direct_control_node')

        self.declare_parameter('joint_step', 0.1)
        self.declare_parameter('motion_duration', 0.1)

        self.joint_step = (
            self.get_parameter('joint_step').get_parameter_value().double_value
        )
        self.motion_duration = (
            self.get_parameter('motion_duration').get_parameter_value().double_value
        )

        self._positions = [0.0] * NUM_JOINTS
        self._forward_mode = True
        self._gazebo_synced = False

        self._traj_pub = self.create_publisher(
            JointTrajectory, CONTROLLER_TOPIC, 10)
        self._state_pub = self.create_publisher(
            JointState, '/current_joint_state', 10)
        self._mode_pub = self.create_publisher(
            String, '/direction_mode', 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._button_sub = self.create_subscription(
            Int32, '/joint_button', self._button_callback, 10)

        self._gazebo_sub = self.create_subscription(
            JointState, '/joint_states', self._gazebo_sync_callback, qos)

        self.get_logger().info("=" * 60)
        self.get_logger().info("direct_control_node started")
        self.get_logger().info(f"  Step    : {self.joint_step} rad per press")
        self.get_logger().info(f"  Duration: {self.motion_duration}s per move")
        self.get_logger().info("  Syncing with Gazebo...")
        self.get_logger().info("=" * 60)

    # ── Continuously sync positions from Gazebo ───────────────────────────────

    def _gazebo_sync_callback(self, msg: JointState):
        if len(msg.name) == 0 or len(msg.position) == 0:
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        for i, joint in enumerate(JOINT_NAMES):
            if joint in name_to_pos:
                self._positions[i] = name_to_pos[joint]

        if not self._gazebo_synced:
            self._gazebo_synced = True
            self.get_logger().info("Synced with Gazebo — Ready for button presses.")
            self.get_logger().info(
                "PB1-PB6: move joints | "
                "PB7 short: toggle direction | "
                "PB7 long: safe HOME"
            )

    # ── Button callback ───────────────────────────────────────────────────────

    def _button_callback(self, msg: Int32):
        button_id = msg.data

        # PB7 short — direction toggle
        if button_id == 0:
            self._forward_mode = not self._forward_mode
            mode_str = 'FORWARD' if self._forward_mode else 'REVERSE'
            self.get_logger().info(f'Direction -> {mode_str}')
            mode_msg = String()
            mode_msg.data = mode_str
            self._mode_pub.publish(mode_msg)
            return

        # PB7 long — safe home
        if button_id == 9:
            self.get_logger().info(
                '\n' + '=' * 60 +
                '\n  HOME — moving to safe position' +
                '\n' + '=' * 60
            )
            self._positions = list(SAFE_HOME)
            self._send_to_gazebo(duration_sec=3.0)
            self._publish_state()
            return

        # PB1-PB6 — move joint
        if button_id < 1 or button_id > NUM_JOINTS:
            self.get_logger().warn(f'Unknown button: {button_id}')
            return

        joint_idx = button_id - 1
        delta = self.joint_step if self._forward_mode else -self.joint_step
        new_pos = self._positions[joint_idx] + delta
        lo, hi = JOINT_LIMITS[joint_idx]
        clamped = clamp(new_pos, lo, hi)

        if abs(clamped - new_pos) > 1e-6:
            self.get_logger().warn(
                f'{JOINT_NAMES[joint_idx]} at limit — clamping.'
            )

        self._positions[joint_idx] = clamped

        direction = 'FWD' if self._forward_mode else 'REV'
        self.get_logger().info(
            f'PB{button_id} [{direction}] '
            f'{JOINT_NAMES[joint_idx]}: {clamped:+.4f} rad '
            f'({math.degrees(clamped):+.2f} deg)'
        )

        self._send_to_gazebo()
        self._publish_state()

    # ── Send trajectory to Gazebo ─────────────────────────────────────────────

    def _send_to_gazebo(self, duration_sec=None):
        if duration_sec is None:
            duration_sec = self.motion_duration

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = list(self._positions)
        point.velocities = [0.0] * NUM_JOINTS
        point.accelerations = [0.0] * NUM_JOINTS

        total_ns = int(duration_sec * 1e9)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=total_ns % 1_000_000_000,
        )

        msg.points = [point]
        self._traj_pub.publish(msg)

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._positions)
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DirectControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()