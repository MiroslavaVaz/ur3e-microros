#!/usr/bin/env python3
"""
joint_state_tracker.py
----------------------
Maintains the current joint state of the UR3e and increments or
decrements the appropriate joint each time a pushbutton event arrives.

Button mapping:
  PB1-PB6        -> increment/decrement joint 1-6 (direction depends on mode)
  PB7 short (0)  -> toggle forward/reverse direction mode
  PB7 long  (9)  -> EXECUTE: send all joints to Gazebo simultaneously

Subscriptions
  /joint_button  (std_msgs/Int32)  - button ID from ESP32

Publications
  /current_joint_state  (sensor_msgs/JointState)  - live joint angles
  /execute_motion       (sensor_msgs/JointState)  - triggered on long press PB7
  /direction_mode       (std_msgs/String)          - current mode

Parameters
  joint_step  (float, default 0.5)  - radians per button press
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState


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


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class JointStateTracker(Node):

    def __init__(self):
        super().__init__('joint_state_tracker')

        self.declare_parameter('joint_step', 0.5)
        self._positions = [0.0] * NUM_JOINTS
        self._press_counts = [0] * NUM_JOINTS
        self._forward_mode = True   # True = forward (+), False = reverse (-)

        self._button_sub = self.create_subscription(
            Int32,
            '/joint_button',
            self._button_callback,
            qos_profile=10,
        )

        self._current_pub = self.create_publisher(
            JointState, '/current_joint_state', qos_profile=10)

        # New publisher — only fires on long press PB7
        self._execute_pub = self.create_publisher(
            JointState, '/execute_motion', qos_profile=10)

        self._mode_pub = self.create_publisher(
            String, '/direction_mode', qos_profile=10)

        self._publish_state()
        self._publish_mode()

        self.get_logger().info(
            f'JointStateTracker started — '
            f'step={self._joint_step():.3f} rad | mode=FORWARD'
        )
        self.get_logger().info(
            'PB1-PB6: move joints | PB7 short: toggle direction | '
            'PB7 long (3s): EXECUTE to Gazebo'
        )

    def _joint_step(self) -> float:
        return self.get_parameter('joint_step').get_parameter_value().double_value

    def _publish_state(self) -> None:
        """Publishes current joint positions — does NOT trigger Gazebo motion."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._positions)
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS
        self._current_pub.publish(msg)

    def _publish_execute(self) -> None:
        """Publishes execute signal — THIS triggers Gazebo motion."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._positions)
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS
        self._execute_pub.publish(msg)

        # Log the full state being sent
        lines = ['\n' + '>' * 60]
        lines.append('  EXECUTE — sending all joints to Gazebo:')
        lines.append('>' * 60)
        for i, (name, pos) in enumerate(zip(JOINT_NAMES, self._positions)):
            lines.append(
                f'  [{i+1}] {name:<25} {pos:+.4f} rad '
                f'({math.degrees(pos):+.2f} deg)'
            )
        lines.append('>' * 60)
        self.get_logger().info('\n'.join(lines))

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = 'FORWARD' if self._forward_mode else 'REVERSE'
        self._mode_pub.publish(msg)

    def _log_state(self, changed_idx: int) -> None:
        step = self._joint_step()
        direction = 'FORWARD' if self._forward_mode else 'REVERSE'
        lines = [
            f'  [{direction}] Joint state after PB{changed_idx + 1} press '
            f'(step={step:.3f} rad, '
            f'total presses: {self._press_counts[changed_idx]}):'
        ]
        for i, (name, pos) in enumerate(zip(JOINT_NAMES, self._positions)):
            marker = ' <-- changed' if i == changed_idx else ''
            lines.append(f'    [{i+1}] {name:<25} {pos:+.4f} rad{marker}')
        self.get_logger().info('\n'.join(lines))

    def _button_callback(self, msg: Int32) -> None:
        button_id = msg.data

        # ── PB7 short press — direction toggle ────────────────────────────────
        if button_id == 0:
            self._forward_mode = not self._forward_mode
            mode_str = 'FORWARD' if self._forward_mode else 'REVERSE'
            self.get_logger().info(f'Direction toggled -> {mode_str}')
            self._publish_mode()
            return

        # ── PB7 long press — EXECUTE all joints to Gazebo ────────────────────
        if button_id == 9:
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info('  LONG PRESS PB7 — SENDING COMMAND TO GAZEBO')
            self.get_logger().info('=' * 60 + '\n')
            self._publish_execute()
            return

        # ── PB1-PB6 — update joint positions internally ───────────────────────
        if button_id < 1 or button_id > NUM_JOINTS:
            self.get_logger().warn(f'Unknown button value: {button_id}. Ignoring.')
            return

        joint_idx = button_id - 1
        step = self._joint_step()

        delta = step if self._forward_mode else -step
        new_pos = self._positions[joint_idx] + delta
        low, high = JOINT_LIMITS[joint_idx]
        clamped = _clamp(new_pos, low, high)

        if clamped != new_pos:
            self.get_logger().warn(
                f'Joint {JOINT_NAMES[joint_idx]} hit limit. Clamping.'
            )

        self._positions[joint_idx] = clamped
        self._press_counts[joint_idx] += 1

        self._log_state(joint_idx)
        self._publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = JointStateTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()