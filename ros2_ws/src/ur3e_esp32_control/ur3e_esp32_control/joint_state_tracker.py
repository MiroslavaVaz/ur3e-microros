#!/usr/bin/env python3
"""
joint_state_tracker.py
----------------------
Maintains the current joint state of the UR3e and increments the
appropriate joint each time a pushbutton event arrives.

Subscriptions
  /joint_button  (std_msgs/Int32)  — button ID from ESP32

Publications
  /current_joint_state  (sensor_msgs/JointState)  — live joint angles
  /target_joint_state   (sensor_msgs/JointState)  — same as current
                                                     (used by downstream
                                                      nodes as target)

Parameters (settable via ROS 2 param or launch file)
  joint_step  (float, default 0.1)  — radians added per button press
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState


# UR3e joint names in DH / ROS convention order
JOINT_NAMES = [
    'shoulder_pan_joint',   # index 0  →  PB1
    'shoulder_lift_joint',  # index 1  →  PB2
    'elbow_joint',          # index 2  →  PB3
    'wrist_1_joint',        # index 3  →  PB4
    'wrist_2_joint',        # index 4  →  PB5
    'wrist_3_joint',        # index 5  →  PB6
]

NUM_JOINTS = len(JOINT_NAMES)

# Joint position limits (radians) — UR3e hardware limits
JOINT_LIMITS = [
    (-2 * math.pi, 2 * math.pi),   # shoulder_pan
    (-2 * math.pi, 2 * math.pi),   # shoulder_lift
    (-math.pi,     math.pi),        # elbow
    (-2 * math.pi, 2 * math.pi),   # wrist_1
    (-2 * math.pi, 2 * math.pi),   # wrist_2
    (-2 * math.pi, 2 * math.pi),   # wrist_3
]


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp value to [low, high]."""
    return max(low, min(high, value))


class JointStateTracker(Node):
    """
    Tracks UR3e joint positions and increments them on button events.

    Each press of PB{n} adds `joint_step` radians to joint n-1.
    The updated state is published immediately so downstream nodes
    (IK node, instruction node, robot command node) can react.
    """

    def __init__(self):
        super().__init__('joint_state_tracker')

        # ---- parameters ------------------------------------------------
        self.declare_parameter('joint_step', 0.1)   # radians per press

        # ---- state -----------------------------------------------------
        self._positions = [0.0] * NUM_JOINTS        # start at home (zeros)
        self._press_counts = [0] * NUM_JOINTS        # cumulative presses

        # ---- subscriptions ---------------------------------------------
        self._button_sub = self.create_subscription(
            Int32,
            '/joint_button',
            self._button_callback,
            qos_profile=10,
        )

        # ---- publications ----------------------------------------------
        self._current_pub = self.create_publisher(
            JointState,
            '/current_joint_state',
            qos_profile=10,
        )
        self._target_pub = self.create_publisher(
            JointState,
            '/target_joint_state',
            qos_profile=10,
        )

        # Publish the initial (all-zero) state so other nodes have
        # something to read right away.
        self._publish_state()

        self.get_logger().info(
            f'JointStateTracker started — '
            f'joint_step={self._joint_step():.3f} rad'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _joint_step(self) -> float:
        return self.get_parameter('joint_step').get_parameter_value().double_value

    def _publish_state(self) -> None:
        """Build and publish a JointState message on both topics."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = list(self._positions)
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS

        self._current_pub.publish(msg)
        self._target_pub.publish(msg)

    def _log_state(self, changed_idx: int) -> None:
        """Pretty-print the full joint state to the ROS logger."""
        step = self._joint_step()
        lines = [
            f'  Joint state after PB{changed_idx + 1} press '
            f'(step={step:.3f} rad, '
            f'total presses on this joint: {self._press_counts[changed_idx]}):'
        ]
        for i, (name, pos) in enumerate(zip(JOINT_NAMES, self._positions)):
            marker = ' <-- incremented' if i == changed_idx else ''
            lines.append(f'    [{i+1}] {name:<25} {pos:+.4f} rad{marker}')
        self.get_logger().info('\n'.join(lines))

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _button_callback(self, msg: Int32) -> None:
        """Increment the joint that corresponds to the pressed button."""
        button_id = msg.data

        # Validate: expect 1–6
        if button_id < 1 or button_id > NUM_JOINTS:
            self.get_logger().warn(
                f'Received out-of-range button value: {button_id}. '
                f'Expected 1–{NUM_JOINTS}. Ignoring.'
            )
            return

        joint_idx = button_id - 1          # convert 1-based to 0-based
        step = self._joint_step()

        # Increment and clamp
        new_pos = self._positions[joint_idx] + step
        low, high = JOINT_LIMITS[joint_idx]
        clamped = _clamp(new_pos, low, high)

        if clamped != new_pos:
            self.get_logger().warn(
                f'Joint {JOINT_NAMES[joint_idx]} hit limit '
                f'({low:.2f}, {high:.2f}). '
                f'Clamping {new_pos:.4f} → {clamped:.4f} rad.'
            )

        self._positions[joint_idx] = clamped
        self._press_counts[joint_idx] += 1

        self._log_state(joint_idx)
        self._publish_state()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

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