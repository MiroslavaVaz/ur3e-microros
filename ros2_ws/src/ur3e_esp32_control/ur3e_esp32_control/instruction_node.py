#!/usr/bin/env python3
"""
instruction_node.py
-------------------
Compares the current joint state (from joint_state_tracker) with the
target joint state (from calc_ik_node) and outputs how many times each
pushbutton needs to be pressed to reach the target.

Subscriptions
  /current_joint_state  (sensor_msgs/JointState)  — current joint angles
  /target_joint_state   (sensor_msgs/JointState)  — target joint angles

Publications
  /instructions  (std_msgs/String)  — human-readable press instructions

Parameters
  joint_step  (float, default 0.1)  — radians per button press
  tolerance   (float, default 0.05) — radians within which joint is "done"
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Button number for each joint (1-based)
BUTTON_MAP = {
    'shoulder_pan_joint':  1,
    'shoulder_lift_joint': 2,
    'elbow_joint':         3,
    'wrist_1_joint':       4,
    'wrist_2_joint':       5,
    'wrist_3_joint':       6,
}

NUM_JOINTS = len(JOINT_NAMES)


class InstructionNode(Node):
    """
    Compares current vs target joint state and outputs button press
    instructions to reach the target position.

    Output example:
        Press PB1 5 times   (shoulder_pan_joint:  +0.00 → +0.50 rad)
        Press PB3 10 times  (elbow_joint:          +0.00 → +1.00 rad)
        Joint 2 already at target ✓
    """

    def __init__(self):
        super().__init__('instruction_node')

        # ---- parameters ------------------------------------------------
        self.declare_parameter('joint_step', 0.1)
        self.declare_parameter('tolerance', 0.05)

        # ---- state -----------------------------------------------------
        self._current_positions = None
        self._target_positions = None

        # ---- subscriptions ---------------------------------------------
        self._current_sub = self.create_subscription(
            JointState,
            '/current_joint_state',
            self._current_callback,
            qos_profile=10,
        )
        self._target_sub = self.create_subscription(
            JointState,
            '/target_joint_state',
            self._target_callback,
            qos_profile=10,
        )

        # ---- publications ----------------------------------------------
        self._instruction_pub = self.create_publisher(
            String,
            '/instructions',
            qos_profile=10,
        )

        self.get_logger().info('InstructionNode started — waiting for joint states...')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _joint_step(self) -> float:
        return self.get_parameter('joint_step').get_parameter_value().double_value

    def _tolerance(self) -> float:
        return self.get_parameter('tolerance').get_parameter_value().double_value

    def _compute_and_publish(self) -> None:
        """Compare current vs target and publish press instructions."""
        if self._current_positions is None or self._target_positions is None:
            return

        step = self._joint_step()
        tol = self._tolerance()

        lines = []
        lines.append('=' * 55)
        lines.append('  BUTTON PRESS INSTRUCTIONS')
        lines.append('=' * 55)

        all_done = True
        total_presses = 0

        for i, name in enumerate(JOINT_NAMES):
            current = self._current_positions[i]
            target = self._target_positions[i]
            diff = target - current
            button = BUTTON_MAP[name]

            if abs(diff) <= tol:
                lines.append(
                    f'  PB{button} ({name:<22}) ✓ already at target'
                )
            else:
                presses = int(round(abs(diff) / step))
                direction = '+' if diff > 0 else '-'
                total_presses += presses
                all_done = False
                lines.append(
                    f'  Press PB{button} {presses:>3} times  '
                    f'({name:<22} '
                    f'{current:+.3f} → {target:+.3f} rad  '
                    f'[{direction}{abs(diff):.3f} rad])'
                )

        lines.append('-' * 55)

        if all_done:
            lines.append('  ✓ ALL JOINTS AT TARGET — robot ready!')
        else:
            lines.append(f'  Total presses needed: {total_presses}')

        lines.append('=' * 55)

        instruction_text = '\n'.join(lines)

        # Publish to topic
        msg = String()
        msg.data = instruction_text
        self._instruction_pub.publish(msg)

        # Also log to terminal
        self.get_logger().info('\n' + instruction_text)

    def _extract_positions(self, msg: JointState) -> list:
        """Extract positions in JOINT_NAMES order from a JointState message."""
        positions = [0.0] * NUM_JOINTS
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        return positions

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _current_callback(self, msg: JointState) -> None:
        """Update current joint state."""
        self._current_positions = self._extract_positions(msg)
        self._compute_and_publish()

    def _target_callback(self, msg: JointState) -> None:
        """Update target joint state and recompute instructions."""
        self._target_positions = self._extract_positions(msg)
        self.get_logger().info('New target joint state received — recomputing instructions...')
        self._compute_and_publish()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = InstructionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()