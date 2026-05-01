#!/usr/bin/env python3
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

BUTTON_MAP = {
    'shoulder_pan_joint':  1,
    'shoulder_lift_joint': 2,
    'elbow_joint':         3,
    'wrist_1_joint':       4,
    'wrist_2_joint':       5,
    'wrist_3_joint':       6,
}

NUM_JOINTS = len(JOINT_NAMES)

REACHED_TOLERANCE = 0.05  # radians (~3 degrees)


class InstructionNode(Node):

    def __init__(self):
        super().__init__('instruction_node')

        self.declare_parameter('joint_step', 0.1)
        self.declare_parameter('tolerance', 0.25)

        self._current_positions = None
        self._target_positions = None
        self._new_target = False
        self._target_reached = False   
        self._current_sub = self.create_subscription(
            JointState, '/current_joint_state', self._current_callback, 10)
        self._target_sub = self.create_subscription(
            JointState, '/target_joint_state', self._target_callback, 10)

        self._instruction_pub = self.create_publisher(String, '/instructions', 10)

        self.get_logger().info('InstructionNode ready — waiting for target...')

    def _joint_step(self):
        return self.get_parameter('joint_step').get_parameter_value().double_value

    def _tolerance(self):
        return self.get_parameter('tolerance').get_parameter_value().double_value

    def _extract_positions(self, msg):
        positions = [0.0] * NUM_JOINTS
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
        return positions

    def _check_reached(self):
        """
        Compare current vs target using a tight tolerance.
        Announces once when all joints are within REACHED_TOLERANCE.
        Resets when a new target is set.
        """
        if self._current_positions is None or self._target_positions is None:
            return
        if self._target_reached:
            return 

        errors = [
            abs(c - t)
            for c, t in zip(self._current_positions, self._target_positions)
        ]

        if all(e < REACHED_TOLERANCE for e in errors):
            self._target_reached = True

            lines = []
            lines.append('\n' + '★' * 60)
            lines.append('  TARGET POSITION REACHED')
            lines.append('★' * 60)
            lines.append(f'  {"JOINT":<22} {"CURRENT":>9} {"TARGET":>9} {"ERROR":>8}')
            lines.append(f'  {"─"*22} {"─"*9} {"─"*9} {"─"*8}')
            for i, name in enumerate(JOINT_NAMES):
                c = self._current_positions[i]
                t = self._target_positions[i]
                e = abs(c - t)
                lines.append(
                    f'  {name:<22} {c:>+8.3f}  {t:>+8.3f}  {e:>7.4f}'
                )
            lines.append('★' * 60)
            lines.append('  Robot is ready for next command.')
            lines.append('★' * 60)

            text = '\n'.join(lines)
            self.get_logger().info(text)

            msg = String()
            msg.data = text
            self._instruction_pub.publish(msg)

    def _print_full_instructions(self):
        """Print full table once when new target arrives."""
        step = self._joint_step()
        tol = self._tolerance()
        lines = []
        lines.append('\n' + '=' * 60)
        lines.append('  TARGET SET — FULL BUTTON PRESS INSTRUCTIONS')
        lines.append('=' * 60)
        lines.append(f'  {"BUTTON":<8} {"JOINT":<22} {"PRESSES":>7}  {"FROM":>8} {"TO":>8}')
        lines.append(f'  {"─"*8} {"─"*22} {"─"*7}  {"─"*8} {"─"*8}')

        total = 0
        for i, name in enumerate(JOINT_NAMES):
            current = self._current_positions[i]
            target = self._target_positions[i]
            diff = target - current
            button = BUTTON_MAP[name]

            if abs(diff) <= tol:
                lines.append(
                    f'  PB{button:<6} {name:<22} {"✓ done":>7}  '
                    f'{current:>+7.3f} {target:>+7.3f}'
                )
            else:
                presses = int(round(abs(diff) / step))
                total += presses
                direction = 'FWD' if diff > 0 else 'REV'
                lines.append(
                    f'  PB{button:<6} {name:<22} {presses:>5}x [{direction}]  '
                    f'{current:>+7.3f} {target:>+7.3f}'
                )

        lines.append('─' * 60)
        lines.append(f'  Total presses needed: {total}')
        lines.append('=' * 60)

        text = '\n'.join(lines)
        self.get_logger().info(text)

        msg = String()
        msg.data = text
        self._instruction_pub.publish(msg)

    def _print_remaining(self):
        """Print only remaining presses — called after each button press."""
        step = self._joint_step()
        tol = self._tolerance()

        remaining = []
        all_done = True

        for i, name in enumerate(JOINT_NAMES):
            current = self._current_positions[i]
            target = self._target_positions[i]
            diff = target - current
            button = BUTTON_MAP[name]

            if abs(diff) > tol:
                presses = int(round(abs(diff) / step))
                direction = 'FWD' if diff > 0 else 'REV'
                remaining.append(f'  PB{button}  {presses}x [{direction}]  ({name})')
                all_done = False

        lines = ['\n' + '─' * 45]
        if all_done:
            lines.append('  ✓ ALL JOINTS AT TARGET — robot ready!')
        else:
            lines.append('  REMAINING:')
            lines.extend(remaining)
        lines.append('─' * 45)

        text = '\n'.join(lines)
        self.get_logger().info(text)

        msg = String()
        msg.data = text
        self._instruction_pub.publish(msg)

    def _current_callback(self, msg):
        self._current_positions = self._extract_positions(msg)
        if self._target_positions is not None and not self._new_target:
            self._print_remaining()
            self._check_reached()   

    def _target_callback(self, msg):
        self._target_positions = self._extract_positions(msg)
        self._target_reached = False   
        if self._current_positions is not None:
            self._new_target = True
            self._print_full_instructions()
            self._new_target = False


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