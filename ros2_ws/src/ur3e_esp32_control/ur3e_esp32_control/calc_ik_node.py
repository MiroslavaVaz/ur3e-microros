#!/usr/bin/env python3
"""
calc_ik_node.py
---------------
Computes inverse kinematics for the UR3e robot given a target (x, y, z)
position and publishes the required joint angles.

Improvements over previous version:
  - Seed is computed from target direction (points arm toward target)
  - Joint angle minimization penalty keeps angles small and natural
  - Multiple restarts pick the best solution found

Subscriptions
  /ik_target  (geometry_msgs/Point)  — target position (x, y, z) in metres

Publications
  /ik_joint_state     (sensor_msgs/JointState)  — computed joint angles
  /target_joint_state (sensor_msgs/JointState)  — same, for instruction node

Parameters
  initial_target_x  (float, default 0.3)
  initial_target_y  (float, default 0.0)
  initial_target_z  (float, default 0.3)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


# UR3e DH parameters [a, d, alpha, theta_offset]
UR3E_DH = [
    [0.0,      0.15185,  math.pi / 2,  0.0],
    [-0.24355, 0.0,      0.0,          0.0],
    [-0.2132,  0.0,      0.0,          0.0],
    [0.0,      0.13105,  math.pi / 2,  0.0],
    [0.0,      0.08535, -math.pi / 2,  0.0],
    [0.0,      0.0921,   0.0,          0.0],
]

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

# Preferred "neutral" angles — solver is penalized for deviating from these
# This is a natural elbow-up forward-reach pose
PREFERRED_ANGLES = [
    0.0,            # shoulder_pan   — straight ahead
    -math.pi / 4,   # shoulder_lift  — tilted forward slightly
    math.pi / 2,    # elbow          — elbow up
    -math.pi / 4,   # wrist_1        — level
    -math.pi / 2,   # wrist_2        — neutral
    0.0,            # wrist_3        — neutral
]


def dh_transform(a, d, alpha, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,  -st * ca,  st * sa,  a * ct],
        [st,   ct * ca, -ct * sa,  a * st],
        [0.0,  sa,       ca,       d],
        [0.0,  0.0,      0.0,      1.0],
    ])


def forward_kinematics(joint_angles):
    T = np.eye(4)
    for i, (a, d, alpha, theta_offset) in enumerate(UR3E_DH):
        T = T @ dh_transform(a, d, alpha, joint_angles[i] + theta_offset)
    return T


def clamp(value, low, high):
    return max(low, min(high, value))


def compute_ik_single(target_x, target_y, target_z, seed_angles):
    """
    Run one IK solve from a given seed.
    Returns (angles, error).
    Uses position error + small joint angle penalty to prefer natural poses.
    """
    angles = list(seed_angles)
    target = np.array([target_x, target_y, target_z])

    max_iterations = 2000
    step_size = 0.3
    tolerance = 0.001
    delta = 1e-6
    penalty_weight = 0.01   # small penalty to keep angles near preferred

    for _ in range(max_iterations):
        T = forward_kinematics(angles)
        current_pos = T[:3, 3]
        error = target - current_pos

        if np.linalg.norm(error) < tolerance:
            break

        # Numerical Jacobian
        J = np.zeros((3, NUM_JOINTS))
        for j in range(NUM_JOINTS):
            ap = list(angles)
            ap[j] += delta
            J[:, j] = (forward_kinematics(ap)[:3, 3] - current_pos) / delta

        J_pinv = np.linalg.pinv(J)
        delta_angles = step_size * J_pinv @ error

        # Add small penalty pulling joints toward preferred angles
        for j in range(NUM_JOINTS):
            delta_angles[j] -= penalty_weight * (angles[j] - PREFERRED_ANGLES[j])

        for j in range(NUM_JOINTS):
            angles[j] = clamp(
                angles[j] + delta_angles[j],
                JOINT_LIMITS[j][0],
                JOINT_LIMITS[j][1]
            )

    T_final = forward_kinematics(angles)
    final_error = np.linalg.norm(target - T_final[:3, 3])
    return angles, final_error


def compute_ik(target_x, target_y, target_z):
    """
    Try multiple seeds and return the best solution.
    Seeds are designed to cover natural arm configurations.
    """
    pan = math.atan2(target_y, target_x)

    seeds = [
        # Seed 1 — natural elbow up, arm pointing at target
        [pan, -math.pi / 4,  math.pi / 2, -math.pi / 4, -math.pi / 2, 0.0],
        # Seed 2 — arm more extended
        [pan, -math.pi / 6,  math.pi / 3, -math.pi / 6, -math.pi / 2, 0.0],
        # Seed 3 — arm reaching forward and down
        [pan, -math.pi / 3,  math.pi / 2, -math.pi / 6, -math.pi / 2, 0.0],
        # Seed 4 — standard UR home-like position
        [pan, -math.pi / 2,  math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0],
        # Seed 5 — slightly offset pan
        [pan + 0.2, -math.pi / 4, math.pi / 2, -math.pi / 4, -math.pi / 2, 0.0],
    ]

    best_angles = None
    best_error = float('inf')

    for seed in seeds:
        angles, error = compute_ik_single(target_x, target_y, target_z, seed)
        # Score = position error + penalty for large joint angles
        joint_penalty = sum(
            abs(a - p) for a, p in zip(angles, PREFERRED_ANGLES)
        ) * 0.01
        score = error + joint_penalty

        if score < best_error:
            best_error = score
            best_angles = angles

    # Final true position error for reporting
    T_final = forward_kinematics(best_angles)
    true_error = np.linalg.norm(
        np.array([target_x, target_y, target_z]) - T_final[:3, 3]
    )
    return best_angles, true_error


class CalcIKNode(Node):

    def __init__(self):
        super().__init__('calc_ik_node')

        self.declare_parameter('initial_target_x', 0.3)
        self.declare_parameter('initial_target_y', 0.0)
        self.declare_parameter('initial_target_z', 0.3)

        self._target_sub = self.create_subscription(
            Point, '/ik_target', self._target_callback, 10)

        self._ik_pub = self.create_publisher(
            JointState, '/ik_joint_state', 10)
        self._target_pub = self.create_publisher(
            JointState, '/target_joint_state', 10)

        x = self.get_parameter('initial_target_x').get_parameter_value().double_value
        y = self.get_parameter('initial_target_y').get_parameter_value().double_value
        z = self.get_parameter('initial_target_z').get_parameter_value().double_value

        self.get_logger().info(
            f'CalcIKNode started — initial target: ({x:.3f}, {y:.3f}, {z:.3f})'
        )
        self._solve_and_publish(x, y, z)

    def _solve_and_publish(self, x, y, z):
        self.get_logger().info(
            f'Computing IK for target: x={x:.4f}, y={y:.4f}, z={z:.4f} m'
        )

        angles, error = compute_ik(x, y, z)

        if error > 0.01:
            self.get_logger().warn(
                f'IK may be inaccurate — residual: {error * 1000:.2f} mm. '
                f'Target may be out of reach.'
            )
        else:
            self.get_logger().info(
                f'IK solved successfully — residual: {error * 1000:.3f} mm'
            )

        lines = ['  IK solution (joint angles):']
        for name, angle in zip(JOINT_NAMES, angles):
            lines.append(
                f'    {name:<25} {angle:+.4f} rad  ({math.degrees(angle):+.2f} deg)'
            )
        self.get_logger().info('\n'.join(lines))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.name = JOINT_NAMES
        msg.position = angles
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS

        self._ik_pub.publish(msg)
        self._target_pub.publish(msg)

    def _target_callback(self, msg):
        self.get_logger().info(
            f'New IK target: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}'
        )
        self._solve_and_publish(msg.x, msg.y, msg.z)


def main(args=None):
    rclpy.init(args=args)
    node = CalcIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()