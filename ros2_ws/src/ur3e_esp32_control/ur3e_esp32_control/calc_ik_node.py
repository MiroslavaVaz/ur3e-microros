#!/usr/bin/env python3
"""
calc_ik_node.py
---------------
Computes inverse kinematics for the UR3e robot given a target (x, y, z)
position and publishes the required joint angles.

This node is compatible with Gazebo and URSim simulations.

Subscriptions
  /ik_target  (geometry_msgs/Point)  — target position (x, y, z) in metres

Publications
  /ik_joint_state  (sensor_msgs/JointState)  — computed joint angles
  /target_joint_state (sensor_msgs/JointState) — same, for instruction node

Parameters
  initial_target_x  (float, default 0.3)   — x position of target
  initial_target_y  (float, default 0.0)   — y position of target
  initial_target_z  (float, default 0.3)   — z position of target

Usage
  # Set a target position via topic:
  ros2 topic pub --once /ik_target geometry_msgs/msg/Point \
      "{x: 0.3, y: 0.1, z: 0.3}"
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


# UR3e DH parameters (metres and radians)
# [a, d, alpha, theta_offset]
UR3E_DH = [
    [0.0,      0.15185,  math.pi / 2,  0.0],   # Joint 1 shoulder_pan
    [-0.24355, 0.0,      0.0,          0.0],   # Joint 2 shoulder_lift
    [-0.2132,  0.0,      0.0,          0.0],   # Joint 3 elbow
    [0.0,      0.13105,  math.pi / 2,  0.0],   # Joint 4 wrist_1
    [0.0,      0.08535, -math.pi / 2,  0.0],   # Joint 5 wrist_2
    [0.0,      0.0921,   0.0,          0.0],   # Joint 6 wrist_3
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

# UR3e joint limits (radians)
JOINT_LIMITS = [
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-math.pi,     math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
    (-2 * math.pi, 2 * math.pi),
]


def dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Compute a single DH transformation matrix."""
    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array([
        [ct,  -st * ca,  st * sa,  a * ct],
        [st,   ct * ca, -ct * sa,  a * st],
        [0.0,  sa,       ca,       d],
        [0.0,  0.0,      0.0,      1.0],
    ])


def forward_kinematics(joint_angles: list) -> np.ndarray:
    """
    Compute forward kinematics for the UR3e.
    Returns the 4x4 end-effector transformation matrix.
    """
    T = np.eye(4)
    for i, (a, d, alpha, theta_offset) in enumerate(UR3E_DH):
        theta = joint_angles[i] + theta_offset
        T = T @ dh_transform(a, d, alpha, theta)
    return T


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_ik(target_x: float, target_y: float, target_z: float,
               initial_angles: list = None) -> list:
    """
    Compute inverse kinematics using numerical gradient descent.

    Uses the Jacobian pseudoinverse method to iteratively solve
    for joint angles that place the end-effector at (x, y, z).

    Returns list of 6 joint angles in radians.
    """
    if initial_angles is None:
        # UR3e typical working configuration
        angles = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
    else:
        angles = list(initial_angles)

    target = np.array([target_x, target_y, target_z])

    max_iterations = 1000
    step_size = 0.5
    tolerance = 0.001      # 1 mm
    delta = 1e-6           # numerical differentiation step

    for iteration in range(max_iterations):
        # Current end-effector position
        T = forward_kinematics(angles)
        current_pos = T[:3, 3]

        error = target - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            break

        # Build Jacobian numerically
        J = np.zeros((3, NUM_JOINTS))
        for j in range(NUM_JOINTS):
            angles_plus = list(angles)
            angles_plus[j] += delta
            T_plus = forward_kinematics(angles_plus)
            J[:, j] = (T_plus[:3, 3] - current_pos) / delta

        # Pseudoinverse step
        J_pinv = np.linalg.pinv(J)
        delta_angles = step_size * J_pinv @ error

        # Update and clamp angles
        for j in range(NUM_JOINTS):
            angles[j] += delta_angles[j]
            low, high = JOINT_LIMITS[j]
            angles[j] = clamp(angles[j], low, high)

    # Final position check
    T_final = forward_kinematics(angles)
    final_pos = T_final[:3, 3]
    final_error = np.linalg.norm(target - final_pos)

    return angles, final_error


class CalcIKNode(Node):
    """
    Computes IK for a UR3e robot and publishes joint angles.

    Compatible with Gazebo and URSim — publishes sensor_msgs/JointState
    which is the standard interface for both simulators.
    """

    def __init__(self):
        super().__init__('calc_ik_node')

        # ---- parameters ------------------------------------------------
        self.declare_parameter('initial_target_x', 0.3)
        self.declare_parameter('initial_target_y', 0.0)
        self.declare_parameter('initial_target_z', 0.3)

        # ---- subscriptions ---------------------------------------------
        self._target_sub = self.create_subscription(
            Point,
            '/ik_target',
            self._target_callback,
            qos_profile=10,
        )

        # ---- publications ----------------------------------------------
        self._ik_pub = self.create_publisher(
            JointState,
            '/ik_joint_state',
            qos_profile=10,
        )
        self._target_pub = self.create_publisher(
            JointState,
            '/target_joint_state',
            qos_profile=10,
        )

        # ---- compute IK for initial target -----------------------------
        x = self.get_parameter('initial_target_x').get_parameter_value().double_value
        y = self.get_parameter('initial_target_y').get_parameter_value().double_value
        z = self.get_parameter('initial_target_z').get_parameter_value().double_value

        self.get_logger().info(
            f'CalcIKNode started — initial target: ({x:.3f}, {y:.3f}, {z:.3f})'
        )
        self._solve_and_publish(x, y, z)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _solve_and_publish(self, x: float, y: float, z: float) -> None:
        """Solve IK and publish resulting joint state."""
        self.get_logger().info(
            f'Computing IK for target: x={x:.4f}, y={y:.4f}, z={z:.4f} m'
        )

        angles, error = compute_ik(x, y, z)

        if error > 0.01:
            self.get_logger().warn(
                f'IK solution may be inaccurate — '
                f'residual error: {error * 1000:.2f} mm. '
                f'Target may be out of reach.'
            )
        else:
            self.get_logger().info(
                f'IK solved successfully — residual error: {error * 1000:.3f} mm'
            )

        # Log the solution
        lines = ['  IK solution (joint angles):']
        for name, angle in zip(JOINT_NAMES, angles):
            lines.append(
                f'    {name:<25} {angle:+.4f} rad  '
                f'({math.degrees(angle):+.2f} deg)'
            )
        self.get_logger().info('\n'.join(lines))

        # Build and publish JointState message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'   # Gazebo compatible
        msg.name = JOINT_NAMES
        msg.position = angles
        msg.velocity = [0.0] * NUM_JOINTS
        msg.effort = [0.0] * NUM_JOINTS

        self._ik_pub.publish(msg)
        self._target_pub.publish(msg)

    # ------------------------------------------------------------------
    # Callback
    # ------------------------------------------------------------------

    def _target_callback(self, msg: Point) -> None:
        """Called when a new target position arrives on /ik_target."""
        self.get_logger().info(
            f'New IK target received: '
            f'x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}'
        )
        self._solve_and_publish(msg.x, msg.y, msg.z)


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

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